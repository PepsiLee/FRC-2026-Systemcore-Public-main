package com.team11855.frc2026.subsystems.vision;

import com.team11855.frc2026.Constants;
import com.team11855.frc2026.Constants.VisionConstants;
import com.team11855.frc2026.RobotState;
import com.team11855.lib.time.RobotTime;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** One camera: MT2 translation + MT1 heading, forwarded once through RobotState. */
public class VisionSubsystem extends SubsystemBase {
    public enum RejectionReason {
        ACCEPTED,
        DISABLED,
        MISSING_MT1,
        MISSING_MT2,
        INVALID_TIMESTAMP,
        STALE,
        UNSYNCHRONIZED,
        DUPLICATE_OR_OLDER,
        INVALID_POSE,
        OUTSIDE_FIELD,
        TAG_TOO_FAR,
        ROTATING_TOO_FAST,
        EXCLUSIVE_TAG_MISMATCH
    }

    private final VisionIO io;
    private final RobotState state;
    private final DoubleSupplier clock;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    private boolean useVision = true;
    private double lastAcceptedMt1Timestamp;
    private RejectionReason lastRejectionReason = RejectionReason.MISSING_MT1;

    public VisionSubsystem(VisionIO io, RobotState state) {
        this(io, state, RobotTime::getTimestampSeconds);
    }

    // 測試可提供固定時鐘；實機、模擬與 replay 皆沿用 RobotTime。
    VisionSubsystem(VisionIO io, RobotState state, DoubleSupplier clock) {
        this.io = io;
        this.state = state;
        this.clock = clock;
    }

    @Override
    public void periodic() {
        double startTime = clock.getAsDouble();
        var latestPose = state.getLatestFieldToRobot();
        double yawDegrees = latestPose == null ? 0.0 : latestPose.getValue().getRotation().getDegrees();
        double yawRate = state.getLatestDriveYawAngularVelocity();
        // 藍方原點、逆時針為正；紅方駕駛反轉不可套用到此資料。
        io.setRobotOrientation(yawDegrees, Units.radiansToDegrees(yawRate));
        io.readInputs(inputs);
        var cam = inputs.camera;
        logCameraInputs(cam, startTime);

        lastRejectionReason = useVision ? evaluate(cam, startTime, yawRate) : RejectionReason.DISABLED;
        if (lastRejectionReason == RejectionReason.ACCEPTED) {
            var mt1 = cam.megatagPoseEstimate;
            var mt2 = cam.megatag2PoseEstimate;
            // 參照 VisionIOLimelightHelper：MT2 的 X/Y，加上 MT1 的朝向。
            // 組成一筆量測，不把同一張影像當成兩個獨立感測器重複融合。
            Pose2d pose = new Pose2d(mt2.fieldToRobot().getTranslation(), mt1.fieldToRobot().getRotation());
            double distanceSquared = Math.pow(cam.megatag2AverageTagDistanceMeters, 2);
            double xyStd = cam.megatag2Count >= 2
                    ? 0.20 + 0.05 * distanceSquared : 0.50 + 0.12 * distanceSquared;
            state.updateMegatagEstimate(new VisionFieldPoseEstimate(
                    pose, mt2.timestampSeconds(),
                    VecBuilder.fill(xyStd, xyStd, VisionConstants.kHeadingStandardDeviationRadians),
                    cam.megatag2Count));
            lastAcceptedMt1Timestamp = mt1.timestampSeconds();
            Logger.recordOutput("Vision/accepted", pose);
            Logger.recordOutput("Vision/Camera/AcceptedMegatagEstimate", pose);
            Logger.recordOutput("Vision/XYStandardDeviation", xyStd);
        }

        Logger.recordOutput("Vision/RejectionReason", lastRejectionReason.toString());
        Logger.recordOutput("Vision/AcceptedThisCycle", lastRejectionReason == RejectionReason.ACCEPTED);
        Logger.recordOutput("Vision/usingVision", useVision);
        Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));
        Logger.recordOutput("Vision/RobotYawDegrees", yawDegrees);
        Logger.recordOutput("Vision/RobotYawRateDegreesPerSecond", Units.radiansToDegrees(yawRate));
        Logger.recordOutput("Vision/latencyPeriodicSec", clock.getAsDouble() - startTime);
    }

    private RejectionReason evaluate(VisionIO.VisionIOInputs.CameraInputs cam, double now, double yawRate) {
        var mt1 = cam.megatagPoseEstimate;
        var mt2 = cam.megatag2PoseEstimate;
        if (mt1 == null || cam.megatagCount < 1) return RejectionReason.MISSING_MT1;
        if (mt2 == null || cam.megatag2Count < 1) return RejectionReason.MISSING_MT2;
        if (!validTimestamp(mt1) || !validTimestamp(mt2) || !Double.isFinite(now)) {
            return RejectionReason.INVALID_TIMESTAMP;
        }
        if (!fresh(mt1.timestampSeconds(), now) || !fresh(mt2.timestampSeconds(), now)) {
            return RejectionReason.STALE;
        }
        if (Math.abs(mt1.timestampSeconds() - mt2.timestampSeconds())
                > VisionConstants.kMaxMegatagTimestampSkewSeconds) {
            return RejectionReason.UNSYNCHRONIZED;
        }
        if (mt1.timestampSeconds() <= lastAcceptedMt1Timestamp
                || mt2.timestampSeconds() <= state.lastUsedMegatagTimestamp()) {
            return RejectionReason.DUPLICATE_OR_OLDER;
        }
        if (!finitePose(mt1.fieldToRobot()) || !finitePose(mt2.fieldToRobot())) {
            return RejectionReason.INVALID_POSE;
        }
        double margin = VisionConstants.kFieldBoundaryMarginMeters;
        var pose = mt2.fieldToRobot();
        if (pose.getX() < -margin || pose.getX() > Constants.kFieldLengthMeters + margin
                || pose.getY() < -margin || pose.getY() > Constants.kFieldWidthMeters + margin) {
            return RejectionReason.OUTSIDE_FIELD;
        }
        double distance = cam.megatag2AverageTagDistanceMeters;
        if (!Double.isFinite(distance) || distance <= 0.0 || distance > VisionConstants.kMaxTagDistanceMeters) {
            return RejectionReason.TAG_TOO_FAR;
        }
        // 同時檢查現在與拍攝前後的角速度；舊 helper 的 signed max 必須取絕對值。
        double peakYawRate = state.getMaxAbsDriveYawAngularVelocityInRange(
                Math.min(mt1.timestampSeconds(), mt2.timestampSeconds()) - 0.3,
                Math.max(mt1.timestampSeconds(), mt2.timestampSeconds())).orElse(yawRate);
        if (!Double.isFinite(yawRate) || !Double.isFinite(peakYawRate)
                || Math.max(Math.abs(yawRate), Math.abs(peakYawRate))
                        > VisionConstants.kMaxVisionYawRateRadiansPerSecond) {
            return RejectionReason.ROTATING_TOO_FAST;
        }
        var exclusiveTag = state.getExclusiveTag();
        if (exclusiveTag.isPresent()
                && (!contains(mt1, exclusiveTag.get()) || !contains(mt2, exclusiveTag.get()))) {
            return RejectionReason.EXCLUSIVE_TAG_MISMATCH;
        }
        return RejectionReason.ACCEPTED;
    }

    private static boolean contains(MegatagPoseEstimate estimate, int tag) {
        return Arrays.stream(estimate.fiducialIds()).anyMatch(id -> id == tag);
    }

    private static boolean validTimestamp(MegatagPoseEstimate estimate) {
        return Double.isFinite(estimate.timestampSeconds()) && estimate.timestampSeconds() > 0.0
                && Double.isFinite(estimate.latency()) && estimate.latency() >= 0.0;
    }

    private static boolean fresh(double timestamp, double now) {
        double age = now - timestamp;
        return age >= -VisionConstants.kFutureTimestampToleranceSeconds
                && age <= VisionConstants.kMaxMeasurementAgeSeconds;
    }

    private static boolean finitePose(Pose2d pose) {
        return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    /** 局部追蹤獨立於場地定位是否被接受，□／△ 仍由相對座標控制 21 號。 */
    public Optional<AprilTagObservation> getAprilTagObservation() {
        return inputs.camera.seesTarget ? inputs.camera.aprilTagObservation : Optional.empty();
    }

    private void logCameraInputs(VisionIO.VisionIOInputs.CameraInputs cam, double now) {
        String prefix = "Vision/Camera";
        Logger.recordOutput(prefix + "/Heartbeat", cam.heartbeat);
        Logger.recordOutput(prefix + "/SeesTarget", cam.seesTarget);
        Logger.recordOutput(prefix + "/TrackingTagId", cam.aprilTagObservation.map(AprilTagObservation::tagId).orElse(-1));
        cam.aprilTagObservation.ifPresent(target -> {
            Logger.recordOutput(prefix + "/CameraToTag", target.cameraToTag());
            Logger.recordOutput(prefix + "/TagCaptureTimestamp", target.timestampSeconds());
        });
        Logger.recordOutput(prefix + "/FiducialCount", cam.fiducialObservations == null ? 0 : cam.fiducialObservations.length);
        if (cam.pose3d != null) Logger.recordOutput(prefix + "/Pose3d", cam.pose3d);
        if (cam.megatagPoseEstimate != null) {
            Logger.recordOutput(prefix + "/MegatagPoseEstimate", cam.megatagPoseEstimate.fieldToRobot());
        }
        Logger.recordOutput(prefix + "/MegatagCount", cam.megatagCount);
        Logger.recordOutput(prefix + "/Megatag2Count", cam.megatag2Count);
        Logger.recordOutput(prefix + "/MT2AverageDistanceMeters", cam.megatag2AverageTagDistanceMeters);
        logEstimate(prefix + "/MT1", cam.megatagPoseEstimate, now);
        logEstimate(prefix + "/MT2", cam.megatag2PoseEstimate, now);
    }

    private static void logEstimate(String prefix, MegatagPoseEstimate estimate, double now) {
        Logger.recordOutput(prefix + "/HasEstimate", estimate != null);
        Logger.recordOutput(prefix + "/Pose", estimate == null ? new Pose2d[0] : new Pose2d[] {estimate.fieldToRobot()});
        Logger.recordOutput(prefix + "/TimestampSeconds", estimate == null ? Double.NaN : estimate.timestampSeconds());
        Logger.recordOutput(prefix + "/AgeSeconds", estimate == null ? Double.NaN : now - estimate.timestampSeconds());
    }

    public RejectionReason getLastRejectionReason() {
        return lastRejectionReason;
    }

    public void setUseVision(boolean useVision) {
        this.useVision = useVision;
    }
}
