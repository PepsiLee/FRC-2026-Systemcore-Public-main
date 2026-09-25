package com.team11855.frc2026.subsystems.vision;

import com.team11855.frc2026.Constants;
import com.team11855.frc2026.Constants.VisionConstants;
import com.team11855.frc2026.RobotState;
import com.team11855.lib.time.RobotTime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Team 254's MT1-first / single-tag gyro fallback, adapted to our single camera.
 *
 * <p>Reference: Team254/FRC-2025-Public ae1aa582. Camera mounting, local tag tracking, input
 * validation, and FPGA timestamps remain specific to this robot.
 */
public class VisionSubsystem extends SubsystemBase {
    public enum RejectionReason {
        ACCEPTED,
        DISABLED,
        MISSING_MT1,
        INVALID_TIMESTAMP,
        STALE,
        DUPLICATE_OR_OLDER,
        INVALID_POSE,
        INVALID_FIDUCIALS,
        INVALID_STD_DEVS,
        INVALID_QUALITY,
        AMBIGUOUS_TAG,
        TAG_TOO_SMALL,
        YAW_MISMATCH,
        NEAR_FIELD_ORIGIN,
        INVALID_HEIGHT,
        EXCLUSIVE_TAG_MISMATCH,
        MISSING_HISTORY,
        NOT_SINGLE_TAG,
        ROTATING_TOO_FAST,
        UNKNOWN_TAG
    }

    public enum EstimateSource {
        NONE,
        MEGATAG,
        GYRO
    }

    private final VisionIO io;
    private final RobotState state;
    private final DoubleSupplier clock;
    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    private boolean useVision = true;
    private RejectionReason lastRejectionReason = RejectionReason.MISSING_MT1;
    private RejectionReason megatagRejectionReason = RejectionReason.MISSING_MT1;
    private RejectionReason gyroRejectionReason = RejectionReason.MISSING_MT1;
    private EstimateSource lastEstimateSource = EstimateSource.NONE;

    public VisionSubsystem(VisionIO io, RobotState state) {
        this(io, state, RobotTime::getTimestampSeconds);
    }

    VisionSubsystem(VisionIO io, RobotState state, DoubleSupplier clock) {
        this.io = io;
        this.state = state;
        this.clock = clock;
    }

    @Override
    public void periodic() {
        double startTime = clock.getAsDouble();
        io.readInputs(inputs);
        logCameraInputs(inputs.camera, startTime);

        // Like 254, process and log candidates even when fusion is disabled.
        var accepted = processCamera(inputs.camera, startTime);
        if (!useVision) {
            lastRejectionReason = RejectionReason.DISABLED;
            lastEstimateSource = EstimateSource.NONE;
        } else {
            accepted.ifPresent(
                    estimate -> {
                        Logger.recordOutput("Vision/accepted", estimate.getVisionRobotPoseMeters());
                        state.updateMegatagEstimate(estimate);
                    });
        }

        Logger.recordOutput("Vision/RejectionReason", lastRejectionReason.toString());
        Logger.recordOutput("Vision/EstimateSource", lastEstimateSource.toString());
        Logger.recordOutput(
                "Vision/Camera/MegatagRejectionReason", megatagRejectionReason.toString());
        Logger.recordOutput("Vision/Camera/GyroRejectionReason", gyroRejectionReason.toString());
        Logger.recordOutput(
                "Vision/Camera/AcceptMegatag", lastEstimateSource == EstimateSource.MEGATAG);
        Logger.recordOutput("Vision/Camera/AcceptGyro", lastEstimateSource == EstimateSource.GYRO);
        Logger.recordOutput("Vision/AcceptedThisCycle", useVision && accepted.isPresent());
        Logger.recordOutput("Vision/usingVision", useVision);
        Logger.recordOutput("Vision/exclusiveTagId", state.getExclusiveTag().orElse(-1));
        Logger.recordOutput("Vision/latencyPeriodicSec", clock.getAsDouble() - startTime);
    }

    private Optional<VisionFieldPoseEstimate> processCamera(
            VisionIO.VisionIOInputs.CameraInputs cam, double now) {
        lastEstimateSource = EstimateSource.NONE;
        lastRejectionReason = validateInputs(cam, now);
        megatagRejectionReason = lastRejectionReason;
        gyroRejectionReason = lastRejectionReason;
        Optional<VisionFieldPoseEstimate> mtEstimate = Optional.empty();
        Optional<VisionFieldPoseEstimate> gyroEstimate = Optional.empty();

        if (lastRejectionReason == RejectionReason.ACCEPTED) {
            mtEstimate = processMegatagPoseEstimate(cam.megatagPoseEstimate, cam);
            gyroEstimate = fuseWithGyro(cam.megatagPoseEstimate, cam);
        }
        logCandidate("Vision/Camera/AcceptedMegatagEstimate", mtEstimate);
        logCandidate("Vision/Camera/FuseWithGyroEstimate", gyroEstimate);

        // 254: prefer MT1, even for a single tag; gyro is a fallback, not a second measurement.
        if (mtEstimate.isPresent()) {
            lastEstimateSource = EstimateSource.MEGATAG;
            return mtEstimate;
        }
        if (gyroEstimate.isPresent()) {
            lastEstimateSource = EstimateSource.GYRO;
            return gyroEstimate;
        }
        lastRejectionReason = megatagRejectionReason;
        return Optional.empty();
    }

    /** Transport/data guards shared by both branches, before applying 254's selection policy. */
    private RejectionReason validateInputs(VisionIO.VisionIOInputs.CameraInputs cam, double now) {
        var estimate = cam.megatagPoseEstimate;
        // tv/priorityid belongs to local tracking; other field tags may still provide an MT1 pose.
        if (estimate == null || cam.megatagCount < 1) return RejectionReason.MISSING_MT1;
        if (!Double.isFinite(now)
                || !Double.isFinite(estimate.timestampSeconds())
                || estimate.timestampSeconds() <= 0.0
                || !Double.isFinite(estimate.latency())
                || estimate.latency() < 0.0) {
            return RejectionReason.INVALID_TIMESTAMP;
        }
        double age = now - estimate.timestampSeconds();
        if (age < -VisionConstants.kFutureTimestampToleranceSeconds
                || age > VisionConstants.kMaxMeasurementAgeSeconds) {
            return RejectionReason.STALE;
        }
        if (estimate.timestampSeconds() <= state.lastUsedMegatagTimestamp()) {
            return RejectionReason.DUPLICATE_OR_OLDER;
        }
        if (!finitePose(estimate.fieldToRobot())
                || cam.pose3d == null
                || !Double.isFinite(cam.pose3d.getZ())) {
            return RejectionReason.INVALID_POSE;
        }
        if (estimate.fiducialIds().length != cam.megatagCount
                || Arrays.stream(estimate.fiducialIds()).anyMatch(id -> id <= 0)
                || cam.fiducialObservations == null
                || cam.fiducialObservations.length != cam.megatagCount
                || Arrays.stream(cam.fiducialObservations)
                        .anyMatch(tag -> tag == null || !Double.isFinite(tag.ambiguity()))) {
            return RejectionReason.INVALID_FIDUCIALS;
        }
        if (cam.standardDeviations == null
                || cam.standardDeviations.length != VisionConstants.kExpectedStdDevArrayLength
                || !positiveFinite(cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex])
                || !positiveFinite(cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex])) {
            return RejectionReason.INVALID_STD_DEVS;
        }
        return RejectionReason.ACCEPTED;
    }

    private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(
            MegatagPoseEstimate poseEstimate, VisionIO.VisionIOInputs.CameraInputs cam) {
        megatagRejectionReason = evaluateMegatag(poseEstimate, cam);
        if (megatagRejectionReason != RejectionReason.ACCEPTED) return Optional.empty();

        // Same quality scaling and conservative XY standard deviation as 254.
        double scaleFactor = 1.0 / poseEstimate.quality();
        double xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex] * scaleFactor;
        double yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex] * scaleFactor;
        double rotStd =
                cam.standardDeviations[VisionConstants.kMegatag1YawStdDevIndex] * scaleFactor;
        double xyStd = Math.max(xStd, yStd);
        if (!positiveFinite(xyStd) || !positiveFinite(rotStd)) {
            megatagRejectionReason = RejectionReason.INVALID_STD_DEVS;
            return Optional.empty();
        }
        return Optional.of(
                new VisionFieldPoseEstimate(
                        poseEstimate.fieldToRobot(),
                        poseEstimate.timestampSeconds(),
                        VecBuilder.fill(xyStd, xyStd, rotStd),
                        poseEstimate.fiducialIds().length));
    }

    private RejectionReason evaluateMegatag(
            MegatagPoseEstimate poseEstimate, VisionIO.VisionIOInputs.CameraInputs cam) {
        if (poseEstimate.fiducialIds().length < 2) {
            for (var fiducial : cam.fiducialObservations) {
                if (fiducial.ambiguity() > VisionConstants.kDefaultAmbiguityThreshold) {
                    return RejectionReason.AMBIGUOUS_TAG;
                }
            }
            if (poseEstimate.avgTagArea() < VisionConstants.kTagMinAreaForSingleTagMegatag) {
                return RejectionReason.TAG_TOO_SMALL;
            }
            var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
            if (poseEstimate.avgTagArea() < VisionConstants.kTagAreaThresholdForYawCheck
                    && priorPose.isPresent()) {
                double yawDiff =
                        Math.abs(
                                MathUtil.angleModulus(
                                        priorPose.get().getRotation().getRadians()
                                                - poseEstimate
                                                        .fieldToRobot()
                                                        .getRotation()
                                                        .getRadians()));
                if (yawDiff > Units.degreesToRadians(VisionConstants.kDefaultYawDiffThreshold)) {
                    return RejectionReason.YAW_MISMATCH;
                }
            }
        }
        if (poseEstimate.fieldToRobot().getTranslation().getNorm()
                < VisionConstants.kDefaultNormThreshold) {
            return RejectionReason.NEAR_FIELD_ORIGIN;
        }
        if (Math.abs(cam.pose3d.getZ()) > VisionConstants.kDefaultZThreshold) {
            return RejectionReason.INVALID_HEIGHT;
        }
        var exclusiveTag = state.getExclusiveTag();
        if (exclusiveTag.isPresent()
                && Arrays.stream(poseEstimate.fiducialIds())
                        .noneMatch(id -> id == exclusiveTag.get())) {
            return RejectionReason.EXCLUSIVE_TAG_MISMATCH;
        }
        if (state.getFieldToRobot(poseEstimate.timestampSeconds()).isEmpty()) {
            return RejectionReason.MISSING_HISTORY;
        }
        if (!positiveFinite(poseEstimate.quality())
                || poseEstimate.quality() > 1.0
                || !Double.isFinite(poseEstimate.avgTagArea())) {
            return RejectionReason.INVALID_QUALITY;
        }
        return RejectionReason.ACCEPTED;
    }

    private Optional<VisionFieldPoseEstimate> fuseWithGyro(
            MegatagPoseEstimate poseEstimate, VisionIO.VisionIOInputs.CameraInputs cam) {
        if (poseEstimate.fiducialIds().length != 1) {
            gyroRejectionReason = RejectionReason.NOT_SINGLE_TAG;
            return Optional.empty();
        }
        double peakYawRate =
                state.getMaxAbsDriveYawAngularVelocityInRange(
                                poseEstimate.timestampSeconds()
                                        - VisionConstants.kHighYawLookbackSeconds,
                                poseEstimate.timestampSeconds())
                        .orElse(Double.POSITIVE_INFINITY);
        // 254's helper returns a signed extreme: take abs so both rotation directions are checked.
        if (!Double.isFinite(peakYawRate)
                || Math.abs(peakYawRate) > VisionConstants.kMaxVisionYawRateRadiansPerSecond) {
            gyroRejectionReason = RejectionReason.ROTATING_TOO_FAST;
            return Optional.empty();
        }
        var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
        if (priorPose.isEmpty() || !finitePose(priorPose.get())) {
            gyroRejectionReason = RejectionReason.MISSING_HISTORY;
            return Optional.empty();
        }
        var maybeFieldToTag =
                Constants.kAprilTagLayoutReefsOnly.getTagPose(poseEstimate.fiducialIds()[0]);
        if (maybeFieldToTag.isEmpty()) {
            gyroRejectionReason = RejectionReason.UNKNOWN_TAG;
            return Optional.empty();
        }

        // 254's fallback deliberately has its own checks; do not reapply MT1 quality/exclusiveTag
        // gates.
        Pose2d fieldToTag =
                new Pose2d(maybeFieldToTag.get().toPose2d().getTranslation(), Rotation2d.kZero);
        Pose2d robotToTag = fieldToTag.relativeTo(poseEstimate.fieldToRobot());
        Pose2d posteriorPose =
                new Pose2d(
                        fieldToTag
                                .getTranslation()
                                .minus(
                                        robotToTag
                                                .getTranslation()
                                                .rotateBy(priorPose.get().getRotation())),
                        priorPose.get().getRotation());
        if (!finitePose(posteriorPose)) {
            gyroRejectionReason = RejectionReason.INVALID_POSE;
            return Optional.empty();
        }
        double xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex];
        double yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex];
        double xyStd = Math.max(xStd, yStd);
        gyroRejectionReason = RejectionReason.ACCEPTED;
        return Optional.of(
                new VisionFieldPoseEstimate(
                        posteriorPose,
                        poseEstimate.timestampSeconds(),
                        VecBuilder.fill(xyStd, xyStd, VisionConstants.kLargeVariance),
                        poseEstimate.fiducialIds().length));
    }

    private static boolean positiveFinite(double value) {
        return Double.isFinite(value) && value > 0.0;
    }

    private static boolean finitePose(Pose2d pose) {
        return pose != null
                && Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    /** Local tracking of tag 21 remains independent of accepted field localization. */
    public Optional<AprilTagObservation> getAprilTagObservation() {
        return inputs.camera.seesTarget ? inputs.camera.aprilTagObservation : Optional.empty();
    }

    private void logCameraInputs(VisionIO.VisionIOInputs.CameraInputs cam, double now) {
        String prefix = "Vision/Camera";
        Logger.recordOutput(prefix + "/Heartbeat", cam.heartbeat);
        Logger.recordOutput(prefix + "/SeesTarget", cam.seesTarget);
        Logger.recordOutput(
                prefix + "/TrackingTagId",
                cam.aprilTagObservation.map(AprilTagObservation::tagId).orElse(-1));
        cam.aprilTagObservation.ifPresent(
                target -> {
                    Logger.recordOutput(prefix + "/CameraToTag", target.cameraToTag());
                    Logger.recordOutput(prefix + "/TagCaptureTimestamp", target.timestampSeconds());
                });
        Logger.recordOutput(
                prefix + "/FiducialCount",
                cam.fiducialObservations == null ? 0 : cam.fiducialObservations.length);
        if (cam.pose3d != null) Logger.recordOutput(prefix + "/Pose3d", cam.pose3d);
        Logger.recordOutput(prefix + "/MegatagCount", cam.megatagCount);
        var mt1 = cam.megatagPoseEstimate;
        Logger.recordOutput(prefix + "/MT1/HasEstimate", mt1 != null);
        Logger.recordOutput(
                prefix + "/MT1/Pose",
                mt1 == null ? new Pose2d[0] : new Pose2d[] {mt1.fieldToRobot()});
        Logger.recordOutput(
                prefix + "/MT1/TimestampSeconds",
                mt1 == null ? Double.NaN : mt1.timestampSeconds());
        Logger.recordOutput(
                prefix + "/MT1/AgeSeconds",
                mt1 == null ? Double.NaN : now - mt1.timestampSeconds());
        Logger.recordOutput(prefix + "/Quality", mt1 == null ? Double.NaN : mt1.quality());
        Logger.recordOutput(prefix + "/AvgTagArea", mt1 == null ? Double.NaN : mt1.avgTagArea());
    }

    private static void logCandidate(String key, Optional<VisionFieldPoseEstimate> estimate) {
        Logger.recordOutput(
                key,
                estimate.map(value -> new Pose2d[] {value.getVisionRobotPoseMeters()})
                        .orElseGet(() -> new Pose2d[0]));
    }

    public RejectionReason getLastRejectionReason() {
        return lastRejectionReason;
    }

    public RejectionReason getMegatagRejectionReason() {
        return megatagRejectionReason;
    }

    public RejectionReason getGyroRejectionReason() {
        return gyroRejectionReason;
    }

    public EstimateSource getLastEstimateSource() {
        return lastEstimateSource;
    }

    public void setUseVision(boolean useVision) {
        this.useVision = useVision;
    }
}
