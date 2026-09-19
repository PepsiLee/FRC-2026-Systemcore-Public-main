package com.team11855.frc2026.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team11855.frc2026.Constants;
import com.team11855.frc2026.Constants.AprilTagTrackingConstants;
import com.team11855.frc2026.subsystems.drive.DriveSubsystem;
import com.team11855.frc2026.subsystems.vision.AprilTagObservation;
import com.team11855.lib.time.RobotTime;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Turns the robot's front toward the configured tag, optionally holding a center-to-tag horizontal
 * distance.
 */
public class AprilTagTrackingCommand extends Command {
    public enum Mode {
        AIM_ONCE,
        FOLLOW_WHILE_HELD
    }

    private final DriveSubsystem drive;
    private final Supplier<Optional<AprilTagObservation>> observationSupplier;
    private final BooleanSupplier controlEnabled;
    private final Mode mode;
    private final Transform3d robotToCamera;
    private final DoubleSupplier clock;
    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
    private final int targetTagId = AprilTagTrackingConstants.kTargetTagId;
    private double startTime;
    private double alignedSince = Double.NaN;
    private boolean finished;

    public AprilTagTrackingCommand(
            DriveSubsystem drive,
            Supplier<Optional<AprilTagObservation>> observationSupplier,
            BooleanSupplier controlEnabled,
            Mode mode) {
        this(
                drive,
                observationSupplier,
                controlEnabled,
                mode,
                Constants.VisionConstants.kRobotToCamera,
                RobotTime::getTimestampSeconds);
    }

    /** Allows geometry and time to be supplied without a physical camera during verification. */
    public AprilTagTrackingCommand(
            DriveSubsystem drive,
            Supplier<Optional<AprilTagObservation>> observationSupplier,
            BooleanSupplier controlEnabled,
            Mode mode,
            Transform3d robotToCamera,
            DoubleSupplier clock) {
        this.drive = drive;
        this.observationSupplier = observationSupplier;
        this.controlEnabled = controlEnabled;
        this.mode = mode;
        this.robotToCamera = robotToCamera;
        this.clock = clock;
        request.withDriveRequestType(
                RobotBase.isSimulation()
                        ? DriveRequestType.OpenLoopVoltage
                        : DriveRequestType.Velocity);
        addRequirements(drive);
        setName(mode == Mode.AIM_ONCE ? "AprilTag Aim Once" : "AprilTag Follow 1m");
    }

    @Override
    public void initialize() {
        startTime = clock.getAsDouble();
        alignedSince = Double.NaN;
        finished = false;
        drive.stop();
        Logger.recordOutput("AprilTagTracking/HasUsableTarget", false);
        Logger.recordOutput("AprilTagTracking/Active", true);
        Logger.recordOutput("AprilTagTracking/ObservedTagId", -1);
        Logger.recordOutput("AprilTagTracking/Mode", mode.toString());
        Logger.recordOutput("AprilTagTracking/LockedTagId", targetTagId);
    }

    @Override
    public void execute() {
        if (!controlEnabled.getAsBoolean()) {
            finished = true;
            Logger.recordOutput("AprilTagTracking/HasUsableTarget", false);
            stop("ControlsUnavailable");
            return;
        }
        double now = clock.getAsDouble();
        if (!Double.isFinite(now) || !Double.isFinite(startTime)) {
            finished = true;
            stop("InvalidClock");
            return;
        }
        if (mode == Mode.AIM_ONCE
                && now - startTime >= AprilTagTrackingConstants.kAimTimeoutSeconds) {
            finished = true;
            stop("AimTimeout");
            return;
        }

        var observation = observationSupplier.get();
        if (observation.isEmpty()) {
            Logger.recordOutput("AprilTagTracking/ObservedTagId", -1);
            targetUnavailable("NoTarget");
            return;
        }
        var target = observation.get();
        Logger.recordOutput("AprilTagTracking/ObservedTagId", target.tagId());
        double age = now - target.timestampSeconds();
        if (target.tagId() <= 0
                || !Double.isFinite(age)
                || target.timestampSeconds() <= 0.0
                || age < -AprilTagTrackingConstants.kFutureTimestampToleranceSeconds
                || age > AprilTagTrackingConstants.kMaxObservationAgeSeconds) {
            targetUnavailable("StaleOrInvalidTarget");
            return;
        }
        var robotToTag = target.robotToTag(robotToCamera);
        double distance = Math.hypot(robotToTag.getX(), robotToTag.getY());
        if (!Double.isFinite(distance)
                || !Double.isFinite(robotToTag.getZ())
                || distance < AprilTagTrackingConstants.kMinTargetDistanceMeters
                || distance > AprilTagTrackingConstants.kMaxTargetDistanceMeters) {
            targetUnavailable("InvalidDistance");
            return;
        }
        // 即使相機回傳其他 ID，也不可改追其他標籤。
        if (target.tagId() != targetTagId) {
            targetUnavailable("DifferentTag");
            return;
        }

        // 以底盤中心為基準：正角表示標籤在車頭左側，CTRE 正角速度也往左轉。
        double headingError = Math.atan2(robotToTag.getY(), robotToTag.getX());
        double distanceError = distance - AprilTagTrackingConstants.kTargetDistanceMeters;
        boolean aligned =
                Math.abs(headingError) <= AprilTagTrackingConstants.kHeadingToleranceRadians;
        double maxOmega =
                Math.min(
                        AprilTagTrackingConstants.kMaxAngularSpeedRadiansPerSecond,
                        Constants.DriveConstants.kDriveMaxAngularRate);
        double omega =
                aligned
                        ? 0.0
                        : MathUtil.clamp(
                                headingError * AprilTagTrackingConstants.kHeadingP,
                                -maxOmega,
                                maxOmega);
        double velocity = 0.0;
        if (mode == Mode.FOLLOW_WHILE_HELD
                && Math.abs(headingError)
                        <= AprilTagTrackingConstants.kMaxHeadingErrorForTranslationRadians
                && Math.abs(distanceError)
                        > AprilTagTrackingConstants.kDistanceToleranceMeters) {
            double maxSpeed =
                    Math.min(
                            AprilTagTrackingConstants.kMaxLinearSpeedMetersPerSecond,
                            Constants.DriveConstants.kDriveMaxSpeed);
            // 太遠向前靠近，太近向後退；水平距離不包含鏡頭與標籤的高度差。
            velocity =
                    MathUtil.clamp(
                            distanceError * AprilTagTrackingConstants.kDistanceP,
                            -maxSpeed,
                            maxSpeed);
        }

        Logger.recordOutput("AprilTagTracking/LockedTagId", targetTagId);
        Logger.recordOutput("AprilTagTracking/DistanceMeters", distance);
        Logger.recordOutput("AprilTagTracking/DistanceErrorMeters", distanceError);
        Logger.recordOutput("AprilTagTracking/HeadingErrorDegrees", Math.toDegrees(headingError));
        Logger.recordOutput("AprilTagTracking/ObservationAgeSeconds", age);
        Logger.recordOutput("AprilTagTracking/HasUsableTarget", true);
        if (mode == Mode.AIM_ONCE) {
            if (!aligned) {
                alignedSince = Double.NaN;
            } else if (Double.isNaN(alignedSince)) {
                alignedSince = now;
            }
            if (aligned && now - alignedSince >= AprilTagTrackingConstants.kAimSettleSeconds) {
                finished = true;
                stop("Aligned");
                return;
            }
        }

        Logger.recordOutput("AprilTagTracking/Status", "Tracking");
        Logger.recordOutput("AprilTagTracking/VelocityX", velocity);
        Logger.recordOutput("AprilTagTracking/OmegaRadiansPerSecond", omega);
        drive.setControl(request.withVelocityX(velocity).withVelocityY(0.0).withRotationalRate(omega));
    }

    private void targetUnavailable(String reason) {
        alignedSince = Double.NaN;
        Logger.recordOutput("AprilTagTracking/HasUsableTarget", false);
        // □ 失去目標就結束；△ 持有底盤並停止，等待指定的 21 號標籤重新出現。
        if (mode == Mode.AIM_ONCE) finished = true;
        stop(reason);
    }

    private void stop(String reason) {
        drive.stop();
        Logger.recordOutput("AprilTagTracking/Status", reason);
        Logger.recordOutput("AprilTagTracking/VelocityX", 0.0);
        Logger.recordOutput("AprilTagTracking/OmegaRadiansPerSecond", 0.0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        stop(interrupted ? "Interrupted" : "Finished");
        Logger.recordOutput("AprilTagTracking/HasUsableTarget", false);
        Logger.recordOutput("AprilTagTracking/Active", false);
    }
}
