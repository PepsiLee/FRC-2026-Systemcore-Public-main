package com.team11855.frc2026.subsystems.drive;

import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.team11855.frc2026.Constants;
import com.team11855.frc2026.RobotState;
import com.team11855.frc2026.subsystems.vision.VisionFieldPoseEstimate;
import com.team11855.frc2026.utils.simulations.MapleSimSwerveDrivetrain;
import com.team11855.lib.pathplanner.auto.AutoBuilder;
import com.team11855.lib.pathplanner.config.ModuleConfig;
import com.team11855.lib.pathplanner.config.PIDConstants;
import com.team11855.lib.pathplanner.config.RobotConfig;
import com.team11855.lib.pathplanner.controllers.PPHolonomicDriveController;
import com.team11855.lib.pathplanner.controllers.PathFollowingController;
import com.team11855.lib.pathplanner.trajectory.PathPlannerTrajectory;
import com.team11855.lib.pathplanner.trajectory.PathPlannerTrajectoryState;
import com.team11855.lib.pathplanner.util.PathPlannerLogging;
import com.team11855.lib.time.RobotTime;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code DriveSubsystem} controls the robot's swerve drive base. It handles all driving
 * operations, including manual control, autonomous path following, and integration with sensors and
 * vision systems.
 */
public class DriveSubsystem extends SubsystemBase {
    DriveIO io;

    DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    DriveViz telemetry = new DriveViz(Constants.DriveConstants.kDriveMaxSpeed);

    RobotState robotState;

    private Controller controller;

    // Serialize the 100 Hz trajectory loop with command and mode changes.
    private final Object controlLock = new Object();

    private final ApplyRobotSpeeds stopRequest =
            new ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final ApplyRobotSpeeds pathplannerAutoRequest =
            new ApplyRobotSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withDesaturateWheelSpeeds(true);

    public DriveSubsystem(DriveIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;

        configurePathPlanner();
    }

    /** Creates a manually stepped controller without hardware configuration or a Notifier. */
    DriveSubsystem(
            DriveIO io, RobotState robotState, PathFollowingController pathFollowingController) {
        this.io = io;
        this.robotState = robotState;
        controller = new Controller(pathFollowingController, false);
    }

    private ChassisSpeeds applyDeadband(ChassisSpeeds input) {
        if (Math.hypot(input.vxMetersPerSecond, input.vyMetersPerSecond) < 0.05) {
            input.vxMetersPerSecond = input.vyMetersPerSecond = 0.0;
        }
        if (Math.abs(input.omegaRadiansPerSecond) < 0.05) {
            input.omegaRadiansPerSecond = 0.0;
        }
        return input;
    }

    private class Controller implements Consumer<PathPlannerTrajectory>, Runnable {
        private final PathFollowingController controller;
        private PathPlannerTrajectory trajectory = null;
        private final Timer timer = new Timer();
        private final Notifier notifier;

        boolean hasSetPriority = false;

        public Controller(PathFollowingController controller, boolean startNotifier) {
            this.controller = controller;
            if (startNotifier) {
                notifier = new Notifier(this);
                notifier.startPeriodic(0.01);
            } else {
                notifier = null;
            }
        }

        // Call only while holding controlLock.
        private void clearTrajectory() {
            trajectory = null;
            timer.stop();
            timer.reset();
        }

        @Override
        public void accept(PathPlannerTrajectory t) {
            synchronized (controlLock) {
                clearTrajectory();
                // A normal path ending at nonzero speed hands off without a zero-speed pulse.
                if (t == null) return;
                if (t.isStayStoppedTrajectory()
                        || t.getStates().isEmpty()
                        || !Double.isFinite(t.getTotalTimeSeconds())) {
                    io.setControl(stopRequest);
                    return;
                }
                controller.reset(
                        robotState.getLatestFieldToRobot().getValue(),
                        robotState.getLatestRobotRelativeChassisSpeed());
                timer.start();
                trajectory = t;
            }
        }

        @Override
        public void run() {
            if (notifier != null && !hasSetPriority) {
                hasSetPriority = Threads.setCurrentThreadPriority(true, 41);
            }

            synchronized (controlLock) {
                if (trajectory == null) return;

                double now = timer.get();
                PathPlannerTrajectoryState targetState = trajectory.sample(now);

                ChassisSpeeds speeds =
                        controller.calculateRobotRelativeSpeeds(
                                robotState.getLatestFieldToRobot().getValue(), targetState);
                if (DriverStation.isEnabled()) {
                    // The trajectory already owns control; do not use the manual takeover API.
                    io.setControl(
                            pathplannerAutoRequest
                                    .withSpeeds(applyDeadband(speeds))
                                    .withWheelForceFeedforwardsX(
                                            targetState.feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(
                                            targetState.feedforwards.robotRelativeForcesYNewtons()));
                }
            }
        }
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.readInputs(inputs);
        telemetry.telemeterize(inputs);
        Logger.processInputs("DriveInputs", inputs);
        io.logModules(inputs);

        robotState.incrementIterationCount();
        if (DriverStation.isDisabled()) {
            configureStandardDevsForDisabled();
        } else {
            configureStandardDevsForEnabled();
        }
        Logger.recordOutput(
                "Drive/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
        Logger.recordOutput(
                "Drive/currentCommand",
                (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    }

    private void configurePathPlanner() {
        ModuleConfig moduleConfig =
                new ModuleConfig(
                        Constants.DriveConstants.kDrivetrain.getModuleConstants()[0].WheelRadius,
                        // 馬達模型使用 Tuner 的 12 V 速度；不以 PS5 的速度上限代替。
                        CompTunerConstants.kSpeedAt12Volts.in(
                                edu.wpi.first.units.Units.MetersPerSecond),
                        Constants.DriveConstants.kWheelCoefficientOfFriction,
                        // MK5i R2 行走馬達為 Kraken X60；沿用原本路徑模型的 FOC 曲線。
                        DCMotor.getKrakenX60Foc(Constants.DriveConstants.kDriveMotorCount),
                        Constants.DriveConstants.kDrivetrain
                                .getModuleConstants()[0]
                                .DriveMotorGearRatio,
                        Constants.DriveConstants.kDrivetrain.getModuleConstants()[0].SlipCurrent,
                        1);

        RobotConfig robotConfig =
                new RobotConfig(
                        Constants.kRobotMassKg,
                        Constants.kRobotMomentOfInertia,
                        moduleConfig,
                        Constants.kCOGHeightMeters,
                        Constants.DriveConstants.kDrivetrain.getModuleLocations());

        controller =
                new Controller(
                        new PPHolonomicDriveController(
                                new PIDConstants(Constants.AutoConstants.kPLTEController, 0.0, 0.0),
                                new PIDConstants(Constants.AutoConstants.kPCTEController, 0.0, 0.0),
                                new PIDConstants(
                                        Constants.AutoConstants.kPathRotationControllerP,
                                        0.0,
                                        0.0),
                                0.01),
                        true);

        AutoBuilder.configure(
                () -> robotState.getLatestFieldToRobot().getValue(),
                this::resetOdometry,
                () -> robotState.getLatestFusedRobotRelativeChassisSpeed(),
                controller,
                robotConfig,
                () -> robotState.isRedAlliance(),
                this);

        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> {
                    robotState.setTrajectoryTargetPose(pose);
                    Logger.recordOutput("PathPlanner/targetPose", pose);
                });

        PathPlannerLogging.setLogCurrentPoseCallback(
                (pose) -> {
                    robotState.setTrajectoryCurrentPose(pose);
                    Logger.recordOutput("PathPlanner/currentPose", pose);
                });

        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "PathPlanner/activePath", activePath.toArray(new Pose2d[0]));
                });

        PathPlannerLogging.setLogTargetChassisSpeedsCallback(
                (chassisSpeeds) -> {
                    Logger.recordOutput("PathPlanner/targetChassisSpeeds", chassisSpeeds);
                });
    }

    public void resetOdometry(Pose2d pose) {
        synchronized (controlLock) {
            stop();
            io.resetOdometry(pose);
        }
    }

    public Consumer<PathPlannerTrajectory> getController() {
        return controller;
    }

    /** Cancels trajectory output before applying a manually supplied drive request. */
    public void setControl(SwerveRequest request) {
        synchronized (controlLock) {
            controller.clearTrajectory();
            io.setControl(request);
        }
    }

    /** Stops immediately and prevents a pending trajectory tick from restoring motion. */
    public void stop() {
        synchronized (controlLock) {
            controller.clearTrajectory();
            io.setControl(stopRequest);
        }
    }

    // API
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return Commands.runEnd(() -> setControl(requestSupplier.get()), this::stop, this)
                .withName("Swerve drive request");
    }

    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        io.addVisionMeasurement(visionFieldPoseEstimate);
    }

    public void setStateStdDevs(double xStd, double yStd, double rotStd) {
        io.setStateStdDevs(xStd, yStd, rotStd);
    }

    public void configureStandardDevsForDisabled() {
        setStateStdDevs(
                Constants.DriveConstants.kDisabledDriveXStdDev,
                Constants.DriveConstants.kDisabledDriveYStdDev,
                Constants.DriveConstants.kDisabledDriveRotStdDev);
    }

    public void configureStandardDevsForEnabled() {
        setStateStdDevs(
                Constants.DriveConstants.kEnabledDriveXStdDev,
                Constants.DriveConstants.kEnabledDriveYStdDev,
                Constants.DriveConstants.kEnabledDriveRotStdDev);
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        if (io instanceof DriveIOSim) {
            return ((DriveIOSim) io).getMapleSimDrive();
        }
        return null;
    }
}
