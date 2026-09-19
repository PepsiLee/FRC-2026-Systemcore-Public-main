// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team11855.frc2026;

import com.team11855.frc2026.commands.AprilTagTrackingCommand;
import com.team11855.frc2026.commands.AprilTagTrackingCommand.Mode;
import com.team11855.frc2026.commands.DriveMaintainingHeadingCommand;
import com.team11855.frc2026.simulation.SimulatedDriveState;
import com.team11855.frc2026.subsystems.drive.DriveIOHardware;
import com.team11855.frc2026.subsystems.drive.DriveIOSim;
import com.team11855.frc2026.subsystems.drive.DriveSubsystem;
import com.team11855.frc2026.subsystems.vision.VisionFieldPoseEstimate;
import com.team11855.frc2026.subsystems.vision.VisionIOHardwareLimelight;
import com.team11855.frc2026.subsystems.vision.VisionIOSimPhoton;
import com.team11855.frc2026.subsystems.vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import java.util.function.BooleanSupplier;

/** Wires the drivetrain, vision and a single PS5 driver controller. */
public class RobotContainer {
    private final RobotState robotState = new RobotState(this::acceptVisionEstimate);
    private final SimulatedDriveState simulatedDriveState =
            RobotBase.isSimulation() ? new SimulatedDriveState() : null;
    private final DriveSubsystem driveSubsystem = buildDriveSystem();
    private final VisionSubsystem visionSubsystem = buildVisionSystem();
    private final CommandPS5Controller driverController =
            new CommandPS5Controller(Constants.kDriverControllerPort);

    private final DriveMaintainingHeadingCommand driveCommand =
            new DriveMaintainingHeadingCommand(
                    driveSubsystem,
                    robotState,
                    () -> translationInput(driverController.getLeftY()),
                    () -> translationInput(driverController.getLeftX()),
                    () -> rotationInput(driverController.getRightX()),
                    this::isDriverControlEnabled);

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(driveCommand);
        driverController
                .options()
                .and(this::isDriverControlEnabled)
                .onTrue(Commands.runOnce(this::resetHeading, driveSubsystem));
        bindAprilTagControls(
                driverController,
                this::isTagTrackingEnabled,
                new AprilTagTrackingCommand(
                        driveSubsystem,
                        visionSubsystem::getAprilTagObservation,
                        this::isTagTrackingEnabled,
                        Mode.AIM_ONCE),
                new AprilTagTrackingCommand(
                        driveSubsystem,
                        visionSubsystem::getAprilTagObservation,
                        this::isTagTrackingEnabled,
                        Mode.FOLLOW_WHILE_HELD));
        SmartDashboard.putString("Drive/CANBus", Constants.kDriveCanBusName);
    }

    /** Bind separately so controller priority can be verified without constructing real motors. */
    static void bindAprilTagControls(
            CommandPS5Controller controller, BooleanSupplier enabled, Command aim, Command follow) {
        controller
                .square()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    // □ 只在實際按下時開始；△ 優先，忽略跟隨期間的 □。
                                    // 分派命令不搶 Drive，真正的 aim/follow 才持有 requirement。
                                    if (enabled.getAsBoolean()
                                            && !controller.getHID().getTriangleButton()) {
                                        CommandScheduler.getInstance().schedule(aim);
                                    }
                                }));
        controller.triangle().and(enabled).whileTrue(follow);
    }

    private boolean isTagTrackingEnabled() {
        return isDriverControlEnabled() && !driverController.getHID().getOptionsButton();
    }

    private DriveSubsystem buildDriveSystem() {
        if (RobotBase.isSimulation()) {
            return new DriveSubsystem(
                    new DriveIOSim(
                            robotState,
                            simulatedDriveState,
                            Constants.DriveConstants.kDrivetrain.getDriveTrainConstants(),
                            Constants.DriveConstants.kDrivetrain.getModuleConstants()),
                    robotState);
        }
        return new DriveSubsystem(
                new DriveIOHardware(
                        robotState,
                        Constants.DriveConstants.kDrivetrain.getDriveTrainConstants(),
                        Constants.DriveConstants.kDrivetrain.getModuleConstants()),
                robotState);
    }

    private VisionSubsystem buildVisionSystem() {
        if (RobotBase.isSimulation()) {
            return new VisionSubsystem(
                    new VisionIOSimPhoton(robotState, simulatedDriveState), robotState);
        }
        return new VisionSubsystem(new VisionIOHardwareLimelight(robotState), robotState);
    }

    private void acceptVisionEstimate(VisionFieldPoseEstimate estimate) {
        driveSubsystem.addVisionMeasurement(estimate);
    }

    private boolean isDriverControlEnabled() {
        return DriverStation.isTeleopEnabled() && driverController.getHID().isConnected();
    }

    // Preserve the original driver's response curves; command requests apply the original deadbands.
    static double translationInput(double axis) {
        return -Math.copySign(Math.pow(Math.abs(axis), 1.5), axis);
    }

    static double rotationInput(double axis) {
        return -Math.copySign(axis * axis, axis);
    }

    /** Keeps field X/Y and resets heading to the driver's alliance direction. */
    public void resetHeading() {
        Rotation2d heading =
                robotState.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;
        driveSubsystem.resetOdometry(
                new Pose2d(robotState.getLatestFieldToRobot().getValue().getTranslation(), heading));
        driveCommand.resetHeading(heading);
    }

    /** Replace this with a command from the local AutoBuilder to enable a chosen drive-only auto. */
    public Command getAutonomousCommand() {
        return Commands.none();
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public RobotState getRobotState() {
        return robotState;
    }
}
