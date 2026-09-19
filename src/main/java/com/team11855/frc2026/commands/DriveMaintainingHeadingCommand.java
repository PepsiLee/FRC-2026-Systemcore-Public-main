package com.team11855.frc2026.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team11855.frc2026.Constants;
import com.team11855.frc2026.Robot;
import com.team11855.frc2026.RobotState;
import com.team11855.frc2026.subsystems.drive.DriveSubsystem;
import com.team11855.lib.util.Util;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveMaintainingHeadingCommand extends Command {
    public DriveMaintainingHeadingCommand(
            DriveSubsystem drivetrain,
            RobotState robotState,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier turn,
            BooleanSupplier driverControlEnabled) {
        mDrivetrain = drivetrain;
        mRobotState = robotState;
        mDriverControlEnabled = driverControlEnabled;
        mThrottleSupplier = throttle;
        mStrafeSupplier = strafe;
        mTurnSupplier = turn;

        driveWithHeading.HeadingController.setPID(
                Constants.DriveConstants.kHeadingControllerP,
                Constants.DriveConstants.kHeadingControllerI,
                Constants.DriveConstants.kHeadingControllerD);

        addRequirements(drivetrain);
        setName("Swerve Drive Maintain Heading");

        if (Robot.isSimulation()) {
            driveNoHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
            driveWithHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        }
    }

    private final RobotState mRobotState;
    private final BooleanSupplier mDriverControlEnabled;
    protected DriveSubsystem mDrivetrain;
    private final DoubleSupplier mThrottleSupplier;
    private final DoubleSupplier mStrafeSupplier;
    private final DoubleSupplier mTurnSupplier;
    private Optional<Rotation2d> mHeadingSetpoint = Optional.empty();
    private Optional<Rotation2d> mPendingHeadingSetpoint = Optional.empty();
    private double mJoystickLastTouched = -1;

    private final SwerveRequest.FieldCentric driveNoHeading =
            new SwerveRequest.FieldCentric()
                    // Alliance conversion is already applied to the suppliers below.
                    .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                    .withDeadband(
                            Constants.DriveConstants.kDriveMaxSpeed
                                    * 0.05) // Add a 5% deadband in open loop
                    .withRotationalDeadband(
                            Constants.DriveConstants.kDriveMaxAngularRate
                                    * Constants.kSteerJoystickDeadband)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                    .withTargetDirectionPerspective(
                            SwerveRequest.TargetDirectionPerspectiveValue.BlueAlliance)
                    .withDeadband(Constants.DriveConstants.kDriveMaxSpeed * 0.05)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    @Override
    public void initialize() {
        mHeadingSetpoint = mPendingHeadingSetpoint;
        mPendingHeadingSetpoint = Optional.empty();
        mJoystickLastTouched = -1;
    }

    /** Seeds the next default-command start without waiting for the odometry callback. */
    public void resetHeading(Rotation2d heading) {
        mPendingHeadingSetpoint = Optional.of(heading);
    }

    @Override
    public void execute() {
        // Evaluate connection/mode before reading axes or issuing a heading-hold request.
        if (!mDriverControlEnabled.getAsBoolean()) {
            mDrivetrain.stop();
            initialize();
            return;
        }
        double throttle = mThrottleSupplier.getAsDouble() * Constants.DriveConstants.kDriveMaxSpeed;
        double strafe = mStrafeSupplier.getAsDouble() * Constants.DriveConstants.kDriveMaxSpeed;
        double turnFieldFrame = mTurnSupplier.getAsDouble();
        double throttleFieldFrame = mRobotState.isRedAlliance() ? -throttle : throttle;
        double strafeFieldFrame = mRobotState.isRedAlliance() ? -strafe : strafe;
        if (Math.abs(turnFieldFrame) > Constants.kSteerJoystickDeadband) {
            mJoystickLastTouched = Timer.getFPGATimestamp();
        }
        if (Math.abs(turnFieldFrame) > Constants.kSteerJoystickDeadband
                || (Util.epsilonEquals(mJoystickLastTouched, Timer.getFPGATimestamp(), 0.25)
                        && Math.abs(
                                        mRobotState.getLatestRobotRelativeChassisSpeed()
                                                .omegaRadiansPerSecond)
                                > Math.toRadians(10))) {
            mDrivetrain.setControl(
                    (driveNoHeading
                            .withVelocityX(throttleFieldFrame)
                            .withVelocityY(strafeFieldFrame)
                            .withRotationalRate(
                                    turnFieldFrame
                                            * Constants.DriveConstants.kDriveMaxAngularRate)));
            mHeadingSetpoint = Optional.empty();
            Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
        } else {
            if (mHeadingSetpoint.isEmpty()) {
                mHeadingSetpoint =
                        Optional.of(mRobotState.getLatestFieldToRobot().getValue().getRotation());
            }
            Logger.recordOutput("DriveMaintainHeading/throttleFieldFrame", throttleFieldFrame);
            Logger.recordOutput("DriveMaintainHeading/strafeFieldFrame", strafeFieldFrame);
            Logger.recordOutput("DriveMaintainHeading/mHeadingSetpoint", mHeadingSetpoint.get());
            mDrivetrain.setControl(
                    driveWithHeading
                            .withVelocityX(throttleFieldFrame)
                            .withVelocityY(strafeFieldFrame)
                            .withTargetDirection(mHeadingSetpoint.get()));
            Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
            Logger.recordOutput(
                    "DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
        }
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.stop();
        initialize();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
