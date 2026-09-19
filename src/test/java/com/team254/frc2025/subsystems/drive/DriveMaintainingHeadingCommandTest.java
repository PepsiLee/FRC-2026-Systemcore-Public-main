package com.team254.frc2025.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.commands.DriveMaintainingHeadingCommand;
import com.team254.frc2025.subsystems.vision.VisionFieldPoseEstimate;
import com.team254.lib.pathplanner.controllers.PathFollowingController;
import com.team254.lib.pathplanner.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveMaintainingHeadingCommandTest {
    private final FakeIO io = new FakeIO();
    private final DriverState state = new DriverState();
    private DriveSubsystem drive;
    private DriveMaintainingHeadingCommand command;
    private boolean enabled = true;
    private double throttle = 0.5;
    private double strafe = 0.25;
    private double turn;
    private int axisReads;

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        drive = new DriveSubsystem(io, state, new UnusedPathController());
        command =
                new DriveMaintainingHeadingCommand(
                        drive,
                        state,
                        () -> {
                            axisReads++;
                            return throttle;
                        },
                        () -> strafe,
                        () -> turn,
                        () -> enabled);
        state.addOdometryMeasurement(1.0, new Pose2d(2, 3, Rotation2d.fromDegrees(35)));
        command.initialize();
    }

    @AfterEach
    void tearDown() {
        command.end(true);
        CommandScheduler.getInstance().unregisterSubsystem(drive);
    }

    @Test
    void disabledOrDisconnectedInputStopsWithoutReadingAxes() {
        command.execute();
        int previousReads = axisReads;
        enabled = false;
        command.execute();
        assertEquals(previousReads, axisReads);
        assertStopped();
        enabled = true;
        command.execute();
        assertInstanceOf(SwerveRequest.FieldCentricFacingAngle.class, io.lastRequest);
    }

    @Test
    void holdKeepsInitialHeadingAndMirrorsRedTranslation() {
        command.execute();
        var hold = assertInstanceOf(SwerveRequest.FieldCentricFacingAngle.class, io.lastRequest);
        assertEquals(35, hold.TargetDirection.getDegrees(), 1e-9);
        assertEquals(0.5 * Constants.DriveConstants.kDriveMaxSpeed, hold.VelocityX, 1e-9);
        state.red = true;
        state.addOdometryMeasurement(1.1, new Pose2d(2, 3, Rotation2d.fromDegrees(40)));
        command.execute();
        assertEquals(35, hold.TargetDirection.getDegrees(), 1e-9);
        assertEquals(-0.5 * Constants.DriveConstants.kDriveMaxSpeed, hold.VelocityX, 1e-9);
        assertEquals(-0.25 * Constants.DriveConstants.kDriveMaxSpeed, hold.VelocityY, 1e-9);
    }

    @Test
    void manualTurnReleasesHoldAndReturningToCenterCapturesNewHeading() {
        turn = 0.5;
        command.execute();
        var manual = assertInstanceOf(SwerveRequest.FieldCentric.class, io.lastRequest);
        assertEquals(0.5 * Constants.DriveConstants.kDriveMaxAngularRate, manual.RotationalRate);
        turn = 0.0;
        state.addOdometryMeasurement(1.1, new Pose2d(2, 3, Rotation2d.fromDegrees(80)));
        command.execute();
        var hold = assertInstanceOf(SwerveRequest.FieldCentricFacingAngle.class, io.lastRequest);
        assertEquals(80, hold.TargetDirection.getDegrees(), 1e-9);
    }

    @Test
    void fullStickUsesFiveMetersPerSecondAndPointFourRotationsPerSecond() {
        throttle = 1.0;
        strafe = 0.0;
        turn = 1.0;
        command.execute();
        var request = assertInstanceOf(SwerveRequest.FieldCentric.class, io.lastRequest);
        assertEquals(5.0, request.VelocityX, 1e-9);
        // 每秒 0.4 圈就是每秒 144 度，送進 CTRE 的值必須是弧度/秒。
        assertEquals(Math.toRadians(144.0), request.RotationalRate, 1e-9);
    }

    @Test
    void optionsHeadingSurvivesDefaultRestartWithStaleOdometry() {
        command.end(true);
        command.resetHeading(Rotation2d.k180deg);
        command.initialize();
        // RobotState still reports the old 35-degree pose until a telemetry callback arrives.
        command.execute();
        var hold = assertInstanceOf(SwerveRequest.FieldCentricFacingAngle.class, io.lastRequest);
        assertEquals(180, hold.TargetDirection.getDegrees(), 1e-9);
        command.end(true);
        assertStopped();
    }

    private void assertStopped() {
        var stop = assertInstanceOf(SwerveRequest.ApplyRobotSpeeds.class, io.lastRequest);
        assertEquals(0.0, stop.Speeds.vxMetersPerSecond);
        assertEquals(0.0, stop.Speeds.vyMetersPerSecond);
        assertEquals(0.0, stop.Speeds.omegaRadiansPerSecond);
    }

    private static class DriverState extends RobotState {
        boolean red;

        DriverState() {
            super(estimate -> {});
        }

        @Override
        public boolean isRedAlliance() {
            return red;
        }
    }

    private static class UnusedPathController implements PathFollowingController {
        @Override
        public ChassisSpeeds calculateRobotRelativeSpeeds(
                Pose2d pose, PathPlannerTrajectoryState target) {
            throw new AssertionError("No trajectory controller should run in this test");
        }

        @Override
        public void reset(Pose2d pose, ChassisSpeeds speeds) {}

        @Override
        public boolean isHolonomic() {
            return true;
        }
    }

    private static class FakeIO implements DriveIO {
        SwerveRequest lastRequest;

        @Override
        public void setControl(SwerveRequest request) {
            lastRequest = request;
        }

        @Override
        public void readInputs(DriveIOInputs inputs) {}

        @Override
        public void logModules(SwerveDriveState driveState) {}

        @Override
        public void resetOdometry(Pose2d pose) {}

        @Override
        public Command applyRequest(Supplier<SwerveRequest> supplier, Subsystem requirement) {
            throw new AssertionError("Use DriveSubsystem control API");
        }

        @Override
        public void addVisionMeasurement(VisionFieldPoseEstimate estimate) {}

        @Override
        public void setStateStdDevs(double x, double y, double rotation) {}
    }
}
