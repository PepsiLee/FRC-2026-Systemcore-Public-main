package com.team11855.frc2026.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team11855.frc2026.RobotState;
import com.team11855.frc2026.commands.AprilTagTrackingCommand;
import com.team11855.frc2026.commands.AprilTagTrackingCommand.Mode;
import com.team11855.frc2026.subsystems.vision.AprilTagObservation;
import com.team11855.frc2026.subsystems.vision.VisionFieldPoseEstimate;
import com.team11855.lib.pathplanner.controllers.PathFollowingController;
import com.team11855.lib.pathplanner.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AprilTagTrackingCommandTest {
    private final FakeIO io = new FakeIO();
    private DriveSubsystem drive;
    private AprilTagTrackingCommand command;
    private Optional<AprilTagObservation> observation = Optional.empty();
    private double now = 10.0;
    private boolean enabled = true;
    private int reads;

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        drive = new DriveSubsystem(io, new RobotState(estimate -> {}), new UnusedPathController());
        start(Mode.FOLLOW_WHILE_HELD, new Transform3d());
    }

    private void start(Mode mode, Transform3d mount) {
        if (command != null) command.end(true);
        command = new AprilTagTrackingCommand(drive, () -> {
            reads++;
            return observation;
        }, () -> enabled, mode, mount, () -> now);
        command.initialize();
    }

    private void see(int id, double x, double y, double z) {
        observation = Optional.of(new AprilTagObservation(id, new Translation3d(x, y, z), now));
        command.execute();
    }

    @AfterEach
    void tearDown() {
        command.end(true);
        CommandScheduler.getInstance().unregisterSubsystem(drive);
    }

    @Test
    void aimTurnsTowardEitherSideWithoutTranslation() {
        start(Mode.AIM_ONCE, new Transform3d());
        see(21, 2.0, 1.0, 0.0);
        assertTrue(request().RotationalRate > 0.0);
        assertEquals(0.0, request().VelocityX);
        assertEquals(0.0, request().VelocityY);
        see(21, 2.0, -1.0, 0.0);
        assertTrue(request().RotationalRate < 0.0);
        assertEquals(0.0, request().VelocityX);
        assertFalse(command.isFinished());
    }

    @Test
    void followApproachesBacksAwayAndStopsInsideDistanceTolerance() {
        see(21, 4.0, 0.0, 0.0);
        assertEquals(0.6, request().VelocityX, 1e-9);
        see(21, 0.6, 0.0, 0.0);
        assertEquals(-0.4, request().VelocityX, 1e-9);
        see(21, 1.03, 0.0, 0.0);
        assertEquals(0.0, request().VelocityX);
        assertEquals(0.0, request().RotationalRate);
        assertFalse(command.isFinished());
    }

    @Test
    void measuresHorizontalRangeFromRobotCenterInsteadOfCameraOrSlantRange() {
        start(Mode.FOLLOW_WHILE_HELD,
                new Transform3d(new Translation3d(0.3, 0.2, 0.54), new Rotation3d()));
        see(21, 0.7, -0.2, 2.0);
        assertEquals(0.0, request().VelocityX);
        assertEquals(0.0, request().RotationalRate);
    }

    @Test
    void largeHeadingErrorRotatesFirstAndCapsAngularSpeed() {
        see(21, 1.0, 2.0, 0.0);
        assertEquals(0.0, request().VelocityX);
        assertEquals(0.0, request().VelocityY);
        assertEquals(1.5, request().RotationalRate, 1e-9);
        see(21, 1.0, -2.0, 0.0);
        assertEquals(-1.5, request().RotationalRate, 1e-9);
    }

    @Test
    void lostStaleFutureAndInvalidTargetsStopWhileFollowKeepsOwnership() {
        see(21, 2.0, 0.0, 0.0);
        now += 0.26;
        command.execute();
        assertStopped();
        assertFalse(command.isFinished());
        observation = Optional.of(new AprilTagObservation(21, new Translation3d(2, 0, 0), now + 1));
        command.execute();
        assertStopped();
        see(21, Double.NaN, 0.0, 0.0);
        assertStopped();
        see(21, 7.0, 0.0, 0.0);
        assertStopped();
        observation = Optional.empty();
        command.execute();
        assertStopped();
        assertFalse(command.isFinished());
    }

    @Test
    void followRejectsOtherTagsEvenBeforeFirstMatchAndAfterRestart() {
        see(19, 2.0, 0.0, 0.0);
        assertStopped();
        assertFalse(command.isFinished());
        see(21, 2.0, 0.0, 0.0);
        assertTrue(request().VelocityX > 0.0);
        see(19, 0.5, 0.0, 0.0);
        assertStopped();
        see(21, 0.5, 0.0, 0.0);
        assertTrue(request().VelocityX < 0.0);
        command.end(true);
        command.initialize();
        see(19, 0.5, 0.0, 0.0);
        assertStopped();
        see(21, 2.0, 0.0, 0.0);
        assertTrue(request().VelocityX > 0.0);
    }

    @Test
    void aimEndsWithoutMovingWhenOnlyAnotherTagIsVisible() {
        start(Mode.AIM_ONCE, new Transform3d());
        see(19, 2.0, 1.0, 0.0);
        assertStopped();
        assertTrue(command.isFinished());
    }

    @Test
    void controlGateStopsBeforeReadingCameraAndEndAlwaysStops() {
        assertTrue(command.getRequirements().contains(drive));
        see(21, 2.0, 0.0, 0.0);
        int previousReads = reads;
        enabled = false;
        command.execute();
        assertEquals(previousReads, reads);
        assertTrue(command.isFinished());
        assertStopped();
        enabled = true;
        command.initialize();
        see(21, 2.0, 0.0, 0.0);
        command.end(true);
        assertStopped();
    }

    @Test
    void aimMustStayAlignedAndTimesOutOrEndsWhenTargetLost() {
        start(Mode.AIM_ONCE, new Transform3d());
        see(21, 2.0, 0.0, 0.0);
        now += 0.10;
        see(21, 2.0, 0.2, 0.0); // Break the settle window.
        now += 0.10;
        see(21, 2.0, 0.0, 0.0);
        assertFalse(command.isFinished());
        now += 0.16;
        see(21, 2.0, 0.0, 0.0);
        assertTrue(command.isFinished());
        assertStopped();

        command.initialize();
        now += 3.01;
        see(21, 2.0, 1.0, 0.0);
        assertTrue(command.isFinished());
        assertStopped();

        command.initialize();
        observation = Optional.empty();
        command.execute();
        assertTrue(command.isFinished());
        assertStopped();
    }

    private SwerveRequest.RobotCentric request() {
        return assertInstanceOf(SwerveRequest.RobotCentric.class, io.lastRequest);
    }

    private void assertStopped() {
        var stop = assertInstanceOf(SwerveRequest.ApplyRobotSpeeds.class, io.lastRequest);
        assertEquals(0.0, stop.Speeds.vxMetersPerSecond);
        assertEquals(0.0, stop.Speeds.vyMetersPerSecond);
        assertEquals(0.0, stop.Speeds.omegaRadiansPerSecond);
    }

    private static class UnusedPathController implements PathFollowingController {
        @Override
        public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d pose, PathPlannerTrajectoryState target) {
            throw new AssertionError("No path should run in tag tracking tests");
        }

        @Override
        public void reset(Pose2d pose, ChassisSpeeds speeds) {}

        @Override
        public boolean isHolonomic() { return true; }
    }

    private static class FakeIO implements DriveIO {
        SwerveRequest lastRequest;

        @Override
        public void setControl(SwerveRequest request) { lastRequest = request; }

        @Override
        public void readInputs(DriveIOInputs inputs) {}

        @Override
        public void logModules(SwerveDriveState state) {}

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
