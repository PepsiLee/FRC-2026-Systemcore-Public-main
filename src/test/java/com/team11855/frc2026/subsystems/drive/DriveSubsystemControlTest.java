package com.team11855.frc2026.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.team11855.frc2026.RobotState;
import com.team11855.frc2026.subsystems.vision.VisionFieldPoseEstimate;
import com.team11855.lib.pathplanner.controllers.PathFollowingController;
import com.team11855.lib.pathplanner.trajectory.PathPlannerTrajectory;
import com.team11855.lib.pathplanner.trajectory.PathPlannerTrajectoryState;
import com.team11855.lib.pathplanner.util.DriveFeedforwards;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.LockSupport;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveSubsystemControlTest {
    private FakeIO io;
    private FakeController pathController;
    private DriveSubsystem drive;

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        io = new FakeIO();
        pathController = new FakeController();
        drive = new DriveSubsystem(io, new RobotState(estimate -> {}), pathController);
    }

    @AfterEach
    void tearDown() {
        drive.stop();
        CommandScheduler.getInstance().unregisterSubsystem(drive);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void manualRequestCancelsTrajectoryAndCommandEndStops() {
        drive.getController().accept(trajectory());
        tick();
        assertEquals(1.0, io.lastSpeeds.vxMetersPerSecond);

        Command manual =
                drive.applyRequest(
                        () -> new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(2.0, 0.0, 0.0)));
        assertTrue(manual.getRequirements().contains(drive));
        manual.initialize();
        manual.execute();
        int manualWriteCount = io.writes;
        tick();
        assertEquals(manualWriteCount, io.writes);
        assertEquals(2.0, io.lastSpeeds.vxMetersPerSecond);

        manual.end(true);
        assertStopped();
        tick();
        assertStopped();
    }

    @Test
    void stopSentinelIsImmediateAndNewTrajectoryCanResume() {
        drive.getController().accept(trajectory());
        tick();
        drive.getController().accept(PathPlannerTrajectory.makeStayInPlaceTrajectory());
        assertStopped();
        int stoppedWriteCount = io.writes;
        tick();
        assertEquals(stoppedWriteCount, io.writes);

        drive.getController().accept(trajectory());
        tick();
        assertEquals(1.0, io.lastSpeeds.vxMetersPerSecond);
        assertEquals(2, pathController.resets);
    }

    @Test
    void normalNonzeroEndReleasesTrajectoryWithoutStopPulse() {
        drive.getController().accept(trajectory());
        tick();
        int previousWrites = io.writes;
        drive.getController().accept(null);
        tick();
        assertEquals(previousWrites, io.writes);
        assertEquals(1.0, io.lastSpeeds.vxMetersPerSecond);
        assertEquals(1, pathController.resets);
    }

    @Test
    void invalidTrajectoryStopsBeforeControllerCanSampleIt() {
        for (double invalidTime : new double[] {Double.NaN, Double.POSITIVE_INFINITY}) {
            drive.getController().accept(trajectory());
            tick();
            int previousResets = pathController.resets;
            PathPlannerTrajectoryState invalid = new PathPlannerTrajectoryState();
            invalid.timeSeconds = invalidTime;
            drive.getController().accept(new PathPlannerTrajectory(List.of(invalid)));
            assertStopped();
            int stoppedWrites = io.writes;
            tick();
            assertEquals(stoppedWrites, io.writes);
            assertEquals(previousResets, pathController.resets);
        }
        drive.getController().accept(new PathPlannerTrajectory(List.of()));
        assertStopped();
    }

    @Test
    void resettingOdometryCancelsTrajectoryAndStopsFirst() {
        drive.getController().accept(trajectory());
        tick();
        Pose2d pose = new Pose2d(3.0, 2.0, new Rotation2d());
        drive.resetOdometry(pose);
        assertEquals(pose, io.resetPose);
        assertTrue(io.wasStoppedAtReset);
        int resetWriteCount = io.writes;
        tick();
        assertEquals(resetWriteCount, io.writes);
    }

    @Test
    void inFlightTrajectoryCannotOverwriteCompletedStop() throws InterruptedException {
        pathController.calculationStarted = new CountDownLatch(1);
        pathController.finishCalculation = new CountDownLatch(1);
        AtomicReference<Throwable> failure = new AtomicReference<>();
        drive.getController().accept(trajectory());
        Thread trajectoryTick = checkedThread(this::tick, failure);
        Thread stop = checkedThread(drive::stop, failure);

        try {
            trajectoryTick.start();
            assertTrue(pathController.calculationStarted.await(2, TimeUnit.SECONDS));
            stop.start();
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
            while (stop.isAlive()
                    && stop.getState() != Thread.State.BLOCKED
                    && System.nanoTime() < deadline) {
                LockSupport.parkNanos(TimeUnit.MILLISECONDS.toNanos(1));
            }
            assertEquals(Thread.State.BLOCKED, stop.getState());
        } finally {
            pathController.finishCalculation.countDown();
            trajectoryTick.join(2000);
            stop.join(2000);
        }

        assertFalse(trajectoryTick.isAlive());
        assertFalse(stop.isAlive());
        assertNull(failure.get());
        assertStopped();
        int stoppedWriteCount = io.writes;
        tick();
        assertEquals(stoppedWriteCount, io.writes);
    }

    private void tick() {
        ((Runnable) drive.getController()).run();
    }

    private void assertStopped() {
        assertEquals(0.0, io.lastSpeeds.vxMetersPerSecond);
        assertEquals(0.0, io.lastSpeeds.vyMetersPerSecond);
        assertEquals(0.0, io.lastSpeeds.omegaRadiansPerSecond);
    }

    private static PathPlannerTrajectory trajectory() {
        PathPlannerTrajectoryState state = new PathPlannerTrajectoryState();
        state.feedforwards = DriveFeedforwards.zeros(4);
        return new PathPlannerTrajectory(List.of(state));
    }

    private static Thread checkedThread(Runnable action, AtomicReference<Throwable> failure) {
        Thread thread =
                new Thread(
                        () -> {
                            try {
                                action.run();
                            } catch (Throwable error) {
                                failure.compareAndSet(null, error);
                            }
                        });
        thread.setDaemon(true);
        return thread;
    }

    private static class FakeController implements PathFollowingController {
        int resets;
        CountDownLatch calculationStarted;
        CountDownLatch finishCalculation;

        @Override
        public ChassisSpeeds calculateRobotRelativeSpeeds(
                Pose2d currentPose, PathPlannerTrajectoryState targetState) {
            if (calculationStarted != null) {
                calculationStarted.countDown();
                try {
                    if (!finishCalculation.await(5, TimeUnit.SECONDS)) {
                        throw new AssertionError("Timed out waiting to finish trajectory tick");
                    }
                } catch (InterruptedException error) {
                    Thread.currentThread().interrupt();
                    throw new AssertionError(error);
                }
            }
            return new ChassisSpeeds(1.0, 0.0, 0.0);
        }

        @Override
        public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
            assertNotNull(currentPose);
            assertNotNull(currentSpeeds);
            resets++;
        }

        @Override
        public boolean isHolonomic() {
            return true;
        }
    }

    private static class FakeIO implements DriveIO {
        volatile ChassisSpeeds lastSpeeds = new ChassisSpeeds();
        volatile int writes;
        Pose2d resetPose;
        boolean wasStoppedAtReset;

        @Override
        public void setControl(SwerveRequest request) {
            ChassisSpeeds speeds = ((ApplyRobotSpeeds) request).Speeds;
            lastSpeeds =
                    new ChassisSpeeds(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond);
            writes++;
        }

        @Override
        public void resetOdometry(Pose2d pose) {
            resetPose = pose;
            wasStoppedAtReset =
                    lastSpeeds.vxMetersPerSecond == 0.0
                            && lastSpeeds.vyMetersPerSecond == 0.0
                            && lastSpeeds.omegaRadiansPerSecond == 0.0;
        }

        @Override
        public void readInputs(DriveIOInputs inputs) {}

        @Override
        public void logModules(SwerveDriveState driveState) {}

        @Override
        public Command applyRequest(Supplier<SwerveRequest> supplier, Subsystem requirement) {
            throw new AssertionError("Commands must use DriveSubsystem's synchronized API");
        }

        @Override
        public void addVisionMeasurement(VisionFieldPoseEstimate estimate) {}

        @Override
        public void setStateStdDevs(double x, double y, double rotation) {}
    }
}
