package com.team11855.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import com.team11855.frc2026.Constants.VisionConstants;
import com.team11855.frc2026.RobotState;
import com.team11855.lib.limelight.LimelightHelpers;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

class SingleCameraVisionTest {
    private static final Pose2d OBSERVED_POSE = new Pose2d(2.0, 3.0, Rotation2d.kZero);
    private final List<VisionFieldPoseEstimate> accepted = new ArrayList<>();
    private FakeCamera io;
    private RobotState state;
    private VisionSubsystem vision;
    private double now = 1.05;

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        io = new FakeCamera();
        state = new RobotState(accepted::add);
        state.addOdometryMeasurement(1.0, OBSERVED_POSE);
        vision = new VisionSubsystem(io, state, () -> now);
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().unregisterSubsystem(vision);
    }

    @Test
    void oneCameraForwardsOneMt1MeasurementWith254Uncertainty() {
        vision.periodic();

        assertEquals(1, accepted.size());
        var estimate = accepted.get(0);
        assertEquals(OBSERVED_POSE, estimate.getVisionRobotPoseMeters());
        assertEquals(1.0, estimate.getTimestampSeconds());
        assertEquals(2, estimate.getNumTags());
        assertEquals(0.5, estimate.getVisionMeasurementStdDevs().get(0, 0), 1e-9);
        assertEquals(0.5, estimate.getVisionMeasurementStdDevs().get(1, 0), 1e-9);
        assertEquals(0.1, estimate.getVisionMeasurementStdDevs().get(2, 0), 1e-9);
        assertEquals(VisionSubsystem.EstimateSource.MEGATAG, vision.getLastEstimateSource());
    }

    @Test
    void duplicateAndOlderFramesAreNotFusedAgain() {
        vision.periodic();
        vision.periodic();
        io.timestamp = 0.9;
        vision.periodic();
        assertEquals(1, accepted.size());

        io.timestamp = 2.0;
        now = 2.05;
        state.addOdometryMeasurement(2.0, OBSERVED_POSE);
        vision.periodic();
        assertEquals(2, accepted.size());
        assertEquals(2.0, state.lastUsedMegatagTimestamp());
    }

    @Test
    void noTargetDoesNotForwardAnOldPoseAndCanRecover() {
        vision.periodic();
        io.timestamp = 2.0;
        now = 2.05;
        io.seesTarget = false;
        io.hasFieldPose = false;
        state.addOdometryMeasurement(2.0, OBSERVED_POSE);
        vision.periodic();
        assertEquals(1, accepted.size());
        assertEquals(1.0, state.lastUsedMegatagTimestamp());

        io.seesTarget = true;
        io.hasFieldPose = true;
        vision.periodic();
        assertEquals(2, accepted.size());
    }

    @Test
    void visionToggleGatesTheExistingRobotStateCallback() {
        vision.setUseVision(false);
        vision.periodic();
        assertTrue(accepted.isEmpty());
        assertEquals(0.0, state.lastUsedMegatagTimestamp());

        vision.setUseVision(true);
        vision.periodic();
        assertEquals(1, accepted.size());
    }

    @Test
    void realAndSimMountingUseTheSamePositionWithTheirOwnAxisConventions() {
        new VisionIOHardwareLimelight(state);
        double[] published =
                NetworkTableInstance.getDefault()
                        .getTable(VisionConstants.kLimelightTableName)
                        .getEntry("camerapose_robotspace_set")
                        .getDoubleArray(new double[0]);
        assertArrayEquals(
                new double[] {
                    VisionConstants.kCameraForwardMeters,
                    VisionConstants.kCameraRightMeters,
                    VisionConstants.kCameraHeightMeters,
                    0.0,
                    VisionConstants.kCameraPitchDegrees,
                    VisionConstants.kCameraYawDegrees
                },
                published,
                1e-9);

        assertEquals(
                21.0,
                NetworkTableInstance.getDefault()
                        .getTable(VisionConstants.kLimelightTableName)
                        .getEntry("priorityid")
                        .getDouble(-1.0));
        var simMount = VisionConstants.kRobotToCamera;
        assertEquals(published[0], simMount.getX(), 1e-9);
        assertEquals(-published[1], simMount.getY(), 1e-9);
        assertEquals(published[2], simMount.getZ(), 1e-9);
        // Check the optical forward direction, including pitch-up and the configured yaw.
        var opticalAxis = new Translation3d(1.0, 0.0, 0.0).rotateBy(simMount.getRotation());
        double pitch = Math.toRadians(published[4]);
        double yaw = Math.toRadians(published[5]);
        assertEquals(Math.cos(pitch) * Math.cos(yaw), opticalAxis.getX(), 1e-9);
        assertEquals(Math.cos(pitch) * Math.sin(yaw), opticalAxis.getY(), 1e-9);
        assertEquals(Math.sin(pitch), opticalAxis.getZ(), 1e-9);
    }

    @Test
    void relativeTagRemainsAvailableWhenFieldFusionIsDisabled() {
        io.relativeTarget = Optional.of(new AprilTagObservation(18, new Translation3d(2, 0, 0), 1));
        vision.setUseVision(false);
        vision.periodic();
        assertTrue(accepted.isEmpty());
        assertEquals(io.relativeTarget, vision.getAprilTagObservation());
        io.seesTarget = false;
        vision.periodic();
        assertTrue(vision.getAprilTagObservation().isEmpty());
    }

    @Test
    void hardwareUsesTargetTimestampAndClearsRelativeDataWhenLost() {
        var hardware = new VisionIOHardwareLimelight(state);
        var inputs = new VisionIO.VisionIOInputs();
        var table = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
        var targetEntry =
                LimelightHelpers.getLimelightDoubleArrayEntry(
                        VisionConstants.kLimelightTableName, "targetpose_cameraspace");
        try {
            table.getEntry("tv").setDouble(1.0);
            table.getEntry("tid").setDouble(18.0);
            table.getEntry("cl").setDouble(10.0);
            table.getEntry("tl").setDouble(20.0);
            targetEntry.set(new double[] {0.2, -0.1, 2.0, 0, 0, 0}, 10_000_000L);
            hardware.readInputs(inputs);
            var first = inputs.camera.aprilTagObservation.orElseThrow();
            assertEquals(9.97, first.timestampSeconds(), 1e-9);
            assertEquals(new Translation3d(2.0, -0.2, 0.1), first.cameraToTag());
            hardware.readInputs(inputs);
            assertEquals(first, inputs.camera.aprilTagObservation.orElseThrow());
            table.getEntry("tv").setDouble(0.0);
            hardware.readInputs(inputs);
            assertTrue(inputs.camera.aprilTagObservation.isEmpty());
        } finally {
            table.getEntry("tv").setDouble(0.0);
            table.getEntry("tid").setDouble(-1.0);
            table.getEntry("cl").setDouble(0.0);
            table.getEntry("tl").setDouble(0.0);
            targetEntry.set(new double[0]);
        }
    }

    @Test
    void otherTagsCanStillLocalizeWhenPriorityTargetIsNotVisible() {
        io.seesTarget = false;
        io.hasFieldPose = true; // The two field tags are 18 and 19, not target 21.
        vision.periodic();
        assertEquals(1, accepted.size());
        assertTrue(vision.getAprilTagObservation().isEmpty());
    }

    @Test
    void hardwareReadsFieldPoseWithTvZeroAndClearsItWhenNoFieldPoseRemains() {
        var hardware = new VisionIOHardwareLimelight(state);
        var inputs = new VisionIO.VisionIOInputs();
        var table = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
        var poseEntry =
                LimelightHelpers.getLimelightDoubleArrayEntry(
                        VisionConstants.kLimelightTableName, "botpose_wpiblue");
        try {
            table.getEntry("tv").setDouble(0.0);
            poseEntry.set(
                    new double[] {
                        2, 3, 0, 0, 0, 0, 0, 2, 0, 1, 4, 18, 0, 0, 4, 1, 1, 0.01, 19, 0, 0, 4, 1, 1,
                        0.01
                    },
                    1_000_000L);
            hardware.readInputs(inputs);
            assertFalse(inputs.camera.seesTarget);
            assertTrue(inputs.camera.aprilTagObservation.isEmpty());
            assertEquals(2, inputs.camera.megatagCount);
            assertEquals(OBSERVED_POSE, inputs.camera.megatagPoseEstimate.fieldToRobot());
            poseEntry.set(new double[0]);
            hardware.readInputs(inputs);
            assertEquals(0, inputs.camera.megatagCount);
            assertNull(inputs.camera.megatagPoseEstimate);
        } finally {
            poseEntry.set(new double[0]);
        }
    }

    private static class FakeCamera implements VisionIO {
        boolean seesTarget = true;
        boolean hasFieldPose = true;
        Optional<AprilTagObservation> relativeTarget = Optional.empty();
        double timestamp = 1.0;

        @Override
        public void readInputs(VisionIOInputs inputs) {
            var camera = inputs.camera;
            camera.seesTarget = seesTarget;
            camera.aprilTagObservation = relativeTarget;
            camera.megatagCount = hasFieldPose ? 2 : 0;
            camera.megatagPoseEstimate =
                    new MegatagPoseEstimate(
                            OBSERVED_POSE, timestamp, 0.02, 4.0, 1.0, new int[] {18, 19});
            camera.pose3d = new Pose3d(OBSERVED_POSE);
            camera.fiducialObservations =
                    new FiducialObservation[] {
                        new FiducialObservation(18, 0, 0, 0.01, 4),
                        new FiducialObservation(19, 0, 0, 0.01, 4)
                    };
            camera.standardDeviations =
                    new double[] {0.4, 0.5, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
    }
}
