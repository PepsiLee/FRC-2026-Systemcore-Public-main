package com.team11855.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import com.team11855.frc2026.Constants;
import com.team11855.frc2026.Constants.VisionConstants;
import com.team11855.frc2026.RobotState;
import com.team11855.lib.limelight.LimelightHelpers;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

class MegatagVisionTest {
    private final List<VisionFieldPoseEstimate> accepted = new ArrayList<>();
    private final FakeCamera io = new FakeCamera();
    private RobotState state;
    private VisionSubsystem vision;
    private double now = 10.05;

    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        state = new RobotState(accepted::add);
        state.addOdometryMeasurement(10, new Pose2d(2, 3, Rotation2d.fromDegrees(40)));
        vision = new VisionSubsystem(io, state, () -> now);
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().unregisterSubsystem(vision);
        for (String key :
                List.of("botpose_wpiblue", "botpose_orb_wpiblue", "targetpose_cameraspace")) {
            LimelightHelpers.getLimelightDoubleArrayEntry(VisionConstants.kLimelightTableName, key)
                    .set(new double[0]);
        }
        var table = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
        table.getEntry("tv").setDouble(0);
        table.getEntry("stddevs").setDoubleArray(new double[12]);
    }

    @Test
    void multiTagUsesMt1TranslationAndHeadingAndOnlyOneCallbackPerFrame() {
        vision.periodic();
        assertEquals(VisionSubsystem.EstimateSource.MEGATAG, vision.getLastEstimateSource());
        var measurement = accepted.get(0);
        assertEquals(io.pose, measurement.getVisionRobotPoseMeters());
        assertEquals(10, measurement.getTimestampSeconds());
        assertEquals(0.5, measurement.getVisionMeasurementStdDevs().get(0, 0), 1e-9);
        assertEquals(0.5, measurement.getVisionMeasurementStdDevs().get(1, 0), 1e-9);
        assertEquals(0.1, measurement.getVisionMeasurementStdDevs().get(2, 0), 1e-9);
        vision.periodic();
        assertEquals(1, accepted.size());
        assertEquals(
                VisionSubsystem.RejectionReason.DUPLICATE_OR_OLDER,
                vision.getLastRejectionReason());
    }

    @Test
    void goodSingleTagPrefersMt1EvenWhenGyroCandidateIsAlsoValid() {
        io.ids = new int[] {18};
        io.area = 4;
        io.ambiguity = 0.1;
        vision.periodic();
        assertEquals(1, accepted.size());
        assertEquals(VisionSubsystem.EstimateSource.MEGATAG, vision.getLastEstimateSource());
        assertEquals(VisionSubsystem.RejectionReason.ACCEPTED, vision.getGyroRejectionReason());
        assertEquals(io.pose, accepted.get(0).getVisionRobotPoseMeters());
        assertEquals(0.5 / 0.9, accepted.get(0).getVisionMeasurementStdDevs().get(0, 0), 1e-9);
        assertEquals(0.1 / 0.9, accepted.get(0).getVisionMeasurementStdDevs().get(2, 0), 1e-9);
    }

    @Test
    void singleTagFallbackRecomputesTranslationWithHistoricalYaw() {
        io.ids = new int[] {18};
        io.area = 0.5; // MT1 branch fails its area gate.
        var tag = Constants.kAprilTagLayoutReefsOnly.getTagPose(18).orElseThrow();
        io.pose = new Pose2d(tag.getX() - 2, tag.getY(), Rotation2d.kZero);
        state.addOdometryMeasurement(10, new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
        // The newer heading must not replace the heading at the image's capture timestamp.
        state.addOdometryMeasurement(10.04, new Pose2d(2, 3, Rotation2d.kZero));
        vision.periodic();

        assertEquals(1, accepted.size());
        assertEquals(VisionSubsystem.EstimateSource.GYRO, vision.getLastEstimateSource());
        assertEquals(
                VisionSubsystem.RejectionReason.TAG_TOO_SMALL, vision.getMegatagRejectionReason());
        var measurement = accepted.get(0);
        assertEquals(tag.getX(), measurement.getVisionRobotPoseMeters().getX(), 1e-9);
        assertEquals(tag.getY() - 2, measurement.getVisionRobotPoseMeters().getY(), 1e-9);
        assertEquals(90, measurement.getVisionRobotPoseMeters().getRotation().getDegrees(), 1e-9);
        assertEquals(0.5, measurement.getVisionMeasurementStdDevs().get(0, 0), 1e-9);
        assertEquals(1e6, measurement.getVisionMeasurementStdDevs().get(2, 0), 1e-9);
        assertEquals(10, measurement.getTimestampSeconds());
        vision.periodic();
        assertEquals(1, accepted.size());
    }

    @Test
    void ambiguityRejectionCanUse254GyroFallback() {
        io.ids = new int[] {18};
        io.ambiguity = 0.3;
        vision.periodic();
        assertEquals(
                VisionSubsystem.RejectionReason.AMBIGUOUS_TAG, vision.getMegatagRejectionReason());
        assertEquals(VisionSubsystem.EstimateSource.GYRO, vision.getLastEstimateSource());
        assertEquals(1, accepted.size());
    }

    @Test
    void smallerSingleTagChecksYawButLargeSingleTagDoesNot() {
        io.ids = new int[] {18};
        io.area = 1.5; // MT1 yaw is 30 degrees; historical yaw is 40.
        vision.periodic();
        assertEquals(
                VisionSubsystem.RejectionReason.YAW_MISMATCH, vision.getMegatagRejectionReason());
        assertEquals(VisionSubsystem.EstimateSource.GYRO, vision.getLastEstimateSource());
        io.timestamp = 10.02;
        io.area = 2.0;
        vision.periodic();
        assertEquals(VisionSubsystem.EstimateSource.MEGATAG, vision.getLastEstimateSource());
        assertEquals(2, accepted.size());
    }

    @Test
    void singleTagYawCheckWrapsAcross180Degrees() {
        io.ids = new int[] {18};
        io.area = 1.5;
        io.pose = new Pose2d(2, 3, Rotation2d.fromDegrees(-179));
        state.addOdometryMeasurement(10, new Pose2d(2, 3, Rotation2d.fromDegrees(179)));
        vision.periodic();
        assertEquals(VisionSubsystem.EstimateSource.MEGATAG, vision.getLastEstimateSource());
    }

    @Test
    void multiTagSkipsSingleTagQualityAndGyroRateGates() {
        io.area = 0.5;
        io.ambiguity = 0.8;
        setYawRate(6);
        vision.periodic();
        assertEquals(1, accepted.size());
        assertEquals(VisionSubsystem.EstimateSource.MEGATAG, vision.getLastEstimateSource());
        assertEquals(
                VisionSubsystem.RejectionReason.NOT_SINGLE_TAG, vision.getGyroRejectionReason());
    }

    @Test
    void failedMultiTagHasNoGyroFallback() {
        io.z = 0.21;
        reject(VisionSubsystem.RejectionReason.INVALID_HEIGHT);
        assertEquals(
                VisionSubsystem.RejectionReason.NOT_SINGLE_TAG, vision.getGyroRejectionReason());
        io.z = 0;
        io.pose = new Pose2d(0.1, 0.1, Rotation2d.kZero);
        reject(VisionSubsystem.RejectionReason.NEAR_FIELD_ORIGIN);
    }

    @Test
    void singleTagFallbackRetains254SeparateHeightAndExclusiveTagPolicy() {
        io.ids = new int[] {18};
        io.z = 0.3;
        vision.periodic();
        assertEquals(
                VisionSubsystem.RejectionReason.INVALID_HEIGHT, vision.getMegatagRejectionReason());
        assertEquals(VisionSubsystem.EstimateSource.GYRO, vision.getLastEstimateSource());
        io.timestamp = 10.02;
        io.z = 0;
        state.setExclusiveTag(21);
        vision.periodic();
        assertEquals(
                VisionSubsystem.RejectionReason.EXCLUSIVE_TAG_MISMATCH,
                vision.getMegatagRejectionReason());
        assertEquals(VisionSubsystem.EstimateSource.GYRO, vision.getLastEstimateSource());
        assertEquals(2, accepted.size());
    }

    @Test
    void exclusiveTagMustBeIncludedInMultiTagEstimate() {
        state.setExclusiveTag(21);
        reject(VisionSubsystem.RejectionReason.EXCLUSIVE_TAG_MISMATCH);
        state.setExclusiveTag(18);
        vision.periodic();
        assertEquals(1, accepted.size());
        assertEquals(VisionSubsystem.EstimateSource.MEGATAG, vision.getLastEstimateSource());
    }

    @Test
    void fallbackRejectsFastRotationInBothDirections() {
        io.ids = new int[] {18};
        io.area = 0.5;
        for (double rate : new double[] {6, -6, Double.NaN}) {
            setYawRate(rate);
            reject(VisionSubsystem.RejectionReason.TAG_TOO_SMALL);
            assertEquals(
                    VisionSubsystem.RejectionReason.ROTATING_TOO_FAST,
                    vision.getGyroRejectionReason());
        }
    }

    @Test
    void fallbackRejectsTagOutsideReefLayout() {
        io.ids = new int[] {1};
        io.area = 0.5;
        reject(VisionSubsystem.RejectionReason.TAG_TOO_SMALL);
        assertEquals(VisionSubsystem.RejectionReason.UNKNOWN_TAG, vision.getGyroRejectionReason());
    }

    @Test
    void staleFutureAndInvalidTimestampsRejectBothBranches() {
        now = 10.6;
        reject(VisionSubsystem.RejectionReason.STALE);
        now = 9.9;
        reject(VisionSubsystem.RejectionReason.STALE);
        now = 10.05;
        for (double timestamp : new double[] {Double.NaN, 0, Double.POSITIVE_INFINITY}) {
            io.timestamp = timestamp;
            reject(VisionSubsystem.RejectionReason.INVALID_TIMESTAMP);
        }
    }

    @Test
    void missingMalformedOrNonfiniteDataCannotReachEitherBranch() {
        io.ids = new int[0];
        reject(VisionSubsystem.RejectionReason.MISSING_MT1);
        io.ids = new int[] {0};
        reject(VisionSubsystem.RejectionReason.INVALID_FIDUCIALS);
        io.ids = new int[] {18};
        io.pose = new Pose2d(Double.NaN, 3, Rotation2d.kZero);
        reject(VisionSubsystem.RejectionReason.INVALID_POSE);
        io.pose = new Pose2d(2, 3, Rotation2d.kZero);
        io.z = Double.NaN;
        reject(VisionSubsystem.RejectionReason.INVALID_POSE);
    }

    @Test
    void missingOrZeroTranslationUncertaintyRejectsBothBranches() {
        io.ids = new int[] {18};
        io.area = 0.5;
        for (double[] stddevs :
                new double[][] {
                    null,
                    new double[0],
                    new double[12],
                    {Double.NaN, 0.5, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0},
                    {0.4, -0.5, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0}
                }) {
            io.stddevs = stddevs;
            reject(VisionSubsystem.RejectionReason.INVALID_STD_DEVS);
        }
    }

    @Test
    void hardwareNeedsOnlyMt1AndDoesNotPublishOrientationOrImuMode() {
        var table = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
        double[] previousOrientation = {12, 34, 0, 0, 0, 0};
        table.getEntry("robot_orientation_set").setDoubleArray(previousOrientation);
        table.getEntry("imumode_set").setDouble(2);
        var hardware = new VisionIOHardwareLimelight(state);
        var mt1 =
                LimelightHelpers.getLimelightDoubleArrayEntry(
                        VisionConstants.kLimelightTableName, "botpose_wpiblue");
        var mt2 =
                LimelightHelpers.getLimelightDoubleArrayEntry(
                        VisionConstants.kLimelightTableName, "botpose_orb_wpiblue");
        mt1.set(
                new double[] {
                    2, 3, 0, 0, 0, 30, 20, 2, 0, 2, 4, 18, 0, 0, 4, 2, 2, 0.1, 19, 0, 0, 4, 2, 2,
                    0.1
                },
                10_020_000L);
        mt2.set(new double[0]);
        table.getEntry("stddevs").setDoubleArray(io.stddevs);
        var hardwareVision = new VisionSubsystem(hardware, state, () -> now);
        try {
            hardwareVision.periodic();
            assertEquals(1, accepted.size());
            assertEquals(io.pose, accepted.get(0).getVisionRobotPoseMeters());
            assertEquals(10, accepted.get(0).getTimestampSeconds(), 1e-9);
            assertArrayEquals(
                    previousOrientation,
                    table.getEntry("robot_orientation_set").getDoubleArray(new double[0]),
                    1e-9);
            assertEquals(2, table.getEntry("imumode_set").getDouble(-1));
            var inputs = new VisionIO.VisionIOInputs();
            hardware.readInputs(inputs);
            assertEquals(0.02, inputs.camera.megatagPoseEstimate.latency(), 1e-9);
            mt1.set(new double[0]);
            hardware.readInputs(inputs);
            assertEquals(0, inputs.camera.megatagCount);
            assertNull(inputs.camera.megatagPoseEstimate);
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(hardwareVision);
            table.getEntry("robot_orientation_set").setDoubleArray(new double[6]);
            table.getEntry("imumode_set").setDouble(0);
        }
    }

    @Test
    void malformedMt1MetadataIsCleared() {
        var hardware = new VisionIOHardwareLimelight(state);
        var inputs = new VisionIO.VisionIOInputs();
        var mt1 =
                LimelightHelpers.getLimelightDoubleArrayEntry(
                        VisionConstants.kLimelightTableName, "botpose_wpiblue");
        for (double[] data :
                new double[][] {
                    {2, 3, 0, 0, 0, 0, 20, 2},
                    {2, 3, 0, 0, 0, 0, 20, Double.NaN, 0, 2, 4}
                }) {
            mt1.set(data, 10_020_000L);
            hardware.readInputs(inputs);
            assertNull(inputs.camera.megatagPoseEstimate);
        }
    }

    @Test
    void poseStructSizeMatchesQualityField() {
        var original = new MegatagPoseEstimate(io.pose, 10, 0.02, 4, 0.8, new int[] {18});
        var buffer = ByteBuffer.allocate(MegatagPoseEstimate.struct.getSize());
        MegatagPoseEstimate.struct.pack(buffer, original);
        assertEquals(buffer.capacity(), buffer.position());
        buffer.flip();
        var decoded = MegatagPoseEstimate.struct.unpack(buffer);
        assertEquals(original.fieldToRobot(), decoded.fieldToRobot());
        assertEquals(original.quality(), decoded.quality());
    }

    private void reject(VisionSubsystem.RejectionReason reason) {
        vision.periodic();
        assertTrue(accepted.isEmpty());
        assertEquals(reason, vision.getLastRejectionReason());
        assertEquals(VisionSubsystem.EstimateSource.NONE, vision.getLastEstimateSource());
    }

    private void setYawRate(double rate) {
        var speeds = new ChassisSpeeds(0, 0, rate);
        state.addDriveMotionMeasurements(
                10, 0, 0, rate, 0, 0, 0, 0, speeds, speeds, speeds, speeds, speeds);
    }

    private static class FakeCamera implements VisionIO {
        int[] ids = {18, 19};
        double timestamp = 10;
        Pose2d pose = new Pose2d(2, 3, Rotation2d.fromDegrees(30));
        double area = 4;
        double ambiguity = 0.1;
        double z;
        double[] stddevs = {0.4, 0.5, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0};

        @Override
        public void readInputs(VisionIOInputs inputs) {
            var cam = inputs.camera;
            cam.seesTarget = ids.length > 0;
            cam.megatagCount = ids.length;
            cam.megatagPoseEstimate =
                    new MegatagPoseEstimate(
                            pose, timestamp, 0.02, area, ids.length > 1 ? 1 : 1 - ambiguity, ids);
            cam.pose3d =
                    new Pose3d(
                            pose.getX(),
                            pose.getY(),
                            z,
                            new Rotation3d(0, 0, pose.getRotation().getRadians()));
            cam.fiducialObservations =
                    Arrays.stream(ids)
                            .mapToObj(id -> new FiducialObservation(id, 0, 0, ambiguity, area))
                            .toArray(FiducialObservation[]::new);
            cam.standardDeviations = stddevs;
        }
    }
}
