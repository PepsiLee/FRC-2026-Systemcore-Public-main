package com.team11855.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import com.team11855.frc2026.Constants;
import com.team11855.frc2026.Constants.VisionConstants;
import com.team11855.frc2026.RobotState;
import com.team11855.lib.limelight.LimelightHelpers;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class MegatagVisionTest {
    private final List<VisionFieldPoseEstimate> accepted = new ArrayList<>();
    private final FakeCamera io = new FakeCamera();
    private RobotState state;
    private VisionSubsystem vision;
    private double now = 10.05;

    @BeforeAll
    static void initializeHal() { assertTrue(HAL.initialize(500, 0)); }

    @BeforeEach
    void setUp() {
        state = new RobotState(accepted::add);
        state.addOdometryMeasurement(10, new Pose2d(2, 3, Rotation2d.fromDegrees(40)));
        vision = new VisionSubsystem(io, state, () -> now);
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().unregisterSubsystem(vision);
        for (String key : List.of("botpose_wpiblue", "botpose_orb_wpiblue", "targetpose_cameraspace")) {
            LimelightHelpers.getLimelightDoubleArrayEntry(VisionConstants.kLimelightTableName, key).set(new double[0]);
        }
        NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName).getEntry("tv").setDouble(0);
    }

    @Test
    void combinesMt2TranslationWithMt1HeadingAndCallsConsumerOnlyOnce() {
        vision.periodic();
        vision.periodic();
        assertEquals(1, accepted.size());
        var measurement = accepted.get(0);
        assertEquals(new Pose2d(4, 5, Rotation2d.fromDegrees(30)), measurement.getVisionRobotPoseMeters());
        assertEquals(10, measurement.getTimestampSeconds());
        assertEquals(0.4, measurement.getVisionMeasurementStdDevs().get(0, 0), 1e-9);
        assertEquals(10, measurement.getVisionMeasurementStdDevs().get(2, 0), 1e-9);
        assertEquals(VisionSubsystem.RejectionReason.DUPLICATE_OR_OLDER, vision.getLastRejectionReason());
    }

    @Test
    void orientationIsSentBeforeReadInDegreesAndDegreesPerSecond() {
        setYawRate(Math.PI / 2);
        vision.periodic();
        assertTrue(io.sentBeforeRead);
        assertEquals(40, io.yaw, 1e-9);
        assertEquals(90, io.rate, 1e-9);
        new VisionIOHardwareLimelight(state).setRobotOrientation(io.yaw, io.rate);
        var table = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
        assertArrayEquals(new double[] {40, 90, 0, 0, 0, 0}, table.getEntry("robot_orientation_set").getDoubleArray(new double[0]), 1e-9);
        assertEquals(0, table.getEntry("imumode_set").getDouble(-1));
    }

    @Test
    void missingEitherAlgorithmRejectsWithoutMt1OnlyFallbackAndCanRecover() {
        io.mt2Count = 0;
        reject(VisionSubsystem.RejectionReason.MISSING_MT2);
        io.mt2Count = 2;
        io.mt1Count = 0;
        reject(VisionSubsystem.RejectionReason.MISSING_MT1);
        io.mt1Count = 2;
        vision.periodic();
        assertEquals(1, accepted.size());
    }

    @Test
    void rejectsStaleFutureAndInvalidTimestamps() {
        now = 10.6;
        reject(VisionSubsystem.RejectionReason.STALE);
        now = 9.9;
        reject(VisionSubsystem.RejectionReason.STALE);
        now = 10.05;
        io.mt1Time = Double.NaN;
        reject(VisionSubsystem.RejectionReason.INVALID_TIMESTAMP);
        io.mt1Time = 10;
        io.mt2Time = 0;
        reject(VisionSubsystem.RejectionReason.INVALID_TIMESTAMP);
    }

    @Test
    void rejectsMismatchedFrameTimesAndReusedMt1Heading() {
        io.mt1Time = 9.9;
        reject(VisionSubsystem.RejectionReason.UNSYNCHRONIZED);
        io.mt1Time = 10;
        vision.periodic();
        io.mt2Time = 10.02;
        vision.periodic();
        assertEquals(1, accepted.size());
        assertEquals(VisionSubsystem.RejectionReason.DUPLICATE_OR_OLDER, vision.getLastRejectionReason());
        io.mt1Time = 10.02;
        vision.periodic();
        assertEquals(2, accepted.size());
    }

    @Test
    void rejectsFastRotationInBothDirections() {
        for (double rate : new double[] {6, -6}) {
            setYawRate(rate);
            reject(VisionSubsystem.RejectionReason.ROTATING_TOO_FAST);
        }
    }

    @Test
    void rejectsInvalidPoseDistanceAndBoundsButAllowsNearOrigin() {
        io.mt2Pose = new Pose2d(Double.NaN, 2, Rotation2d.kZero);
        reject(VisionSubsystem.RejectionReason.INVALID_POSE);
        io.mt2Pose = new Pose2d(Constants.kFieldLengthMeters + 1, 2, Rotation2d.kZero);
        reject(VisionSubsystem.RejectionReason.OUTSIDE_FIELD);
        io.mt2Pose = new Pose2d(0.1, 0.1, Rotation2d.kZero);
        for (double distance : new double[] {0, -1, Double.NaN, 7}) {
            io.distance = distance;
            reject(VisionSubsystem.RejectionReason.TAG_TOO_FAR);
        }
        io.distance = 2;
        vision.periodic();
        assertEquals(1, accepted.size());
    }

    @Test
    void singleTagUsesLargerTranslationUncertainty() {
        io.mt1Count = 1;
        io.mt2Count = 1;
        vision.periodic();
        assertEquals(1, accepted.size());
        assertEquals(0.98, accepted.get(0).getVisionMeasurementStdDevs().get(0, 0), 1e-9);
        assertEquals(1, accepted.get(0).getNumTags());
    }

    @Test
    void hardwareReadsBothChannelsAndPreservesTimestampAndClearsMissingChannel() {
        var hardware = new VisionIOHardwareLimelight(state);
        var inputs = new VisionIO.VisionIOInputs();
        var mt1 = LimelightHelpers.getLimelightDoubleArrayEntry(VisionConstants.kLimelightTableName, "botpose_wpiblue");
        var mt2 = LimelightHelpers.getLimelightDoubleArrayEntry(VisionConstants.kLimelightTableName, "botpose_orb_wpiblue");
        mt1.set(new double[] {2, 3, 0, 0, 0, 30, 20, 2, 0, 2, 4}, 10_020_000L);
        mt2.set(new double[] {4, 5, 0, 0, 0, 90, 20, 2, 0, 2, 4}, 10_020_000L);
        hardware.readInputs(inputs);
        assertEquals(30, inputs.camera.megatagPoseEstimate.fieldToRobot().getRotation().getDegrees(), 1e-9);
        assertEquals(90, inputs.camera.megatag2PoseEstimate.fieldToRobot().getRotation().getDegrees(), 1e-9);
        assertEquals(10, inputs.camera.megatag2PoseEstimate.timestampSeconds(), 1e-9);
        assertEquals(0.02, inputs.camera.megatag2PoseEstimate.latency(), 1e-9);
        assertEquals(2, inputs.camera.megatag2AverageTagDistanceMeters, 1e-9);
        hardware.readInputs(inputs);
        assertEquals(10, inputs.camera.megatag2PoseEstimate.timestampSeconds(), 1e-9);
        mt2.set(new double[0]);
        hardware.readInputs(inputs);
        assertEquals(2, inputs.camera.megatagCount);
        assertEquals(0, inputs.camera.megatag2Count);
        assertNull(inputs.camera.megatag2PoseEstimate);
        assertTrue(Double.isNaN(inputs.camera.megatag2AverageTagDistanceMeters));
    }

    @Test
    void malformedPoseMetadataIsCleared() {
        var hardware = new VisionIOHardwareLimelight(state);
        var inputs = new VisionIO.VisionIOInputs();
        var mt2 = LimelightHelpers.getLimelightDoubleArrayEntry(VisionConstants.kLimelightTableName, "botpose_orb_wpiblue");
        mt2.set(new double[] {2, 3, 0, 0, 0, 0, 20, 2}, 10_020_000L);
        hardware.readInputs(inputs);
        assertNull(inputs.camera.megatag2PoseEstimate);
        mt2.set(new double[] {2, 3, 0, 0, 0, 0, 20, Double.NaN, 0, 2, 4}, 10_020_001L);
        hardware.readInputs(inputs);
        assertNull(inputs.camera.megatag2PoseEstimate);
    }

    @Test
    void poseStructSizeMatchesQualityField() {
        var original = observation(new Pose2d(2, 3, Rotation2d.kZero), 10);
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
    }

    private void setYawRate(double rate) {
        var speeds = new ChassisSpeeds(0, 0, rate);
        state.addDriveMotionMeasurements(10, 0, 0, rate, 0, 0, 0, 0, speeds, speeds, speeds, speeds, speeds);
    }

    private static MegatagPoseEstimate observation(Pose2d pose, double time) {
        return new MegatagPoseEstimate(pose, time, 0.02, 4, 1, new int[] {18, 19});
    }

    private static class FakeCamera implements VisionIO {
        Pose2d mt1Pose = new Pose2d(2, 3, Rotation2d.fromDegrees(30));
        Pose2d mt2Pose = new Pose2d(4, 5, Rotation2d.fromDegrees(90));
        double mt1Time = 10, mt2Time = 10, distance = 2, yaw, rate;
        int mt1Count = 2, mt2Count = 2;
        boolean sent, sentBeforeRead;

        @Override
        public void setRobotOrientation(double yaw, double rate) {
            this.yaw = yaw;
            this.rate = rate;
            sent = true;
        }

        @Override
        public void readInputs(VisionIOInputs inputs) {
            sentBeforeRead = sent;
            sent = false;
            inputs.camera.megatagCount = mt1Count;
            inputs.camera.megatag2Count = mt2Count;
            inputs.camera.megatagPoseEstimate = observation(mt1Pose, mt1Time);
            inputs.camera.megatag2PoseEstimate = observation(mt2Pose, mt2Time);
            inputs.camera.megatag2AverageTagDistanceMeters = distance;
        }
    }
}
