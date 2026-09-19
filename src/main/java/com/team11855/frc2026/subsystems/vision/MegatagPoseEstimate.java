package com.team11855.frc2026.subsystems.vision;

import com.team11855.lib.limelight.LimelightHelpers;
import com.team11855.lib.util.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Objects;

/**
 * Represents a robot pose estimate using multiple AprilTags (Megatag).
 *
 * @param fieldToRobot The estimated robot pose on the field
 * @param timestampSeconds The timestamp when this estimate was captured
 * @param latency Processing latency in seconds
 * @param avgTagArea Average area of detected tags
 * @param quality Quality score of the pose estimate (0-1)
 * @param fiducialIds IDs of fiducials used for this estimate
 */
public record MegatagPoseEstimate(
        Pose2d fieldToRobot,
        double timestampSeconds,
        double latency,
        double avgTagArea,
        double quality,
        int[] fiducialIds)
        implements StructSerializable {

    public MegatagPoseEstimate {
        if (fieldToRobot == null) {
            fieldToRobot = MathHelpers.kPose2dZero;
        }
        if (fiducialIds == null) {
            fiducialIds = new int[0];
        }
    }

    /** Converts a Limelight pose estimate to a MegatagPoseEstimate. */
    public static MegatagPoseEstimate fromLimelight(LimelightHelpers.PoseEstimate poseEstimate) {
        Pose2d fieldToRobot = poseEstimate.pose;
        if (fieldToRobot == null) {
            fieldToRobot = MathHelpers.kPose2dZero;
        }
        var rawFiducials = poseEstimate.rawFiducials == null
                ? new LimelightHelpers.RawFiducial[0] : poseEstimate.rawFiducials;
        var validFiducials = Arrays.stream(rawFiducials).filter(Objects::nonNull).toArray(
                LimelightHelpers.RawFiducial[]::new);
        int[] fiducialIds = Arrays.stream(validFiducials).mapToInt(tag -> tag.id).toArray();
        double quality = validFiducials.length == 0 ? 0.0
                : validFiducials.length > 1 ? 1.0 : 1.0 - validFiducials[0].ambiguity;
        return new MegatagPoseEstimate(
                fieldToRobot,
                poseEstimate.timestampSeconds,
                poseEstimate.latency / 1000.0, // Limelight 毫秒 → 本介面的秒。
                poseEstimate.avgTagArea,
                quality,
                fiducialIds);
    }

    public static final MegatagPoseEstimateStruct struct = new MegatagPoseEstimateStruct();

    public static class MegatagPoseEstimateStruct implements Struct<MegatagPoseEstimate> {

        @Override
        public Class<MegatagPoseEstimate> getTypeClass() {
            return MegatagPoseEstimate.class;
        }

        @Override
        public String getTypeString() {
            return "record:MegatagPoseEstimate";
        }

        @Override
        public int getSize() {
            return Pose2d.struct.getSize() + 4 * Double.BYTES;
        }

        @Override
        public String getSchema() {
            return "Pose2d fieldToRobot; double timestampSeconds; double latency; double avgTagArea; double quality";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Pose2d.struct};
        }

        @Override
        public MegatagPoseEstimate unpack(ByteBuffer bb) {
            Pose2d fieldToRobot = Pose2d.struct.unpack(bb);
            double timestampSeconds = bb.getDouble();
            double latency = bb.getDouble();
            double avgTagArea = bb.getDouble();
            double quality = bb.getDouble();
            int[] fiducialIds = new int[0];
            return new MegatagPoseEstimate(
                    fieldToRobot, timestampSeconds, latency, avgTagArea, quality, fiducialIds);
        }

        @Override
        public void pack(ByteBuffer bb, MegatagPoseEstimate value) {
            Pose2d.struct.pack(bb, value.fieldToRobot());
            bb.putDouble(value.timestampSeconds());
            bb.putDouble(value.latency());
            bb.putDouble(value.avgTagArea());
            bb.putDouble(value.quality());
        }

        @Override
        public String getTypeName() {
            return "MegatagPoseEstimate";
        }
    }
}
