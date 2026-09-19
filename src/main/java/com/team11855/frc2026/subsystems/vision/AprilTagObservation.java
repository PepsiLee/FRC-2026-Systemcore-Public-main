package com.team11855.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;

/**
 * One identified tag relative to the camera. Camera coordinates use WPILib forward/left/up axes;
 * timestampSeconds is capture time in FPGA seconds, not CTRE time or the time the getter was called.
 */
public record AprilTagObservation(
        int tagId, Translation3d cameraToTag, double timestampSeconds) {

    /** Converts Limelight camera-space right/down/forward coordinates to forward/left/up. */
    public static Optional<AprilTagObservation> fromLimelight(
            double tagId, double[] targetPose, double timestampSeconds) {
        if (!Double.isFinite(tagId)
                || tagId < 1.0
                || tagId > Integer.MAX_VALUE
                || tagId != Math.rint(tagId)
                || targetPose == null
                || targetPose.length < 6
                || !Double.isFinite(timestampSeconds)
                || timestampSeconds <= 0.0) {
            return Optional.empty();
        }
        for (int i = 0; i < 3; i++) {
            if (!Double.isFinite(targetPose[i])) return Optional.empty();
        }
        if (targetPose[2] <= 0.0) return Optional.empty();
        return Optional.of(
                new AprilTagObservation(
                        (int) tagId,
                        new Translation3d(targetPose[2], -targetPose[0], -targetPose[1]),
                        timestampSeconds));
    }

    /** Tag position relative to the drivetrain center, including the camera mounting transform. */
    public Translation3d robotToTag(Transform3d robotToCamera) {
        return cameraToTag
                .rotateBy(robotToCamera.getRotation())
                .plus(robotToCamera.getTranslation());
    }
}
