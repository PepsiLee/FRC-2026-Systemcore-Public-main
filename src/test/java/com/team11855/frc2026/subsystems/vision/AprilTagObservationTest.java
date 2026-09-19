package com.team11855.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class AprilTagObservationTest {
    @Test
    void limelightCameraAxesConvertRightDownForwardToForwardLeftUp() {
        var target = AprilTagObservation.fromLimelight(18, new double[] {0.3, -0.2, 2, 0, 0, 0}, 10)
                .orElseThrow();
        assertEquals(18, target.tagId());
        assertEquals(new Translation3d(2, -0.3, 0.2), target.cameraToTag());
        assertEquals(10, target.timestampSeconds());
    }

    @Test
    void rejectsMissingMalformedOrNonAprilTagMeasurements() {
        double[] valid = {0, 0, 2, 0, 0, 0};
        for (double id : new double[] {-1, 0, 1.5, Double.NaN, Double.POSITIVE_INFINITY}) {
            assertTrue(AprilTagObservation.fromLimelight(id, valid, 10).isEmpty());
        }
        for (double time : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            assertTrue(AprilTagObservation.fromLimelight(18, valid, time).isEmpty());
        }
        assertTrue(AprilTagObservation.fromLimelight(18, null, 10).isEmpty());
        assertTrue(AprilTagObservation.fromLimelight(18, new double[0], 10).isEmpty());
        assertTrue(AprilTagObservation.fromLimelight(18, new double[6], 10).isEmpty());
        assertTrue(AprilTagObservation.fromLimelight(18, new double[] {0, 0, -1, 0, 0, 0}, 10).isEmpty());
        assertTrue(AprilTagObservation.fromLimelight(18, new double[] {Double.NaN, 0, 2, 0, 0, 0}, 10).isEmpty());
    }

    @Test
    void robotCenterTransformIncludesMountTranslationAndRotation() {
        var target = new AprilTagObservation(18, new Translation3d(2, 0, 0), 10);
        var mount = new Transform3d(new Translation3d(0.3, -0.2, 0.54),
                new Rotation3d(0, 0, Math.PI / 2));
        var centerToTag = target.robotToTag(mount);
        assertEquals(0.3, centerToTag.getX(), 1e-9);
        assertEquals(1.8, centerToTag.getY(), 1e-9);
        assertEquals(0.54, centerToTag.getZ(), 1e-9);
    }
}
