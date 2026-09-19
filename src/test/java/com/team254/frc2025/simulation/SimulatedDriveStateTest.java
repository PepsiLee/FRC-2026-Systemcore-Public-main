package com.team254.frc2025.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class SimulatedDriveStateTest {
    @Test
    void hasNoTruthPoseUntilTheDrivePublishesOne() {
        var state = new SimulatedDriveState();

        assertNull(state.getLatestFieldToRobot());
        assertTrue(state.getFieldToRobot(0.0).isEmpty());
    }

    @Test
    void sharesLatestPoseAndInterpolatesHistoryWithoutAContainer() {
        var state = new SimulatedDriveState();
        var start = new Pose2d(1.0, 2.0, Rotation2d.kZero);
        var end = new Pose2d(3.0, 4.0, Rotation2d.kZero);

        state.addFieldToRobot(10.0, start);
        state.addFieldToRobot(10.5, end);

        assertEquals(end, state.getLatestFieldToRobot());
        assertEquals(
                new Pose2d(2.0, 3.0, Rotation2d.kZero),
                state.getFieldToRobot(10.25).orElseThrow());
    }

    @Test
    void expiresOldTruthInsteadOfBlendingItIntoNewSamples() {
        var state = new SimulatedDriveState();
        var expired = new Pose2d(1.0, 2.0, Rotation2d.kZero);
        var current = new Pose2d(5.0, 6.0, Rotation2d.kZero);

        state.addFieldToRobot(10.0, expired);
        state.addFieldToRobot(12.0, current);

        assertEquals(current, state.getLatestFieldToRobot());
        assertEquals(current, state.getFieldToRobot(10.0).orElseThrow());
    }
}
