package com.team254.lib.pathplanner.path;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class PathPlannerCompatibilityTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void waypointFlipKeepsReefscapeDimensions() {
        Waypoint point = new Waypoint(null, new Translation2d(2.0, 3.0), null);
        Waypoint flipped = point.flip();
        assertEquals(Units.feetToMeters(57.573) - 2.0, flipped.anchor().getX(), 1e-9);
        assertEquals(Units.feetToMeters(26.417) - 3.0, flipped.anchor().getY(), 1e-9);
        assertEquals(point.anchor(), flipped.flip().anchor());
    }

    @Test
    void veryShortPathKeepsStartAndEndPoints() {
        PathPlannerPath path =
                new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(
                                new Pose2d(1.0, 1.0, Rotation2d.kZero),
                                new Pose2d(1.005, 1.0, Rotation2d.kZero)),
                        new PathConstraints(2.0, 2.0, 2.0, 2.0, 2.0, 2.0),
                        new IdealStartingState(0.0, Rotation2d.kZero),
                        new GoalEndState(0.0, Rotation2d.kZero));
        assertTrue(path.numPoints() >= 2);
        assertEquals(1.0, path.getPoint(0).position.getX(), 1e-9);
        assertEquals(1.005, path.getPoint(path.numPoints() - 1).position.getX(), 1e-9);
    }
}
