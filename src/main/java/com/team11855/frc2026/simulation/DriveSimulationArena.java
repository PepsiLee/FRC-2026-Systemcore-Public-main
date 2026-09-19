package com.team11855.frc2026.simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape.ReefscapeFieldObstacleMap;

/** Keeps the 2025 field obstacles without spawning game pieces or scoring simulations. */
public final class DriveSimulationArena extends SimulatedArena {
    public DriveSimulationArena() {
        super(new ReefscapeFieldObstacleMap());
        disableBreakdownPublishing();
    }

    @Override
    public void placeGamePiecesOnField() {
        // This branch simulates only the drivetrain and cameras, including after a field reset.
    }
}
