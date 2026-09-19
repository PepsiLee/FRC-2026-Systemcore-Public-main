package com.team11855.frc2026.simulation;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DriveSimulationArenaTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void resettingArenaDoesNotSpawnGamePieces() {
        DriveSimulationArena arena = new DriveSimulationArena();
        arena.resetFieldForAuto();
        assertTrue(arena.gamePiecesOnField().isEmpty());
        assertTrue(arena.gamePieceLaunched().isEmpty());
    }
}
