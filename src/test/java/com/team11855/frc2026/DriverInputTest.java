package com.team11855.frc2026;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.PS5ControllerSim;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DriverInputTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void ps5AxesPreserveTranslationAndTurnDirections() {
        CommandPS5Controller controller = new CommandPS5Controller(0);
        PS5ControllerSim joystick = new PS5ControllerSim(controller.getHID());
        joystick.setLeftY(-0.25);
        joystick.setLeftX(0.25);
        joystick.setRightX(0.5);
        joystick.notifyNewData();

        assertEquals(0.125, RobotContainer.translationInput(controller.getLeftY()), 1e-9);
        assertEquals(-0.125, RobotContainer.translationInput(controller.getLeftX()), 1e-9);
        assertEquals(-0.25, RobotContainer.rotationInput(controller.getRightX()), 1e-9);

        joystick.setLeftY(0.0);
        joystick.setLeftX(0.0);
        joystick.setRightX(0.0);
        joystick.notifyNewData();
        assertEquals(0.0, RobotContainer.translationInput(controller.getLeftY()), 1e-9);
        assertEquals(0.0, RobotContainer.rotationInput(controller.getRightX()), 1e-9);
    }

    @Test
    void fullDeflectionPreservesOriginalSpeedScaling() {
        assertEquals(1.0, RobotContainer.translationInput(-1.0));
        assertEquals(-1.0, RobotContainer.translationInput(1.0));
        assertEquals(1.0, RobotContainer.rotationInput(-1.0));
        assertEquals(-1.0, RobotContainer.rotationInput(1.0));
    }
}
