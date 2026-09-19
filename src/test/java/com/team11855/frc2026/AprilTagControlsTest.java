package com.team11855.frc2026;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.PS5ControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AprilTagControlsTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private SubsystemBase drive;
    private PS5ControllerSim joystick;
    private ProbeCommand aim;
    private ProbeCommand follow;
    private boolean enabled;

    @BeforeAll
    static void initializeHal() { assertTrue(HAL.initialize(500, 0)); }

    @BeforeEach
    void setUp() {
        scheduler.cancelAll();
        scheduler.getDefaultButtonLoop().clear();
        scheduler.setActiveButtonLoop(scheduler.getDefaultButtonLoop());
        scheduler.enable();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        var controller = new CommandPS5Controller(0);
        joystick = new PS5ControllerSim(controller.getHID());
        joystick.setSquareButton(false);
        joystick.setTriangleButton(false);
        joystick.setOptionsButton(false);
        joystick.notifyNewData();
        drive = new SubsystemBase() {};
        aim = new ProbeCommand(drive);
        follow = new ProbeCommand(drive);
        enabled = true;
        RobotContainer.bindAprilTagControls(controller,
                () -> enabled && !controller.getHID().getOptionsButton(), aim, follow);
        tick();
    }

    private void tick() {
        joystick.notifyNewData();
        scheduler.run();
    }

    @AfterEach
    void tearDown() {
        scheduler.cancelAll();
        scheduler.getDefaultButtonLoop().clear();
        scheduler.unregisterSubsystem(drive);
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test
    void squareRunsOnceWithoutHoldingAndDoesNotRepeatWhileHeld() {
        joystick.setSquareButton(true);
        tick();
        assertTrue(scheduler.isScheduled(aim));
        aim.done = true;
        tick();
        tick();
        assertFalse(scheduler.isScheduled(aim));
        assertEquals(1, aim.starts);
        joystick.setSquareButton(false);
        tick();
        aim.done = false;
        joystick.setSquareButton(true);
        tick();
        assertEquals(2, aim.starts);
        joystick.setSquareButton(false);
        tick();
        assertTrue(scheduler.isScheduled(aim));
    }

    @Test
    void triangleWinsSimultaneousPressAndSquareCannotInterruptFollow() {
        joystick.setSquareButton(true);
        joystick.setTriangleButton(true);
        tick();
        assertTrue(scheduler.isScheduled(follow));
        assertEquals(0, aim.starts);
        joystick.setSquareButton(false);
        tick();
        joystick.setSquareButton(true);
        tick();
        assertTrue(scheduler.isScheduled(follow));
        assertEquals(0, aim.starts);
        joystick.setTriangleButton(false);
        tick();
        assertFalse(scheduler.isScheduled(follow));
        assertEquals(1, follow.interruptions);
        assertEquals(0, aim.starts); // Releasing △ while □ is held must not start aiming.
    }

    @Test
    void triangleInterruptsExistingAimAndReleaseEndsFollow() {
        joystick.setSquareButton(true);
        tick();
        assertTrue(scheduler.isScheduled(aim));
        joystick.setTriangleButton(true);
        tick();
        assertFalse(scheduler.isScheduled(aim));
        assertEquals(1, aim.interruptions);
        assertTrue(scheduler.isScheduled(follow));
        joystick.setTriangleButton(false);
        tick();
        assertFalse(scheduler.isScheduled(follow));
    }

    @Test
    void gateAndOptionsPreventStartAndCancelHeldFollow() {
        enabled = false;
        joystick.setSquareButton(true);
        joystick.setTriangleButton(true);
        tick();
        assertEquals(0, aim.starts);
        assertEquals(0, follow.starts);
        enabled = true;
        tick();
        assertTrue(scheduler.isScheduled(follow));
        joystick.setOptionsButton(true);
        tick();
        assertFalse(scheduler.isScheduled(follow));
        joystick.setOptionsButton(false);
        tick();
        assertEquals(2, follow.starts); // Held △ becomes active again when the gate opens.
        enabled = false;
        tick();
        assertFalse(scheduler.isScheduled(follow));
    }

    private static class ProbeCommand extends Command {
        int starts;
        int interruptions;
        boolean done;

        ProbeCommand(SubsystemBase drive) { addRequirements(drive); }

        @Override
        public void initialize() { starts++; }

        @Override
        public boolean isFinished() { return done; }

        @Override
        public void end(boolean interrupted) { if (interrupted) interruptions++; }
    }
}
