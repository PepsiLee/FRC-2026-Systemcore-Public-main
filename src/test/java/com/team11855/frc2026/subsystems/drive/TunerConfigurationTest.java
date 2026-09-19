package com.team11855.frc2026.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class TunerConfigurationTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void simulationAndPlannerUseTheExportedGeometryInModuleOrder() {
        CommandSwerveDrivetrain configuration = CompTunerConstants.createDrivetrain();
        var modules = configuration.getModuleConstants();
        Translation2d[] plannerLocations = configuration.getModuleLocations();
        var exported =
                new SwerveModuleConstants<?, ?, ?>[] {
                    CompTunerConstants.FrontLeft,
                    CompTunerConstants.FrontRight,
                    CompTunerConstants.BackLeft,
                    CompTunerConstants.BackRight
                };
        double halfSpacing = Units.inchesToMeters(11);
        Translation2d[] expected = {
            new Translation2d(halfSpacing, halfSpacing),
            new Translation2d(halfSpacing, -halfSpacing),
            new Translation2d(-halfSpacing, halfSpacing),
            new Translation2d(-halfSpacing, -halfSpacing)
        };
        for (int i = 0; i < 4; i++) {
            assertEquals(expected[i], plannerLocations[i]);
            assertEquals(exported[i].LocationX, modules[i].LocationX);
            assertEquals(exported[i].LocationY, modules[i].LocationY);
            assertEquals(exported[i].DriveMotorGearRatio, modules[i].DriveMotorGearRatio);
            assertEquals(exported[i].SteerMotorGearRatio, modules[i].SteerMotorGearRatio);
            assertEquals(exported[i].WheelRadius, modules[i].WheelRadius);
        }
    }

    @Test
    void buildingSimulationDoesNotOverwriteHardwareCalibrationOrPid() {
        double hardwareOffset = CompTunerConstants.FrontLeft.EncoderOffset;
        double hardwareP = CompTunerConstants.FrontLeft.SteerMotorGains.kP;
        double hardwareD = CompTunerConstants.FrontLeft.SteerMotorGains.kD;
        boolean hardwareRightInverted = CompTunerConstants.FrontRight.DriveMotorInverted;
        var first = CompTunerConstants.createDrivetrain().getModuleConstants();
        var second = CompTunerConstants.createDrivetrain().getModuleConstants();

        assertNotSame(CompTunerConstants.FrontLeft, first[0]);
        assertNotSame(first[0], second[0]);
        assertEquals(hardwareOffset, CompTunerConstants.FrontLeft.EncoderOffset);
        assertEquals(hardwareP, CompTunerConstants.FrontLeft.SteerMotorGains.kP);
        assertEquals(hardwareD, CompTunerConstants.FrontLeft.SteerMotorGains.kD);
        assertEquals(hardwareRightInverted, CompTunerConstants.FrontRight.DriveMotorInverted);
        assertEquals(0.0, first[0].EncoderOffset);
        assertFalse(first[1].DriveMotorInverted);

        first[0].SteerMotorGains.kP = 123.0;
        assertEquals(hardwareP, CompTunerConstants.FrontLeft.SteerMotorGains.kP);
        assertNotEquals(123.0, second[0].SteerMotorGains.kP);
    }
}
