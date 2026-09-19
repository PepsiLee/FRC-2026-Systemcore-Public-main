package com.team254.frc2025.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team254.frc2025.Constants;
import com.team254.frc2025.RobotState;
import com.team254.frc2025.simulation.DriveSimulationArena;
import com.team254.frc2025.simulation.SimulatedDriveState;
import com.team254.frc2025.utils.simulations.MapleSimSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Consumer;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code DriveIOSim} class extends {@link DriveIOHardware} to provide simulation-specific
 * functionality for the swerve drive system. It integrates with WPILib's simulation framework for
 * testing and development.
 */
public class DriveIOSim extends DriveIOHardware {

    private SimulatedDriveState simulatedDriveState = null;
    // 使用 Tuner 指定的 4 ms（250 Hz），此 Notifier 是唯一的物理更新入口。
    private static final double kSimLoopPeriod =
            CompTunerConstants.kSimulationLoopPeriod.in(Units.Seconds);
    private final SwerveModuleConstants<?, ?, ?>[] simulationModules;
    private Notifier simNotifier = null;
    private double lastSimTime;
    public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    Consumer<SwerveDriveState> simTelemetryConsumer =
            swerveDriveState -> {
                // Protect at init
                if (simulatedDriveState == null) {
                    return;
                }

                if (Constants.useMapleSim && mapleSimSwerveDrivetrain != null) {
                    swerveDriveState.Pose =
                            mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
                }
                simulatedDriveState.addFieldToRobot(swerveDriveState.Pose);
                telemetryConsumer_.accept(swerveDriveState);
            };

    public DriveIOSim(
            RobotState robotState,
            SimulatedDriveState simulatedDriveState,
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(robotState, driveTrainConstants, modules);
        this.simulatedDriveState = simulatedDriveState;
        // 與 CTRE 模擬共用本次傳入的模組設定，不再另外讀取另一台機器人的參數。
        this.simulationModules = modules.clone();

        // Rewrite the telemetry consumer with a consumer for sim
        registerTelemetry(simTelemetryConsumer);
        startSimThread();
    }

    public void startSimThread() {
        if (Constants.useMapleSim) {
            // Select the matching field before constructing or registering any simulated drive.
            SimulatedArena.overrideInstance(new DriveSimulationArena());
            mapleSimSwerveDrivetrain =
                    new MapleSimSwerveDrivetrain(
                            Units.Seconds.of(kSimLoopPeriod),
                            Units.Pounds.of(Constants.DriveConstants.kRobotWeightPounds),
                            Units.Inches.of(Constants.DriveConstants.kBumperLengthInches),
                            Units.Inches.of(Constants.DriveConstants.kBumperWidthInches),
                            // 使用者確認：MK5i R2 的行走與轉向馬達皆為 Kraken X60。
                            DCMotor.getKrakenX60(Constants.DriveConstants.kDriveMotorCount),
                            DCMotor.getKrakenX60(Constants.DriveConstants.kDriveMotorCount),
                            1.2,
                            getModuleLocations(),
                            getPigeon2(),
                            getModules(),
                            simulationModules);
            simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        } else {
            lastSimTime = Utils.getCurrentTimeSeconds();
            simNotifier =
                    new Notifier(
                            () -> {
                                final double currentTime = Utils.getCurrentTimeSeconds();
                                double deltaTime = currentTime - lastSimTime;
                                lastSimTime = currentTime;
                                updateSimState(deltaTime, RobotController.getBatteryVoltage());
                            });
        }
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void resetOdometry(Pose2d pose) {
        if (Constants.useMapleSim && mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.05);
        }
        super.resetOdometry(pose);
    }

    @Override
    public void readInputs(DriveIOInputs inputs) {
        super.readInputs(inputs);

        // Handle the viz
        var pose = simulatedDriveState.getLatestFieldToRobot();
        if (pose != null) {
            Logger.recordOutput("Drive/Viz/SimPose", pose);
        }
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        return mapleSimSwerveDrivetrain;
    }
}
