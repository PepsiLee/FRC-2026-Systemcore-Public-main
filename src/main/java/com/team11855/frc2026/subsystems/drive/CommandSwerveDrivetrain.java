package com.team11855.frc2026.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team11855.frc2026.Constants;
import com.team11855.frc2026.utils.simulations.MapleSimSwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;

/**
 * 本專案的底盤設定容器，保存 CAN bus、Pigeon 與四個模組參數，不建立馬達硬體。
 * CompTunerConstants.createDrivetrain() 用此介面把 Tuner 設定交給 RobotContainer；
 * RobotContainer 再建立 DriveIOHardware 或 DriveIOSim，最後交給 DriveSubsystem。
 */
public class CommandSwerveDrivetrain {
    private final SwerveDrivetrainConstants driveTrainConstants;
    private final SwerveModuleConstants<?, ?, ?>[] moduleConstants;

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        this.driveTrainConstants = driveTrainConstants;
        // Regulate module constants if in simulation mode
        if (Constants.useMapleSim) {
            this.moduleConstants =
                    MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules);
        } else {
            this.moduleConstants = modules;
        }
    }

    public SwerveDrivetrainConstants getDriveTrainConstants() {
        return driveTrainConstants;
    }

    public SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
        return moduleConstants;
    }

    /** 路徑模型直接讀取同一組模組中心座標，單位公尺，順序為左前、右前、左後、右後。 */
    public Translation2d[] getModuleLocations() {
        return Arrays.stream(moduleConstants)
                .map(module -> new Translation2d(module.LocationX, module.LocationY))
                .toArray(Translation2d[]::new);
    }
}
