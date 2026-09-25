package com.team11855.frc2026;

import com.team11855.frc2026.subsystems.drive.CommandSwerveDrivetrain;
import com.team11855.frc2026.subsystems.drive.CompTunerConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.Arrays;

public class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    // 與底盤使用相同的 CANivore 名稱，讓 CAN 診斷也讀取同一條 bus。
    public static final String kDriveCanBusName = CompTunerConstants.kCANBusName;
    public static boolean kIsReplay = false;

    public static final double kSteerJoystickDeadband = 0.05;
    // 使用者同意：下列整車尺寸、重量、慣量先沿用原預設；尚未依新底盤實測。
    public static final double kRobotWidth = Units.inchesToMeters(35.625);
    public static final double kRobotDiagonal = Math.sqrt(2.0) * kRobotWidth;
    public static final double kRobotMassKg = Units.lbsToKilograms(147.92);
    public static final double kRobotMomentOfInertia = 2 * 9.38; // kg * m^2
    public static final double kCOGHeightMeters = Units.inchesToMeters(0.0);

    public static final class DriveConstants {
        // PS5 平移速度上限，單位 m/s；獨立於 Tuner 的 kSpeedAt12Volts 馬達模型。
        public static final double kDriveMaxSpeed = 5.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 10.0;
        public static final double kMaxXAccelerationMetersPerSecondSquared = 10.0;
        public static final double kMaxYAccelerationMetersPerSecondSquared = 10.0;
        // Tuner 原值 0.4 圈/秒；1 圈 = 2π 弧度，換算為 0.4 × 2π ≈ 2.513 rad/s。
        // 此值供底盤 request 使用；rad/s 是弧度/秒，不是角度/秒。
        public static final double kDriveMaxAngularRate = 0.4 * 2.0 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 20.0;
        public static final double kHeadingControllerP = 5.0;
        public static final double kHeadingControllerI = 0;
        public static final double kHeadingControllerD = 0;
        // 實機與模擬都由使用者的 Tuner 設定建立；模擬專用調整只作用於新建的模組。
        public static final CommandSwerveDrivetrain kDrivetrain =
                CompTunerConstants.createDrivetrain();
        // 使用者同意：模擬重量與保險桿外尺寸先沿用原預設，不能用模組中心間距代替。
        public static final double kRobotWeightPounds = 150.0;
        public static final double kBumperLengthInches = 35.625;
        public static final double kBumperWidthInches = 35.625;
        public static final double kWheelCoefficientOfFriction = 1.0;
        public static final int kDriveMotorCount = 1;

        public static final double kDisabledDriveXStdDev = 1.0;
        public static final double kDisabledDriveYStdDev = 1.0;
        public static final double kDisabledDriveRotStdDev = 1.0;

        public static final double kEnabledDriveXStdDev = 0.3;
        public static final double kEnabledDriveYStdDev = 0.3;
        public static final double kEnabledDriveRotStdDev = 0.2;

        public static final double kDrivePitchThresholdRadians = Units.degreesToRadians(10.0);
        public static final double kDriveRollThresholdRadians = Units.degreesToRadians(10.0);
    }

    /** PS5 controller index in the Driver Station USB tab. */
    public static final int kDriverControllerPort = 0;

    public static final boolean useMapleSim = true;

    // April Tag Layout
    public static final AprilTagFieldLayout kAprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final int[] kAllowedTagIDs = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};
    public static final AprilTagFieldLayout kAprilTagLayoutReefsOnly =
            new AprilTagFieldLayout(
                    kAprilTagLayout.getTags().stream()
                            .filter(
                                    tag ->
                                            Arrays.stream(kAllowedTagIDs)
                                                    .anyMatch(element -> element == tag.ID))
                            .toList(),
                    kAprilTagLayout.getFieldLength(),
                    kAprilTagLayout.getFieldWidth());

    public static final double kFieldWidthMeters = kAprilTagLayout.getFieldWidth();
    public static final double kFieldLengthMeters = kAprilTagLayout.getFieldLength();

    // Limelight constants
    public static final class VisionConstants {

        // Large variance used to downweight unreliable vision measurements
        public static final double kLargeVariance = 1e6;

        // Standard deviation constants
        public static final int kMegatag1XStdDevIndex = 0;
        public static final int kMegatag1YStdDevIndex = 1;
        public static final int kMegatag1YawStdDevIndex = 5;

        // Validation constants
        public static final int kExpectedStdDevArrayLength = 12;

        public static final int kMinFiducialCount = 1;

        // 單顆 Limelight：名稱必須與裝置的 NetworkTables 名稱完全相同。
        public static final String kLimelightTableName = "limelight-rear";

        // ===== 鏡頭安裝位置：更換安裝位置時只修改這一區 =====
        // 使用者確認：正中間、鏡頭高 54 cm、水平安裝、朝車頭。
        // 原點是底盤定位中心投影到地面的點；請量到相機鏡頭中心。
        // 距離直接填「公尺」：例如前方 30 cm 填 0.30，不需另外換成英吋。
        public static final double kCameraForwardMeters = 0.0; // 前方為正、後方為負。
        public static final double kCameraRightMeters = 0.0; // Limelight：右方為正、左方為負。
        public static final double kCameraHeightMeters = 0.54; // 鏡頭中心離地高度，向上為正。
        public static final double kCameraPitchDegrees = 0.0; // 俯仰角（度）；抬頭為正、低頭為負。
        public static final double kCameraYawDegrees = 0.0; // 水平朝向（度）；0 朝前、180 朝後。
        // 名稱含 rear 不會自動把鏡頭轉向後方；實際朝後時，請把上面的 yaw 改成 180。
        // 本設定沿用沒有左右側傾的安裝方式，roll 固定為 0 度。

        // 追蹤控制與 PhotonVision 模擬共用 WPILib 前／左／上座標；由上方唯一一組設定自動換算。
        // Limelight 的 right 要反號成 left；抬頭角要反號成 WPILib 的 pitch。
        // 這是推導值，不要另外手動修改，避免實機與模擬的安裝位置不一致。
        public static final Transform3d kRobotToCamera =
                new Transform3d(
                        new Translation3d(
                                kCameraForwardMeters, -kCameraRightMeters, kCameraHeightMeters),
                        new Rotation3d(
                                0.0,
                                -Units.degreesToRadians(kCameraPitchDegrees),
                                Units.degreesToRadians(kCameraYawDegrees)));

        // Team 254 MT1 主分支門檻；單 Tag 失敗時另試歷史航向備援。
        public static final double kDefaultAmbiguityThreshold = 0.19;
        public static final double kDefaultYawDiffThreshold = 5.0; // 度。
        public static final double kTagAreaThresholdForYawCheck = 2.0; // 影像面積百分比。
        public static final double kTagMinAreaForSingleTagMegatag = 1.0;
        public static final double kDefaultZThreshold = 0.2; // 公尺。
        public static final double kDefaultNormThreshold = 1.0; // 距離場地原點，非標籤距離。
        public static final double kHighYawLookbackSeconds = 0.3;
        public static final double kMaxVisionYawRateRadiansPerSecond = 5.0; // 僅限 gyro 備援。

        // 保留本機資料逾時保護；時間仍為 FPGA 秒，僅 DriveIO 轉換 CTRE 時基。
        public static final double kMaxMeasurementAgeSeconds = 0.50;
        public static final double kFutureTimestampToleranceSeconds = 0.05;

        public static final double kCameraHorizontalFOVDegrees = 81.0;
        public static final double kCameraVerticalFOVDegrees = 55.0;
        public static final int kCameraImageWidth = 1280;
        public static final int kCameraImageHeight = 800;
    }

    /** AprilTag 對準／跟隨專用參數；不改手動駕駛、路徑或朝向維持的 PID。 */
    public static final class AprilTagTrackingConstants {
        public static final int kTargetTagId = 21; // □ 與 △ 固定瞄準的 AprilTag 編號。
        public static final double kTargetDistanceMeters = 1.0; // 底盤中心至標籤的水平距離。
        public static final double kDistanceToleranceMeters = 0.05; // ±5 cm 內停止前後移動。
        public static final double kHeadingToleranceRadians = Units.degreesToRadians(2.0);
        public static final double kHeadingP = 3.0; // rad 誤差 → rad/s。
        public static final double kDistanceP = 1.0; // m 誤差 → m/s。
        public static final double kMaxLinearSpeedMetersPerSecond = 0.6;
        public static final double kMaxAngularSpeedRadiansPerSecond = 1.5; // 弧度/秒。
        // 偏離車頭超過 15° 先轉向，不前後移動，避免側邊目標造成錯誤接近。
        public static final double kMaxHeadingErrorForTranslationRadians =
                Units.degreesToRadians(15.0);
        public static final double kAimSettleSeconds = 0.15; // □ 連續對準才結束。
        public static final double kAimTimeoutSeconds = 3.0; // □ 最長轉向時間。
        public static final double kMaxObservationAgeSeconds = 0.25;
        public static final double kFutureTimestampToleranceSeconds = 0.05;
        public static final double kMinTargetDistanceMeters = 0.10;
        public static final double kMaxTargetDistanceMeters = 6.0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kDriveMaxSpeed;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.74;
        // 與駕駛設定共用換算後的角速度上限，單位 rad/s。
        public static final double kMaxAngularSpeedRadiansPerSecond =
                DriveConstants.kDriveMaxAngularRate;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 31.538;

        public static final double kPXYController = 5.0;
        // 使用者 Tuner 檔的路徑平移 P=1.0，同時用於沿路徑與橫向誤差控制。
        public static final double kPLTEController = 1.0;
        public static final double kPCTEController = 1.0;
        // 路徑旋轉 P=1.0；與定點對位、PS5 朝向維持的 PID 分開，避免一起被改動。
        public static final double kPathRotationControllerP = 1.0;
        public static final double kPThetaController = 5.0;

        public static final double kTranslationKa = 0.0;
        public static final double kMaxEndPathVelocity = 2.0; // m/s

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
