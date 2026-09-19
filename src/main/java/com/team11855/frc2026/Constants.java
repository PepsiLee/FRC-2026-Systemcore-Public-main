package com.team11855.frc2026;

import com.team11855.frc2026.subsystems.drive.CommandSwerveDrivetrain;
import com.team11855.frc2026.subsystems.drive.CompTunerConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

        // Standard deviation array indices for Megatag2
        public static final int kMegatag2XStdDevIndex = 6;
        public static final int kMegatag2YStdDevIndex = 7;
        public static final int kMegatag2YawStdDevIndex = 11;

        // Validation constants
        public static final int kExpectedStdDevArrayLength = 12;

        public static final int kMinFiducialCount = 1;

        // Camera A (Left-side Camera)
        public static final double kCameraAPitchDegrees = 20.0;
        public static final double kCameraAPitchRads = Units.degreesToRadians(kCameraAPitchDegrees);
        public static final double kCameraAHeightOffGroundMeters = Units.inchesToMeters(8.3787);
        public static final String kLimelightATableName = "limelight-left";
        public static final double kRobotToCameraAForward = Units.inchesToMeters(7.8757);
        public static final double kRobotToCameraASide = Units.inchesToMeters(-11.9269);
        public static final Rotation2d kCameraAYawOffset = Rotation2d.fromDegrees(0.0);
        public static final Transform2d kRobotToCameraA =
                new Transform2d(
                        new Translation2d(kRobotToCameraAForward, kRobotToCameraASide),
                        kCameraAYawOffset);

        // Camera B (Right-side camera)
        public static final double kCameraBPitchDegrees = 20.0;
        public static final double kCameraBPitchRads = Units.degreesToRadians(kCameraBPitchDegrees);
        public static final double kCameraBHeightOffGroundMeters = Units.inchesToMeters(8.3787);
        public static final String kLimelightBTableName = "limelight-right";
        public static final double kRobotToCameraBForward = Units.inchesToMeters(7.8757);
        public static final double kRobotToCameraBSide = Units.inchesToMeters(11.9269);
        public static final Rotation2d kCameraBYawOffset = Rotation2d.fromDegrees(0.0);
        public static final Transform2d kRobotToCameraB =
                new Transform2d(
                        new Translation2d(kRobotToCameraBForward, kRobotToCameraBSide),
                        kCameraBYawOffset);

        // Vision processing constants
        public static final double kDefaultAmbiguityThreshold = 0.19;
        public static final double kDefaultYawDiffThreshold = 5.0;
        public static final double kTagAreaThresholdForYawCheck = 2.0;
        public static final double kTagMinAreaForSingleTagMegatag = 1.0;
        public static final double kDefaultZThreshold = 0.2;
        public static final double kDefaultNormThreshold = 1.0;
        public static final double kMinAmbiguityToFlip = 0.08;

        public static final double kCameraHorizontalFOVDegrees = 81.0;
        public static final double kCameraVerticalFOVDegrees = 55.0;
        public static final int kCameraImageWidth = 1280;
        public static final int kCameraImageHeight = 800;

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
