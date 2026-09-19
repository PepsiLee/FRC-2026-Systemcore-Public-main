package com.team11855.frc2026.subsystems.vision;

import com.team11855.frc2026.Constants.AprilTagTrackingConstants;
import com.team11855.frc2026.Constants.VisionConstants;
import com.team11855.frc2026.RobotState;
import com.team11855.lib.limelight.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Optional;

/** Hardware implementation of VisionIO using a single Limelight camera. */
public class VisionIOHardwareLimelight implements VisionIO {
    protected final NetworkTable table =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
    protected final RobotState robotState;

    private static final double[] DEFAULT_STDDEVS =
            new double[VisionConstants.kExpectedStdDevArrayLength];

    /** Creates a new Limelight vision IO instance. */
    public VisionIOHardwareLimelight(RobotState robotState) {
        this.robotState = robotState;
        setLLSettings();
    }

    @Override
    public void setRobotOrientation(double yawDegrees, double yawRateDegreesPerSecond) {
        LimelightHelpers.SetRobotOrientation(
                VisionConstants.kLimelightTableName, yawDegrees, yawRateDegreesPerSecond,
                0.0, 0.0, 0.0, 0.0);
    }

    /** Publishes the shared mounting configuration to the only Limelight. */
    private void setLLSettings() {
        // 啟動時由程式設定安裝位置；請修改 Constants.VisionConstants，而非只改網頁 UI。
        double[] cameraPose = {
            VisionConstants.kCameraForwardMeters,
            VisionConstants.kCameraRightMeters,
            VisionConstants.kCameraHeightMeters,
            0.0,
            VisionConstants.kCameraPitchDegrees,
            VisionConstants.kCameraYawDegrees
        };
        table.getEntry("camerapose_robotspace_set").setDoubleArray(cameraPose);
        // 使用底盤提供的外部朝向，不依賴 Limelight 型號是否內建 IMU。
        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightTableName, 0);
        // 優先選取 21 號供局部追蹤；不要用 fiducial_id_filters_set 限制場地定位。
        LimelightHelpers.setPriorityTagID(
                VisionConstants.kLimelightTableName, AprilTagTrackingConstants.kTargetTagId);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        readCameraData(table, inputs.camera, VisionConstants.kLimelightTableName);
    }

    /** Reads relative target data even when the field-pose estimator rejects a measurement. */
    private Optional<AprilTagObservation> readAprilTagObservation(boolean seesTarget) {
        if (!seesTarget) return Optional.empty();
        var sample =
                LimelightHelpers.getLimelightDoubleArrayEntry(
                                VisionConstants.kLimelightTableName, "targetpose_cameraspace")
                        .getAtomic();
        double captureLatency = table.getEntry("cl").getDouble(0.0);
        double pipelineLatency = table.getEntry("tl").getDouble(0.0);
        if (!Double.isFinite(captureLatency)
                || !Double.isFinite(pipelineLatency)
                || captureLatency < 0.0
                || pipelineLatency < 0.0) {
            return Optional.empty();
        }
        // 使用 NT 更新時間減掉相機延遲；不可用「現在」刷新舊影格的有效期限。
        double captureTime =
                sample.timestamp / 1_000_000.0 - (captureLatency + pipelineLatency) / 1_000.0;
        return AprilTagObservation.fromLimelight(
                table.getEntry("tid").getDouble(-1.0), sample.value, captureTime);
    }

    /** Reads data from a single Limelight camera. */
    private void readCameraData(
            NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {
        camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
        camera.aprilTagObservation = readAprilTagObservation(camera.seesTarget);
        camera.heartbeat = table.getEntry("hb").getDouble(Double.NaN);
        // 場地定位依各自的 tagCount，不以局部追蹤的 tv / tid 作為有效條件。
        // 每輪清空兩組資料，斷線時仍由 subsystem 依原時間戳判定過期。
        camera.megatagPoseEstimate = null;
        camera.megatag2PoseEstimate = null;
        camera.megatagCount = 0;
        camera.megatag2Count = 0;
        camera.megatag2AverageTagDistanceMeters = Double.NaN;
        camera.fiducialObservations = new FiducialObservation[0];
        camera.pose3d = null;
        camera.standardDeviations = table.getEntry("stddevs").getDoubleArray(DEFAULT_STDDEVS);

        var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt1 != null && mt1.tagCount > 0) {
            camera.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(mt1);
            camera.megatagCount = mt1.tagCount;
            camera.fiducialObservations = FiducialObservation.fromLimelight(mt1.rawFiducials);
            camera.pose3d = LimelightHelpers.toPose3D(
                    LimelightHelpers.getBotPose_wpiBlue(limelightName));
        }
        if (mt2 != null && mt2.tagCount > 0) {
            camera.megatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(mt2);
            camera.megatag2Count = mt2.tagCount;
            camera.megatag2AverageTagDistanceMeters = mt2.avgTagDist;
        }
    }
}
