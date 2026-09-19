package com.team11855.frc2026.subsystems.vision;

import com.team11855.frc2026.Constants.VisionConstants;
import com.team11855.frc2026.RobotState;
import com.team11855.lib.limelight.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

/** Hardware implementation of VisionIO using a single Limelight camera. */
public class VisionIOHardwareLimelight implements VisionIO {
    protected final NetworkTable table =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
    RobotState robotState;
    AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());
    int imuMode = 1;

    private static final double[] DEFAULT_STDDEVS =
            new double[VisionConstants.kExpectedStdDevArrayLength];

    /** Creates a new Limelight vision IO instance. */
    public VisionIOHardwareLimelight(RobotState robotState) {
        this.robotState = robotState;
        setLLSettings();
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
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        readCameraData(table, inputs.camera, VisionConstants.kLimelightTableName);
        latestInputs.set(inputs);
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
        if (camera.seesTarget) {
            try {
                var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                var robotPose3d =
                        LimelightHelpers.toPose3D(
                                LimelightHelpers.getBotPose_wpiBlue(limelightName));

                if (megatag != null) {
                    camera.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                    camera.megatagCount = megatag.tagCount;
                    camera.fiducialObservations =
                            FiducialObservation.fromLimelight(megatag.rawFiducials);
                }
                if (robotPose3d != null) {
                    camera.pose3d = robotPose3d;
                }

                camera.standardDeviations =
                        table.getEntry("stddevs").getDoubleArray(DEFAULT_STDDEVS);
            } catch (Exception e) {
                System.err.println("Error processing Limelight data: " + e.getMessage());
            }
        }
    }
}
