package com.team11855.frc2026.subsystems.vision;

import com.team11855.frc2026.Constants;
import com.team11855.frc2026.Constants.VisionConstants;
import com.team11855.frc2026.RobotState;
import com.team11855.frc2026.simulation.SimulatedDriveState;
import com.team11855.lib.limelight.LimelightHelpers;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Simulation implementation of VisionIO using PhotonVision simulation. Extends Limelight hardware
 * implementation to reuse data processing logic.
 */
public class VisionIOSimPhoton extends VisionIOHardwareLimelight {
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.kLimelightTableName);
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;
    private final SimulatedDriveState simulatedDriveState;

    private final int kResWidth = 1280;
    private final int kResHeight = 800;

    /** Creates a new simulated vision IO instance using PhotonVision. */
    public VisionIOSimPhoton(RobotState state, SimulatedDriveState simulatedDriveState) {
        super(state);
        this.simulatedDriveState = simulatedDriveState;

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(Constants.kAprilTagLayoutReefsOnly);

        SimCameraProperties prop = new SimCameraProperties();
        prop.setCalibration(kResWidth, kResHeight, Rotation2d.fromDegrees(97.7));
        prop.setCalibError(0.35, 0.5);
        prop.setFPS(45);
        prop.setAvgLatencyMs(20);
        prop.setLatencyStdDevMs(5);
        prop.setExposureTimeMs(0.65);

        cameraSim = new PhotonCameraSim(camera, prop);
        cameraSim.setMinTargetAreaPixels(1000);

        // 使用與實機相同的安裝設定，只建立一顆模擬鏡頭。
        visionSim.addCamera(cameraSim, VisionConstants.kRobotToCamera);

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        Pose2d simulatedPose = simulatedDriveState.getLatestFieldToRobot();
        if (simulatedPose != null) {
            visionSim.update(simulatedPose);
            Logger.recordOutput("Vision/SimIO/updateSimPose", simulatedPose);
        }

        // 寫入父類別讀取的同一張表，沿用實機的單相機處理路徑。
        writeToTable(camera.getAllUnreadResults(), table, cameraSim);

        super.readInputs(inputs);
    }

    /** Generates robot pose data from PhotonVision results. */
    private List<Double> getBotpose(
            Transform3d fieldToCamera,
            int numTags,
            PhotonPipelineResult result,
            PhotonCameraSim cameraSim) {
        if (result == null || !result.hasTargets()) return null;

        Optional<Transform3d> optRobotToCamera =
                visionSim.getRobotToCamera(cameraSim, Timer.getFPGATimestamp());
        Pose3d fieldToRobot;
        if (optRobotToCamera.isPresent()) {
            Transform3d cameraToRobot = optRobotToCamera.get().inverse();
            Pose3d robotPose3d =
                    new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
                            .transformBy(cameraToRobot);
            fieldToRobot = robotPose3d;
        } else {
            fieldToRobot = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());
        }

        List<Double> pose_data =
                new ArrayList<>(
                        Arrays.asList(
                                fieldToRobot.getX(),
                                fieldToRobot.getY(),
                                fieldToRobot.getZ(),
                                0.0,
                                0.0,
                                fieldToRobot.getRotation().getMeasureZ().in(Units.Degree),
                                result.metadata.getLatencyMillis(),
                                (double) numTags,
                                0.0,
                                0.0,
                                result.getBestTarget().getArea()));

        for (var target : result.getTargets()) {
            pose_data.addAll(
                    Arrays.asList(
                            (double) target.getFiducialId(),
                            target.getYaw(), // txnc
                            target.getPitch(), // tync
                            target.getArea(), // ta
                            0.0, // distToCamera
                            0.0, // distToRobot
                            target.getPoseAmbiguity() // ambiguity
                            ));
        }
        return pose_data;
    }

    /**
     * Writes simulated vision data to NetworkTables for consumption by Limelight processing code.
     */
    private void writeToTable(
            List<PhotonPipelineResult> results, NetworkTable table, PhotonCameraSim cameraSim) {
        // 沒有新影格不等於失去標籤；保留原時間戳，追蹤命令仍會檢查資料逾時。
        if (results.isEmpty()) return;
        for (var result : results) {
            boolean seesTarget = false;
            List<Double> pose_data = null;
            if (result.getMultiTagResult().isPresent()) {
                var multiTagResult = result.getMultiTagResult().get();
                Transform3d best = multiTagResult.estimatedPose.best;

                pose_data =
                        getBotpose(best, multiTagResult.fiducialIDsUsed.size(), result, cameraSim);
            } else if (result.hasTargets()) {
                var bestTarget = result.getBestTarget();
                Transform3d best =
                        Constants.kAprilTagLayoutReefsOnly
                                .getTagPose(bestTarget.getFiducialId())
                                .get()
                                .minus(Pose3d.kZero)
                                .plus(bestTarget.getBestCameraToTarget().inverse());

                pose_data = getBotpose(best, 1, result, cameraSim);
            }

            if (pose_data != null) {
                table.getEntry("botpose_wpiblue")
                        .setDoubleArray(
                                pose_data.stream().mapToDouble(Double::doubleValue).toArray());
                table.getEntry("botpose_orb_wpiblue")
                        .setDoubleArray(
                                pose_data.stream().mapToDouble(Double::doubleValue).toArray());
                // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll,
                // MT2pitch,
                // MT2yaw]
                table.getEntry("stddevs")
                        .setDoubleArray(
                                new Double[] {
                                    0.3, 0.3, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                                });
                seesTarget = true;
            }
            table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
            table.getEntry("tl").setDouble(0.0); // cl 已包含 Photon 的總延遲。
            if (result.hasTargets()) {
                var bestTarget = result.getBestTarget();
                var translation = bestTarget.getBestCameraToTarget().getTranslation();
                table.getEntry("tid").setDouble(bestTarget.getFiducialId());
                // Photon 前／左／上 → Limelight 右／下／前；追蹤只使用前三個位置值。
                // 保留 Photon 的拍攝時間；即使一次讀到多張排隊影格也不把舊資料刷新成現在。
                long publishTimestamp =
                        Math.round((result.getTimestampSeconds()
                                + result.metadata.getLatencyMillis() / 1_000.0) * 1_000_000.0);
                LimelightHelpers.getLimelightDoubleArrayEntry(
                                VisionConstants.kLimelightTableName, "targetpose_cameraspace")
                        .set(
                                new double[] {
                                    -translation.getY(), -translation.getZ(), translation.getX(),
                                    0.0, 0.0, 0.0
                                },
                                publishTimestamp);
            } else {
                table.getEntry("tid").setDouble(-1.0);
                table.getEntry("targetpose_cameraspace").setDoubleArray(new double[0]);
            }
            table.getEntry("tv").setInteger(seesTarget ? 1 : 0);
        }
    }
}
