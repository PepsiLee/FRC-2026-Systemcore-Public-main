package com.team11855.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

import java.util.Optional;

/** Interface for vision system hardware abstraction. */
public interface VisionIO {

    /** Container for all vision input data. */
    class VisionIOInputs {
        /** Input data from a single camera. */
        public static class CameraInputs {
            public double heartbeat = Double.NaN;
            public boolean seesTarget;
            // 局部追蹤量測獨立於場地定位；看不到目標時每輪清空，不保留舊有效旗標。
            public Optional<AprilTagObservation> aprilTagObservation = Optional.empty();
            public FiducialObservation[] fiducialObservations;
            public MegatagPoseEstimate megatagPoseEstimate;
            public int megatagCount;
            public Pose3d pose3d;
            public double[] standardDeviations =
                    new double[12]; // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x,
            // MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
        }

        /** 唯一一顆 Limelight 的輸入；實機與模擬使用相同資料結構。 */
        public final CameraInputs camera = new CameraInputs();
    }

    void readInputs(VisionIOInputs inputs);
}
