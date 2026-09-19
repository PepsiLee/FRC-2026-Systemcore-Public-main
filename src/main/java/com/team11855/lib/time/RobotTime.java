package com.team11855.lib.time;

import edu.wpi.first.wpilibj.Timer;

public class RobotTime {
    public static double getTimestampSeconds() {
        return Timer.getFPGATimestamp();
    }
}
