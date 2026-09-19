// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.frc2025;

import com.ctre.phoenix6.SignalLogger;
import com.team254.lib.pathplanner.commands.PathfindingCommand;
import com.team254.lib.pathplanner.pathfinding.Pathfinding;
import com.team254.lib.util.CANBusStatusLogger;
import com.team254.lib.util.OSUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Robot lifecycle for the Drive + Vision branch. Autonomous is stationary by default. */
public class Robot extends LoggedRobot {
    private static final int kRTPriority = 2;
    private static final int kNonRTPriority = 1;

    private final RobotContainer robotContainer;
    private final Command warmupCommand;
    private final CANBusStatusLogger driveCAN =
            new CANBusStatusLogger(Constants.kDriveCanBusName);
    private Command autonomousCommand;
    private boolean hasEnabled;
    private double timeOfLastSync;
    private int disabledIterations;

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });

        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            if (!DriverStation.isFMSAttached()) {
                Logger.addDataReceiver(new NT4Publisher());
            }
        } else if (Constants.kIsReplay) {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter());
        }

        Logger.start();
        if (!Logger.hasReplaySource()) {
            RobotController.setTimeSource(RobotController::getFPGATime);
        }

        robotContainer = new RobotContainer();
        if (RobotBase.isSimulation()) {
            robotContainer.getDriveSubsystem().resetOdometry(new Pose2d(3, 3, Rotation2d.kZero));
        }
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SignalLogger.enableAutoLogging(false);
        Pathfinding.ensureInitialized();
        Pathfinding.setTeleopObstacles();
        // Warm up the same local PathPlanner fork used by Drive; its output consumer is a no-op.
        warmupCommand = PathfindingCommand.warmupCommand();
        warmupCommand.schedule();
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(
                DriverStation.isEnabled(), DriverStation.isEnabled() ? kRTPriority : kNonRTPriority);
        try {
            CommandScheduler.getInstance().run();
            robotContainer.getRobotState().updateLogger();
        } finally {
            Threads.setCurrentThreadPriority(false, kNonRTPriority);
        }
    }

    private void cancelAutonomousAndStop() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
        robotContainer.getDriveSubsystem().stop();
    }

    @Override
    public void disabledInit() {
        cancelAutonomousAndStop();
        timeOfLastSync = Timer.getFPGATimestamp();
        disabledIterations = 0;
        Pathfinding.setTeleopObstacles();
        Pathfinding.enableCaching();
        Pathfinding.setCacheDistanceToleranceMeters(0.0);
    }

    @Override
    public void disabledPeriodic() {
        if (hasEnabled && Timer.getFPGATimestamp() - timeOfLastSync >= 10.0) {
            OSUtil.fsSyncAsync();
            timeOfLastSync = Timer.getFPGATimestamp();
        }
        if (disabledIterations++ % 50 == 0) {
            NetworkTableInstance.getDefault().flush();
        }
        driveCAN.logStatus();
    }

    @Override
    public void disabledExit() {
        warmupCommand.cancel();
        robotContainer.getVisionSubsystem().setUseVision(true);
    }

    @Override
    public void autonomousInit() {
        cancelAutonomousAndStop();
        hasEnabled = true;
        Pathfinding.setCacheDistanceToleranceMeters(0.8);
        autonomousCommand = robotContainer.getAutonomousCommand();
        autonomousCommand.schedule();
    }

    @Override
    public void autonomousExit() {
        cancelAutonomousAndStop();
    }

    @Override
    public void teleopInit() {
        cancelAutonomousAndStop();
        hasEnabled = true;
        Pathfinding.disableCaching();
        Pathfinding.setTeleopObstacles();
    }

    @Override
    public void teleopExit() {
        robotContainer.getDriveSubsystem().stop();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        robotContainer.getDriveSubsystem().stop();
    }

    @Override
    public void testExit() {
        robotContainer.getDriveSubsystem().stop();
    }

    // DriveIOSim owns the physics tick; simulationPeriodic must not advance MapleSim again.
}
