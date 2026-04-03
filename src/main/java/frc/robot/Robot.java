// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.HubShiftUtil;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    
    private final RobotContainer m_robotContainer;
    
    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildOConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("Robot", Constants.getRobot().toString());
        Logger.recordMetadata("Mode", Constants.getMode().toString());
        Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });
        
        // Set up data receivers & replay source
        switch (Constants.getMode()) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());

                // Setup PDP Logging
                LoggedPowerDistribution.getInstance(1, ModuleType.kRev);

                break;
            
            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                Logger.addDataReceiver(new WPILOGWriter());
                
                // Silence Joystick Warnings
                DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());
                monitorCommandStatus() ;
                break;
            
            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
                break;
        }
        
        // Start AdvantageKit
        Logger.start();

        // Publish Deploy Directory (for layout/asset downloading)
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        // Disable CTRE hoot files
        SignalLogger.enableAutoLogging(false);
        
        m_robotContainer = new RobotContainer();

    }

    private void monitorCommandStatus() {
        // CommandScheduler.getInstance().onCommandInterrupt(command -> System.out.println("CommandInterrupt: " + command.getName()));   
        // CommandScheduler.getInstance().onCommandFinish(command -> System.out.println("CommandFinish: " + command.getName()));
        // CommandScheduler.getInstance().onCommandInitialize(command -> System.out.println("CommandInitialize: " + command.getName()));
    }
    
    @Override
    public void robotPeriodic() {
        RobotState.periodic();
        CommandScheduler.getInstance().run();
    }
    
    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void disabledExit() {}
    
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void autonomousExit() {}
    
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }
    
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void teleopExit() {}
    
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
    
    @Override
    public void testPeriodic() {}
    
    @Override
    public void testExit() {}
}
