// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Climber;

public class Robot extends LoggedRobot {
  private PowerDistribution pdh;
  private AutoSelector autoSelector;
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private static Robot instance;

  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStar());

    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/media/sda")); // Log to a USB stick
      // Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      pdh = new PowerDistribution(21, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    robotContainer = new RobotContainer();
    autoSelector = new AutoSelector(robotContainer);

    instance = this;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    robotContainer.climberSubsystem.setDirection(Climber.Direction.STOP);
    robotContainer.pivotSubsystem.setAngle(robotContainer.pivotSubsystem.getDefaultAngle());
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Reset Swerve Module to absolute because of race condition on start up
    robotContainer.swerveSubsystem.resetModulesToAbsolute();

    robotContainer.resetPIDs();

    autonomousCommand = autoSelector.getSelectedCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    // Reset Swerve Module to absolute because of race condition on start up
    robotContainer.swerveSubsystem.resetModulesToAbsolute();

    robotContainer.resetPIDs();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    robotContainer.resetPIDs();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static Robot getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Robot instance not initialized");
    }
    return instance;
  }
}
