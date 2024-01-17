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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.OperationMode;
import frc.robot.modes.AutoMode;
import frc.robot.modes.DisabledMode;
import frc.robot.modes.TeleOpMode;
import frc.robot.modes.TestMode;
import frc.robot.subsystems.IMU;

public class Robot extends LoggedRobot {
  private IMU imu;
  private PowerDistribution pdh;

  private OperationMode disabledMode, autoMode, teleOpMode, testMode;

  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStar());

    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/media/sda")); // Log to a USB stick
      // Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      pdh = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    imu = IMU.getInstance();

    disabledMode = new DisabledMode();
    autoMode = new AutoMode();
    teleOpMode = new TeleOpMode();
    testMode = new TestMode();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    disabledMode.enter();
  }

  @Override
  public void disabledPeriodic() {
    disabledMode.periodic();
  }

  @Override
  public void disabledExit() {
    disabledMode.exit();
  }

  @Override
  public void autonomousInit() {
    autoMode.enter();
  }

  @Override
  public void autonomousPeriodic() {
    autoMode.periodic();
  }

  @Override
  public void autonomousExit() {
    autoMode.exit();
  }

  @Override
  public void teleopInit() {
    teleOpMode.enter();
  }

  @Override
  public void teleopPeriodic() {
    teleOpMode.periodic();
  }

  @Override
  public void teleopExit() {
    teleOpMode.exit();
  }

  @Override
  public void testInit() {
    testMode.enter();
  }

  @Override
  public void testPeriodic() {
    testMode.periodic();
  }

  @Override
  public void testExit() {
    testMode.exit();
  }
}
