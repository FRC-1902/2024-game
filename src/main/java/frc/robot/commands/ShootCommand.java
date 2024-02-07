// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
  Shooter shootSubsystem;
  boolean earlyExit;
  Double shotTime;

  /** Creates a new ShootCommand. */
  public ShootCommand(Shooter shooterSubsystem) {
    this.shootSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootSubsystem);

    shotTime = null;
    earlyExit = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!shootSubsystem.pieceSensorActive()) {
      earlyExit = true;
      return;
    } else {
      earlyExit = false;
    }
    shootSubsystem.setFlywheel(1, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (earlyExit) {
      return;
    }

    // shoot once revved up
    if (shootSubsystem.atRPM()) {
      shootSubsystem.setIndexer(1);
    }

    // get ready to exit when piece is no longer detected
    if (!shootSubsystem.pieceSensorActive()) {
      shotTime = Timer.getFPGATimestamp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootSubsystem.setFlywheel(0, 0);
    shootSubsystem.setIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if time elapsed after shot is greater than 0.5 seconds, end command
    return earlyExit || (shotTime != null && Timer.getFPGATimestamp() - shotTime > 0.5);
  }
}
