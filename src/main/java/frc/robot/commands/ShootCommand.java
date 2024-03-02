// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
  Shooter shooterSubsystem;
  boolean earlyExit;
  Double shotTime;
  Double elapsedTime;

  /** Creates a new ShootCommand. */
  public ShootCommand(Shooter shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);

    shotTime = null;
    earlyExit = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!shooterSubsystem.pieceSensorActive()) {
      DataLogManager.log("NO Piece Detected to Shoot, exiting shoot command early");
      earlyExit = true;
      return;
    } else {
      earlyExit = false;
    }
    shooterSubsystem.setFlywheel(1, 0);
    elapsedTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (earlyExit) {
      return;
    }

    // shoot once revved up or time elapsed is greater than 1.5 seconds
    if (shooterSubsystem.atRPM() || Timer.getFPGATimestamp() - elapsedTime > 1.5) {
      shooterSubsystem.setIndexer(1);
    }

    // get ready to exit when piece is no longer detected
    if (!shooterSubsystem.pieceSensorActive()) {
      shotTime = Timer.getFPGATimestamp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheel(0, 0);
    shooterSubsystem.setIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if time elapsed after shot is greater than 0.5 seconds, end command
    return earlyExit || (shotTime != null && Timer.getFPGATimestamp() - shotTime > 0.5);
  }
}
