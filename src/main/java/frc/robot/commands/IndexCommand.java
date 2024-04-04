// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.Shooter;

public class IndexCommand extends Command {
  Shooter shooterSubsystem;
  boolean exit;
  /** Creates a new TmpIndexCommand. */
  public IndexCommand(Shooter shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setIndexer(1);
    exit = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.topPieceSensorActive()) {
      shooterSubsystem.setIndexer(-0.15);
      exit = true;
    } else {
      shooterSubsystem.setIndexer(1.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setIndexer(0.0);

    if (shooterSubsystem.midPieceSensorActive())
      Controllers.getInstance().vibrate(100, 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.midPieceSensorActive() && !shooterSubsystem.topPieceSensorActive() && exit;
  }
}
