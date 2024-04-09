// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {
  /** Creates a new IntakeCommand. */
  Intake intakeSubsystem;
  Shooter shooterSubsystem;
  IndexCommand indexCommand;

  public IntakeCommand(Intake intakeSubsystem, Shooter shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(intakeSubsystem, shooterSubsystem);

    indexCommand = new IndexCommand(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.set(0.25);
    shooterSubsystem.setIndexer(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.set(0);
    shooterSubsystem.setIndexer(0);

    if (shooterSubsystem.topPieceSensorActive())
      indexCommand.schedule();

    if (shooterSubsystem.midPieceSensorActive())
      Controllers.getInstance().vibrate(100, 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.midPieceSensorActive() && shooterSubsystem.topPieceSensorActive();
  }
}
