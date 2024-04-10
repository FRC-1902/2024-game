// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class OuttakeCommand extends Command {
  /** Creates a new OuttakeCommand. */
  Shooter shooterSubsystem;
  Intake intakeSubsystem;
  /** Creates a new TmpIndexCommand. */
  public OuttakeCommand(Shooter shooterSubsystem, Intake intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setIndexer(-0.4);
    shooterSubsystem.setFlywheelRPM(-5200);
    intakeSubsystem.set(-0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setIndexer(0.0);
    shooterSubsystem.setFlywheelRPM(0);
    intakeSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
