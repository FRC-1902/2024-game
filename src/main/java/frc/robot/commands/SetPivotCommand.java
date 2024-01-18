// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class SetPivotCommand extends Command {
  Pivot pivotSubsystem;

  Rotation2d angle;

  /**
   * Creates a new SetPivotCommand.
   * @param angle rotation from directly down
   */
  public SetPivotCommand(Rotation2d angle) {
    pivotSubsystem = Pivot.getInstance();
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotSubsystem.setAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotSubsystem.atSetpoint();
  }
}
