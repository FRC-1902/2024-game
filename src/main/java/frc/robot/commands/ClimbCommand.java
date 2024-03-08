// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.Climber.Direction;
import frc.robot.subsystems.Controllers.ControllerName;

public class ClimbCommand extends Command {
  Climber climberSubsystem;
  Controllers controllers;

  /** Creates a new Climb. */
  public ClimbCommand(Climber climberSubsystem) {
    this.climberSubsystem = climberSubsystem; 
    controllers = Controllers.getInstance();
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dpadAngle = controllers.getDPAD(ControllerName.MANIP);
    // if dpad isn't pressed
    if (dpadAngle == -1) {
      return;
    }
    
    if (dpadAngle <= 45 || dpadAngle >= 315) {
      climberSubsystem.setDirection(Direction.UP);
    } else if (dpadAngle >= 135 && dpadAngle <= 225) {
      climberSubsystem.setDirection(Direction.DOWN);
    } else if (dpadAngle == 90 || dpadAngle == 270) {
      climberSubsystem.setDirection(Direction.STOP);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
