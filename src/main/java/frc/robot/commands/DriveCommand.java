// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Controllers.Axis;
import frc.robot.subsystems.Controllers.Button;
import frc.robot.subsystems.Controllers.ControllerName;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.Swerve;

public class DriveCommand extends Command {
  Swerve swerveSubsystem;
  Controllers controllers;
  /** Creates a new DriveCommand. */
  public DriveCommand(Swerve swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);

    this.swerveSubsystem = swerveSubsystem;
    controllers = Controllers.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.LY), Constants.STICK_DEADBAND);
    double strafeVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.LX), Constants.STICK_DEADBAND);
    double rotationVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.RX), Constants.STICK_DEADBAND);
    boolean isFieldRelative = !controllers.get(ControllerName.DRIVE, Button.LB);

    // cube controls for better handling, and scale down for softer pre-season movement
    translationVal = Math.pow(translationVal, 3.0);
    translationVal *= 1.0;
    strafeVal = Math.pow(strafeVal, 3.0);
    strafeVal *= 1.0;
    rotationVal = Math.pow(rotationVal, 3.0);
    rotationVal *= 0.2;

    // drive
    swerveSubsystem.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
        rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY, 
        isFieldRelative, 
        true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(
      new Translation2d(0, 0), 
      0.0, 
      true, 
      true
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
