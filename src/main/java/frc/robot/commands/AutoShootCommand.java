// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoShootCommand{
  Shooter shooterSubsystem;
  Pivot pivotSubsystem;
  Swerve swerveSubsystem;
  IMU imu;
  AutoDriveCommands autoDriveCommands;

  /**
   * Immediately schedules routine to shoot automatically into the speaker
   */
  public void startAutoShotSequence() {
    new SequentialCommandGroup(
      autoDriveCommands.turnCommand(calculateFaceAngle()),
      new SetPivotCommand(calculateShotAngle(), pivotSubsystem),
      new ShootCommand(shooterSubsystem)
    ).schedule();
  }

  /** Creates a new AutoShootCommand. */
  public AutoShootCommand(AutoDriveCommands autoDriveCommands, Shooter shooterSubsystem, Pivot pivotSubsystem, Swerve swerveSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    imu = IMU.getInstance();

    this.autoDriveCommands = autoDriveCommands;
  }

  private Rotation2d calculateFaceAngle() {
    // get angle of target position vector to determine where to face
    return calculateTargetVector().getAngle();
  }

  private Rotation2d calculateShotAngle() {
    Translation2d targetVector = calculateTargetVector();
    double distance = Math.sqrt(Math.pow(targetVector.getX(), 2) + Math.pow(targetVector.getY(), 2));

    // magic precalculated numbers!!!!! based on how shooter distance relates to the target angle in degrees
    return Rotation2d.fromDegrees(
      Constants.Arm.SHOOTER_MAGIC_A * Math.pow(distance, 3) + 
      Constants.Arm.SHOOTER_MAGIC_B * Math.pow(distance, 2) + 
      Constants.Arm.SHOOTER_MAGIC_C * distance + 
      Constants.Arm.SHOOTER_MAGIC_C
    );
  }

  /**
   * @return vector starting at the robot's current position and ending at the speaker position
   */
  private Translation2d calculateTargetVector() {
    Pose2d currentPosition = swerveSubsystem.getPose();
    Pose2d targetPosition;

    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        targetPosition = FieldConstants.BlueConstants.SPEAKER;
      } else {
        targetPosition = FieldConstants.RedConstants.SPEAKER;
      }
    } else {
      // assume alliance is blue if alliance isn't set
      targetPosition = FieldConstants.BlueConstants.SPEAKER;
    }

    // redefine target position vector relative to current position as new origin
    targetPosition = targetPosition.relativeTo(currentPosition);

    return targetPosition.getTranslation();
  }
}
