// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoShootBuilder{
  Shooter shooterSubsystem;
  Pivot pivotSubsystem;
  Swerve swerveSubsystem;
  IMU imu;
  AutoDriveBuilder autoDriveCommands;

  Command shotSequence;

  /**
   * Immediately schedules routine to shoot automatically into the speaker
   */
  public void startShotSequence() {
    DataLogManager.log(String.format("Starting Shot Sequence at X: %.3f Y: %.3f", swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY()));
    DataLogManager.log("Distance: " + getVecDistance(calculateTargetVector()));
    
    if (getVecDistance(calculateTargetVector()) > Constants.Arm.SHOOTER_MAX_DISTANCE) {
      return;
    }
    
    // shotSequence = new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     autoDriveCommands.getTurnCommand(calculateFaceAngle()),
    //     new SetPivotCommand(calculateShotAngle(), pivotSubsystem),
    //     new InstantCommand(() -> shooterSubsystem.setFlywheel(1, 0))
    //   ),
    //   new ShootCommand(shooterSubsystem, pivotSubsystem)
    // );
    // shotSequence.schedule();
  }

  public void cancelShotSequence() {
    if (shotSequence != null && !shotSequence.isFinished()) {
      shooterSubsystem.setFlywheel(0, 0);
      shotSequence.cancel();
    }
  }

  public boolean isShotDone() {
    return shotSequence.isFinished();
  }

  /** Creates a new AutoShootCommand. */
  public AutoShootBuilder(AutoDriveBuilder autoDriveCommands, Shooter shooterSubsystem, Pivot pivotSubsystem, Swerve swerveSubsystem) {
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
    double distance = getVecDistance(targetVector);

    // magic precalculated numbers!!!!! based on how shooter distance relates to the target angle in degrees
    return Rotation2d.fromDegrees(
      Constants.Arm.SHOOTER_MAGIC_A * Math.pow(distance, 3) + 
      Constants.Arm.SHOOTER_MAGIC_B * Math.pow(distance, 2) + 
      Constants.Arm.SHOOTER_MAGIC_C * distance + 
      Constants.Arm.SHOOTER_MAGIC_C
    );
  }

  private double getVecDistance(Translation2d vec) {
    return Math.sqrt(Math.pow(vec.getX(), 2) + Math.pow(vec.getY(), 2));
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
