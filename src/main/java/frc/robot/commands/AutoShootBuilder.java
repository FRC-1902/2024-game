// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.SetPivotCommand;;


public class AutoShootBuilder{
  Shooter shooterSubsystem;
  Pivot pivotSubsystem;
  Swerve swerveSubsystem;
  IMU imu;
  AutoDriveBuilder autoDriveCommands;

  Command shotSequence;

  public Command getShotSequence() {    
    shotSequence = new SequentialCommandGroup(
      new InstantCommand(() -> DataLogManager.log("Distance: " + getVecDistance(calculateTargetVector()))),
      // check is within distance
      new ConditionalCommand(
        // proper shot sequence
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            autoDriveCommands.getTurnCommand(this::calculateFaceAngle),
            new InstantCommand(() -> System.out.println(calculateShotAngle())),
            new SetPivotCommand(this::calculateShotAngle, pivotSubsystem),
            new InstantCommand(() -> shooterSubsystem.setFlywheelRPM(3200))
          ),
          new ShootCommand(shooterSubsystem, pivotSubsystem)
        ),
        // if not within distance
        new InstantCommand(() -> DataLogManager.log("BAD SHOT DISTANCE")),
        () -> getVecDistance(calculateTargetVector()) < Constants.Arm.SHOOTER_MAX_DISTANCE || shooterSubsystem.midPieceSensorActive()
      )
    );
    return shotSequence;
  }

  /**
   * Immediately schedules routine to shoot automatically into the speaker
   */
  public void startShotSequence() {
    shotSequence = getShotSequence();
    shotSequence.schedule();
  }

  public void cancelShotSequence() {
    if (shotSequence != null && !shotSequence.isFinished()) {
      new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem).schedule();
      shooterSubsystem.setFlywheelRPM(0);
      shotSequence.cancel();
    }
  }

  public boolean shotSequenceFinished() {
    return shotSequence != null && shotSequence.isFinished();
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
      Constants.Arm.SHOOTER_MAGIC_D
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

    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        targetPosition = FieldConstants.BlueConstants.SPEAKER;
      } else {
        targetPosition = FieldConstants.RedConstants.SPEAKER;
      }
    } else {
      // assume alliance is blue if alliance isn't set
      targetPosition = FieldConstants.BlueConstants.SPEAKER;
    }

    // redefine target position vector relative to current position as new origin
    return currentPosition.relativeTo(targetPosition).getTranslation().times(-1);
    // targetPosition = targetPosition.relativeTo(currentPosition);

    // return targetPosition.getTranslation();
  }
}
