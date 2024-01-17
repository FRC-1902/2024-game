// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private static Pivot instance;

  private CANSparkMax pivotMotor1;
  private CANSparkMax pivotMotor2;
  private DutyCycleEncoder pivotEncoder;
  private PIDController pivotPID;

  /** Creates a new Pivot. */
  private Pivot() {
    pivotMotor1 = new CANSparkMax(Constants.Arm.PIVOT_MOTOR_1_ID, MotorType.kBrushless);
    pivotMotor2 = new CANSparkMax(Constants.Arm.PIVOT_MOTOR_1_ID, MotorType.kBrushless);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(pivotMotor1, Usage.MINIMAL);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(pivotMotor2, Usage.MINIMAL);
    pivotMotor1.setSmartCurrentLimit(Constants.Arm.PIVOT_CURRENT_LIMIT);
    pivotMotor2.setSmartCurrentLimit(Constants.Arm.PIVOT_CURRENT_LIMIT);
    pivotMotor1.setIdleMode(IdleMode.kBrake);
    pivotMotor2.setIdleMode(IdleMode.kBrake);

    pivotMotor2.follow(pivotMotor1);

    pivotEncoder = new DutyCycleEncoder(Constants.Arm.PIVOT_ENCODER_PORT);
    pivotEncoder.setPositionOffset(Constants.Arm.PIVOT_ANGLE_OFFSET); // TODO: set me

    pivotPID = new PIDController(Constants.Arm.PIVOT_KP, Constants.Arm.PIVOT_KI, Constants.Arm.PIVOT_KD);
    pivotPID.enableContinuousInput(0, 1);
    pivotPID.setTolerance(Constants.Arm.PIVOT_DEGREES_TOLERANCE);
  }

  /**
   * Sets the angle of the arm.
   * Returns early if the arm is out of bounds specified in constants
   * @param angle The angle to set the arm to.
   */
  public void setAngle(Rotation2d angle) {
    if (angle.getRotations() > Constants.Arm.PIVOT_MAX_ROTATION.getRotations() || angle.getDegrees() < Constants.Arm.PIVOT_MIN_ROTATION.getRotations()) {
      return;
    }
    pivotPID.setSetpoint(angle.getRotations());
  }

  public void setToDefaultAngle() {
    setAngle(new Rotation2d(0)); // TODO: set me
  }

  /**
   * Gets the angle of the arm, 0 is directly down into the pivot
   * @return The angle of the arm.
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition());
  }

  /**
   * Gets the angle of the shooter, where 0 is horizontal to the ground
   * @param pivotAngle The angle of the pivot.
   * @return Pose2d of The angle of the shooter & location relative to pivot
   */
  public static Rotation2d getShooterTheta(Rotation2d pivotAngle) {
    Translation2d armTranslation = new Translation2d(
      -Constants.Arm.ARM_LENGTH * Math.sin(pivotAngle.getRadians()), 
      -Constants.Arm.ARM_LENGTH * Math.cos(pivotAngle.getRadians())
    );

    Translation2d wristTranslation = new Translation2d(
      armTranslation.getX() + Constants.Arm.WRIST_LENGTH * Math.cos(Constants.Arm.WRIST_OFFSET.getRadians() + (Math.PI / 2.0) - pivotAngle.getRadians()),
      armTranslation.getY() + Constants.Arm.WRIST_LENGTH * Math.sin(Constants.Arm.WRIST_OFFSET.getRadians() + (Math.PI / 2.0) - pivotAngle.getRadians())
    );

    // Stop divide by 0 errors if pointed straight up or down
    if (armTranslation.getX() == wristTranslation.getX()) {
      if (armTranslation.getY() - wristTranslation.getY() > 0) {
        return Rotation2d.fromDegrees(90);
      } else {
        return Rotation2d.fromDegrees(270);
      }
    }

    return Rotation2d.fromRadians(Math.atan(
      (armTranslation.getY() - wristTranslation.getY()) / (armTranslation.getX() - wristTranslation.getX())
    ));
  }

  /**
   * Gets the angle of the shooter, where 0 is horizontal to the ground
   * @return Pose2d of The angle of the shooter & location relative to pivot
   */
  public Rotation2d getCurrentShooterTheta() {
    return getShooterTheta(getAngle());
  }

  /**
   * @return if the pivot pid is at the setpoint
   */
  public boolean atSetpoint() {
    return pivotPID.atSetpoint();
  }

  /**
   * Pivot watchdog, checking if the pivot is out of bounds
   * @return isOutOfBounds
   */
  private boolean checkPivotWatchdog() {
    Rotation2d currentAngle = getAngle();
    if (currentAngle.getDegrees() > Constants.Arm.PIVOT_MAX_ROTATION.getDegrees() || currentAngle.getDegrees() < Constants.Arm.PIVOT_MIN_ROTATION.getDegrees()) {
      DataLogManager.log("[Pivot Watchdog] Arm out of bounds!");
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    if (checkPivotWatchdog()) {
      return;
    }

    // TODO: TEST BEFORE MOVING ARM!!!!!!!!!!!!!
    pivotMotor1.set(
      pivotPID.calculate(pivotEncoder.getAbsolutePosition()) 
      + Constants.Arm.PIVOT_KF * Math.sin(pivotEncoder.getAbsolutePosition() * Math.PI * 2)
    );
  }

  public static Pivot getInstance() {
    if (instance == null) {
      instance = new Pivot();
    }
    return instance;
  }
}
