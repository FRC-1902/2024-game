// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private CANSparkMax pivotMotor1;
  private CANSparkMax pivotMotor2;
  private SparkAbsoluteEncoder pivotEncoder;
  private ProfiledPIDController pivotPID;

  private double watchdogPrintTime;

  /** Creates a new Pivot. */
  public Pivot(Shooter shooterSubsystem) {
    // R pivot
    pivotMotor1 = new CANSparkMax(Constants.Arm.PIVOT_MOTOR_1_ID, MotorType.kBrushless);
    // L pivot
    pivotMotor2 = new CANSparkMax(Constants.Arm.PIVOT_MOTOR_2_ID, MotorType.kBrushless);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(pivotMotor1, Usage.ALL); // XXX:
    CANSparkMaxUtil.setCANSparkMaxBusUsage(pivotMotor2, Usage.ALL); // XXX:
    pivotMotor1.setSmartCurrentLimit(Constants.Arm.PIVOT_CURRENT_LIMIT);
    pivotMotor2.setSmartCurrentLimit(Constants.Arm.PIVOT_CURRENT_LIMIT);
    pivotMotor1.setIdleMode(IdleMode.kBrake);
    pivotMotor2.setIdleMode(IdleMode.kBrake);

    pivotMotor1.setInverted(false);
    pivotMotor2.setInverted(true);

    pivotEncoder = shooterSubsystem.getPivotEncoder();
    pivotEncoder.setInverted(true);
    pivotEncoder.setZeroOffset(Constants.Arm.PIVOT_ANGLE_OFFSET.getRotations());

    pivotPID = new ProfiledPIDController(Constants.Arm.PIVOT_KP, Constants.Arm.PIVOT_KI, Constants.Arm.PIVOT_KD, new TrapezoidProfile.Constraints(100, 1.0));
    pivotPID.setTolerance(Constants.Arm.PIVOT_DEGREES_TOLERANCE);
    pivotPID.setIntegratorRange(-0.15, 0.15);
    pivotPID.setIZone(0.1);
    setAngle(getDefaultAngle());
  }

  /**
   * Sets the angle of the arm.
   * Returns early if the arm is out of bounds specified in constants
   * 
   * @param angle The angle to set the arm to.
   */
  public void setAngle(Rotation2d angle) {
    if (angle.getRotations() > Constants.Arm.PIVOT_MAX_ROTATION.getRotations()
        || angle.getDegrees() < Constants.Arm.PIVOT_MIN_ROTATION.getRotations()) {
      return;
    }
    resetPIDs();
    pivotPID.setGoal(angle.getRotations());
  }

  public Rotation2d getDefaultAngle() {
    return Rotation2d.fromRotations(0.154);
  }

  /**
   * Gets the angle of the arm, 0 is directly down into the pivot
   * 
   * @return The angle of the arm.
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(pivotEncoder.getPosition());
  }

  /**
   * Gets the angle of the shooter, where 0 is horizontal to the ground
   * 
   * @param pivotAngle The angle of the pivot.
   * @return Pose2d of The angle of the shooter & location relative to pivot
   */
  public static Rotation2d getShooterTheta(Rotation2d pivotAngle) {
    Translation2d armTranslation = new Translation2d(
        -Constants.Arm.ARM_LENGTH * Math.sin(pivotAngle.getRadians()),
        -Constants.Arm.ARM_LENGTH * Math.cos(pivotAngle.getRadians()));

    Translation2d wristTranslation = new Translation2d(
        armTranslation.getX() + Constants.Arm.WRIST_LENGTH
            * Math.cos(Constants.Arm.WRIST_OFFSET.getRadians() + (Math.PI / 2.0) - pivotAngle.getRadians()),
        armTranslation.getY() + Constants.Arm.WRIST_LENGTH
            * Math.sin(Constants.Arm.WRIST_OFFSET.getRadians() + (Math.PI / 2.0) - pivotAngle.getRadians()));

    // Stop divide by 0 errors if pointed straight up or down
    if (armTranslation.getX() == wristTranslation.getX()) {
      if (armTranslation.getY() - wristTranslation.getY() > 0) {
        return Rotation2d.fromDegrees(90);
      } else {
        return Rotation2d.fromDegrees(270);
      }
    }

    return Rotation2d.fromRadians(Math.atan(
        (armTranslation.getY() - wristTranslation.getY()) / (armTranslation.getX() - wristTranslation.getX())));
  }

  /**
   * Gets the angle of the shooter, where 0 is horizontal to the ground
   * 
   * @return Pose2d of The angle of the shooter & location relative to pivot
   */
  public Rotation2d getCurrentShooterTheta() {
    return getShooterTheta(getAngle());
  }

  /**
   * @return if the pivot pid is at the setpoint
   */
  public boolean atSetpoint() {
    return pivotPID.atGoal();
  }

  /**
   * Pivot watchdog, checking if the pivot is out of bounds
   * 
   * @return isOutOfBounds
   */
  private boolean checkPivotWatchdog() {
    Rotation2d currentAngle = getAngle();
    if (currentAngle.getDegrees() > Constants.Arm.PIVOT_MAX_ROTATION.getDegrees()
        || currentAngle.getDegrees() < Constants.Arm.PIVOT_MIN_ROTATION.getDegrees()) {
      // reduce spamming of warning messages
      if (Timer.getFPGATimestamp() - watchdogPrintTime > 1) {
        DataLogManager.log("[Pivot Watchdog] Arm out of bounds!");
        watchdogPrintTime = Timer.getFPGATimestamp();
      }
      return true;
    } else {
      return false;
    }
  }

  public void resetPIDs() {
    pivotPID.reset(getAngle().getRotations());
  }

  @Override
  public void periodic() {
    double setPower = pivotPID.calculate(getAngle().getRotations());
    double feedFoward = Constants.Arm.PIVOT_KF * Math.sin(getAngle().getRadians());

    // reducing feedforward power on down strokes 
    if (Math.signum(setPower) == Math.signum(feedFoward) || setPower == 0.0) {
      setPower += feedFoward;
    } else {
      setPower += feedFoward / 1.7;
    }

    // XXX: maybe migrate to logs?
    SmartDashboard.putNumber("PivotEncoder", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Power", setPower);
    SmartDashboard.putNumber("Pivot Current 1", pivotMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Current 2", pivotMotor2.getOutputCurrent());

    // just yell if pivot out of bounds
    checkPivotWatchdog();

    // robot not enabled
    if (!Robot.getInstance().isEnabled()) {
      pivotMotor1.set(0);
      pivotMotor2.set(0);

      // integrator safety when stuck out of bounds
      resetPIDs();

      return;
    }

    // don't power pivot when down
    if (pivotPID.getSetpoint().position == getDefaultAngle().getRotations() && pivotPID.atSetpoint() || getAngle().getRotations() < getDefaultAngle().getRotations()) {
      pivotMotor1.set(0);
      pivotMotor2.set(0);
      return;
    }

    pivotMotor1.set(setPower);
    pivotMotor2.set(setPower);
  }
}
