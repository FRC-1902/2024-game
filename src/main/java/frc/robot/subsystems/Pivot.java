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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private CANSparkMax pivotMotor1;
  private CANSparkMax pivotMotor2;
  private SparkAbsoluteEncoder pivotEncoder;
  private ProfiledPIDController pivotPID;

  private double watchdogPrintTime;

  private SendableChooser ones;
  private SendableChooser twos;
  private SendableChooser threes;


  /** Creates a new Pivot. */
  public Pivot(Shooter shooterSubsystem) {
    // motor config
    pivotMotor1 = new CANSparkMax(Constants.Arm.PIVOT_MOTOR_1_ID, MotorType.kBrushless);
    pivotMotor2 = new CANSparkMax(Constants.Arm.PIVOT_MOTOR_2_ID, MotorType.kBrushless);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(pivotMotor1, Usage.ALL); // want to know if it's not working
    CANSparkMaxUtil.setCANSparkMaxBusUsage(pivotMotor2, Usage.ALL); // want to know if it's not working
    pivotMotor1.setSmartCurrentLimit(Constants.Arm.PIVOT_CURRENT_LIMIT);
    pivotMotor2.setSmartCurrentLimit(Constants.Arm.PIVOT_CURRENT_LIMIT);
    pivotMotor1.setIdleMode(IdleMode.kBrake);
    pivotMotor2.setIdleMode(IdleMode.kBrake);
    pivotMotor1.setInverted(false);
    pivotMotor2.setInverted(true);

    // encoder config
    pivotEncoder = shooterSubsystem.getPivotEncoder();
    pivotEncoder.setInverted(true);
    pivotEncoder.setZeroOffset(Constants.Arm.PIVOT_ANGLE_OFFSET.getRotations());

    // pid config
    pivotPID = new ProfiledPIDController(Constants.Arm.PIVOT_KP, Constants.Arm.PIVOT_KI, Constants.Arm.PIVOT_KD,
        new TrapezoidProfile.Constraints(100, 4.0));
    pivotPID.setTolerance(Constants.Arm.PIVOT_DEGREES_TOLERANCE.getRotations());
    pivotPID.setIntegratorRange(-0.1, 0.1);
    pivotPID.setIZone(0.1);

    // set down @ init
    setAngle(getDefaultAngle());


    ones = new SendableChooser();
    ones.setDefaultOption("0.2", 0.2);
    ones.addOption("0.3", 0.3);
    ones.addOption("0.4", 0.4);
    ones.addOption("0.5", 0.5);

    twos = new SendableChooser();
    twos.setDefaultOption("0.00", 0.00);
    twos.addOption("0.01", 0.01);
    twos.addOption("0.02", 0.02);
    twos.addOption("0.03", 0.03);
    twos.addOption("0.04", 0.04);
    twos.addOption("0.05", 0.05);
    twos.addOption("0.06", 0.06);
    twos.addOption("0.07", 0.07);
    twos.addOption("0.08", 0.08);
    twos.addOption("0.09", 0.09);

    threes = new SendableChooser();
    threes.setDefaultOption("0.000", 0.000);
    threes.addOption("0.0025", 0.0025);
    threes.addOption("0.005", 0.005);
    threes.addOption("0.0075", 0.0075);

    SmartDashboard.putData("Pivot Ones", ones);
    SmartDashboard.putData("Pivot Twos", twos);
    SmartDashboard.putData("Pivot Threes", threes);
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
    return Rotation2d.fromRotations(0.165);
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
    // double setpoint = (double) ones.getSelected() + (double) twos.getSelected() + (double) threes.getSelected();
    // if (Math.abs(pivotPID.getGoal().position - setpoint) > 0.001)
    //   setAngle(Rotation2d.fromRotations(setpoint));

    double setPower = pivotPID.calculate(getAngle().getRotations());
    double feedFoward = Constants.Arm.PIVOT_KF * Math.sin(getAngle().getRadians());

    // reducing feedforward power on down strokes
    if (Math.signum(setPower) == Math.signum(feedFoward) || setPower == 0.0) {
      setPower += feedFoward;
    } else {
      setPower += feedFoward / 1.0;
    }

    // TODO: migrate to logs
    SmartDashboard.putNumber("PivotEncoder", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Current 1", pivotMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Current 2", pivotMotor2.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Power", setPower);

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
    if (pivotPID.getGoal().position == getDefaultAngle().getRotations()
        && (pivotPID.atGoal() || getAngle().getRotations() < getDefaultAngle().getRotations())) {
      pivotMotor1.set(0);
      pivotMotor2.set(0);
      return;
    }

    pivotMotor1.set(setPower);
    pivotMotor2.set(setPower);
  }
}
