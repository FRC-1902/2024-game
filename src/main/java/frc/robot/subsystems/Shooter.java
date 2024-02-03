// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;


public class Shooter extends SubsystemBase {
  private CANSparkMax leftShooterMotor, rightShooterMotor;
  private CANSparkMax indexMotor;
  private Rev2mDistanceSensor pieceSensor;

  /** Creates a new Shooter. */ 
  public Shooter() {
    leftShooterMotor = new CANSparkMax(Constants.Arm.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(Constants.Arm.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(leftShooterMotor, Usage.VELOCITY_ONLY);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(rightShooterMotor, Usage.VELOCITY_ONLY);
    leftShooterMotor.enableVoltageCompensation(Constants.Arm.SHOOTER_VOLTAGE_COMPENSATION);
    rightShooterMotor.enableVoltageCompensation(Constants.Arm.SHOOTER_VOLTAGE_COMPENSATION);
    leftShooterMotor.setSmartCurrentLimit(Constants.Arm.SHOOTER_CURRENT_LIMIT);
    rightShooterMotor.setSmartCurrentLimit(Constants.Arm.SHOOTER_CURRENT_LIMIT);
    leftShooterMotor.setIdleMode(IdleMode.kCoast);
    rightShooterMotor.setIdleMode(IdleMode.kCoast);

    indexMotor = new CANSparkMax(Constants.Arm.INDEXER_MOTOR_ID, MotorType.kBrushless);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(indexMotor, Usage.MINIMAL);
    indexMotor.setSmartCurrentLimit(Constants.Arm.INDEX_CURRENT_LIMIT);
    indexMotor.setIdleMode(IdleMode.kBrake); // XXX: maybe?

    pieceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP);
    pieceSensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kMillimeters);
  }

  /**
   * Sets the flywheel to a given power.
   * @param power The power to set the flywheel to between -1 and 1
   * @param differential The differential of power between sides, as to not push power above -1 or
   */
  public void setFlywheel(double power, double differential) {
    leftShooterMotor.set(power + differential);
    rightShooterMotor.set(power - differential);
  }

  // TODO: write this
  public boolean atRPM() {
    return false;
  }

  /**
   * Sets the indexer to a given power.
   * @param power The power to set the indexer to.
   */
  public void setIndexer(double power) {
    indexMotor.set(power);
  }

  public boolean pieceSensorActive() {
    return pieceSensor.getRange() <= Constants.Arm.PIECE_SENSOR_MIN_DIST && pieceSensor.getRange() >= Constants.Arm.PIECE_SENSOR_MAX_DIST;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
