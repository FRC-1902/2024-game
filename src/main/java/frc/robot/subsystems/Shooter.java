// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;


public class Shooter extends SubsystemBase {
  private CANSparkMax topShooterMotor;
  private CANSparkMax bottomShooterMotor;
  private CANSparkMax indexMotor;
  private ColorSensorV3 pieceSensor;

  /** Creates a new Shooter. */ 
  public Shooter() {
    topShooterMotor = new CANSparkMax(Constants.Arm.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    bottomShooterMotor = new CANSparkMax(Constants.Arm.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(topShooterMotor, Usage.VELOCITY_ONLY);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(bottomShooterMotor, Usage.VELOCITY_ONLY);
    topShooterMotor.enableVoltageCompensation(Constants.Arm.SHOOTER_VOLTAGE_COMPENSATION);
    bottomShooterMotor.enableVoltageCompensation(Constants.Arm.SHOOTER_VOLTAGE_COMPENSATION);
    topShooterMotor.setSmartCurrentLimit(Constants.Arm.SHOOTER_CURRENT_LIMIT);
    bottomShooterMotor.setSmartCurrentLimit(Constants.Arm.SHOOTER_CURRENT_LIMIT);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);

    topShooterMotor.setInverted(true);
    bottomShooterMotor.setInverted(true);

    indexMotor = new CANSparkMax(Constants.Arm.INDEXER_MOTOR_ID, MotorType.kBrushless);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(indexMotor, Usage.MINIMAL);
    indexMotor.setSmartCurrentLimit(Constants.Arm.INDEX_CURRENT_LIMIT);
    indexMotor.setIdleMode(IdleMode.kCoast);

    pieceSensor = new ColorSensorV3(Port.kMXP);

    configureShuffleboardData();
  }

  public SparkAbsoluteEncoder getPivotEncoder() {  
    return topShooterMotor.getAbsoluteEncoder();
  }

  /**
   * Sets the flywheel to a given power.
   * @param power The power to set the flywheel to between -1 and 1
   * @param differential The differential of power between sides, as to not push power above -1 or
   */
  public void setFlywheel(double power, double differential) {
    topShooterMotor.set(power + differential);
    bottomShooterMotor.set(power - differential);
  }

  public double getRPM() {
    return (topShooterMotor.getEncoder().getVelocity() + bottomShooterMotor.getEncoder().getVelocity()) / 2.0;
  }

  /**
   * Sets the indexer to a given power.
   * @param power The power to set the indexer to.
   */
  public void setIndexer(double power) {
    indexMotor.set(power);
  }

  public boolean pieceSensorActive() {
    return pieceSensor.getRed() > 1200;
  }

  private void configureShuffleboardData() {
    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab.addDouble("RPM", this::getRPM);
    shooterTab.addBoolean("Piece Sensor Active", this::pieceSensorActive);
    shooterTab.addNumber("Piece Sensor Red", pieceSensor::getRed);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}