// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import com.revrobotics.Rev2mDistanceSensor;


public class Shooter extends SubsystemBase {
  private CANSparkMax topShooterMotor;
  private CANSparkMax bottomShooterMotor;
  private CANSparkMax indexMotor;
  private DigitalInput midPieceSensor;
  private Rev2mDistanceSensor topPieceSensor;

  /** Creates a new Shooter. */ 
  public Shooter() {
    // shooter motors setup
    topShooterMotor = new CANSparkMax(Constants.Arm.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    bottomShooterMotor = new CANSparkMax(Constants.Arm.BOTTOM_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(topShooterMotor, Usage.VELOCITY_ONLY);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(bottomShooterMotor, Usage.VELOCITY_ONLY);
    topShooterMotor.enableVoltageCompensation(Constants.Arm.SHOOTER_VOLTAGE_COMPENSATION);
    bottomShooterMotor.enableVoltageCompensation(Constants.Arm.SHOOTER_VOLTAGE_COMPENSATION);
    topShooterMotor.setSmartCurrentLimit(Constants.Arm.SHOOTER_CURRENT_LIMIT);
    bottomShooterMotor.setSmartCurrentLimit(Constants.Arm.SHOOTER_CURRENT_LIMIT);
    topShooterMotor.setIdleMode(IdleMode.kBrake);
    bottomShooterMotor.setIdleMode(IdleMode.kBrake);
    topShooterMotor.setInverted(true);
    bottomShooterMotor.setInverted(true);

    // indexer motors setup
    indexMotor = new CANSparkMax(Constants.Arm.INDEXER_MOTOR_ID, MotorType.kBrushless);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(indexMotor, Usage.MINIMAL);
    indexMotor.setSmartCurrentLimit(Constants.Arm.INDEX_CURRENT_LIMIT);
    indexMotor.setIdleMode(IdleMode.kCoast);

    // piece sensor setup
    midPieceSensor = new DigitalInput(Constants.Arm.IR_PIECE_SENSOR_PORT);
    topPieceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP);
    topPieceSensor.setAutomaticMode(true);
    topPieceSensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kMillimeters);

    configureShuffleboardData();
  }

  /**
   * Get the pivot encoder, currently mounted on one of the spark maxes on the shooter
   * @return
   */
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

  /**
   * Average flywheel motors' RPM.
   * @return shooter rpm
   */
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

  /**
   * Is the 2m dts sensor past some threshold
   * @return isActive
   */
  public boolean topPieceSensorActive() {
    return topPieceSensor.getRange() < 200 && topPieceSensor.getRange() > 1;
  }

  /**
   * Is the ir sensor triggered
   * @return isActive
   */
  public boolean midPieceSensorActive() {
    return midPieceSensor.get();
  }

  private void configureShuffleboardData() {
    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab.addDouble("RPM", this::getRPM);
    shooterTab.addBoolean("Piece Sensor Active", this::topPieceSensorActive);
    shooterTab.addNumber("2m Dts", topPieceSensor::getRange);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
