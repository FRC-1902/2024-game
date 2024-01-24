// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;  
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax leftMotor; 
  CANSparkMax rightMotor; 
  DigitalInput leftBottomSwitch; 
  DigitalInput rightBottomSwitch;
  DigitalInput leftTopSwitch;
  DigitalInput rightTopSwitch;

  Direction targetDirection;

  public enum Direction {
    UP, DOWN
  }

  public Climber() {
    leftMotor = new CANSparkMax(Constants.Climber.LEFT_MOTOR_ID, MotorType.kBrushless); 
    rightMotor = new CANSparkMax(Constants.Climber.RIGHT_MOTOR_ID, MotorType.kBrushless);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(leftMotor, Usage.MINIMAL);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(rightMotor, Usage.MINIMAL);
    leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    leftMotor.setSmartCurrentLimit(Constants.Climber.CLIMBER_CURRENT_LIMIT);
    rightMotor.setSmartCurrentLimit(Constants.Climber.CLIMBER_CURRENT_LIMIT);

    // normally closed limit switches
    leftBottomSwitch =  new DigitalInput(Constants.Climber.LB_SWITCH_PORT);
    rightBottomSwitch = new DigitalInput(Constants.Climber.RB_SWITCH_PORT); 
    leftTopSwitch = new DigitalInput(Constants.Climber.LT_SWITCH_PORT); 
    rightTopSwitch = new DigitalInput(Constants.Climber.RT_SWITCH_PORT);

    targetDirection = Direction.DOWN;
  }

  public void setDirection(Direction targetDirection) {
    this.targetDirection = targetDirection;
  }

  public boolean atSetpoint() {
    switch (targetDirection) {
      case UP:
        return !leftTopSwitch.get() && !rightTopSwitch.get();
      case DOWN:
        return !leftBottomSwitch.get() && !rightBottomSwitch.get();
      default:
        return false;
    }
  }

  @Override
  public void periodic() { // XXX: maybe different motor power?
    switch (targetDirection) {
      case UP:
        if (leftTopSwitch.get()) {
          leftMotor.set(0.5);
        } else {
          leftMotor.set(0.0);
        }
        if (rightTopSwitch.get()) {
          rightMotor.set(0.5);
        } else {
          rightMotor.set(0.0);
        }
        break;
      case DOWN:
        if (leftBottomSwitch.get()) {
          leftMotor.set(-0.5);
        } else {
          leftMotor.set(0.0);
        }
        if (rightBottomSwitch.get()) {
          rightMotor.set(-0.5);
        } else {
          rightMotor.set(0.0);
        }
        break;
      default:
        break;
    }
    // This method will be called once per scheduler run
  }
}
