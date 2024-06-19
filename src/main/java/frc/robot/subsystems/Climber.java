// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  LoggedDashboardBoolean leftBottomSwitchLogged;
  LoggedDashboardBoolean leftTopSwitchLogged;
  LoggedDashboardBoolean rightBottomSwitchLogged;
  LoggedDashboardBoolean rightTopSwitchLogged;

  LoggedDashboardChooser<Boolean> climberDisabledChooser;

  public enum Direction {
    UP, DOWN, STOP
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
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    // normally closed limit switches
    leftBottomSwitch =  new DigitalInput(Constants.Climber.LB_SWITCH_PORT);
    rightBottomSwitch = new DigitalInput(Constants.Climber.RB_SWITCH_PORT); 
    leftTopSwitch = new DigitalInput(Constants.Climber.LT_SWITCH_PORT); 
    rightTopSwitch = new DigitalInput(Constants.Climber.RT_SWITCH_PORT);

    targetDirection = Direction.STOP;
    configureNTData();
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

  private void configureNTData() {
    leftBottomSwitchLogged = new LoggedDashboardBoolean("Climber/Left Bottom Switch");
    leftTopSwitchLogged = new LoggedDashboardBoolean("Climber/Left Top Switch");
    rightBottomSwitchLogged = new LoggedDashboardBoolean("Climber/Right Bottom Switch");
    rightTopSwitchLogged = new LoggedDashboardBoolean("Climber/Right Top Switch");

    climberDisabledChooser = new LoggedDashboardChooser<>("Climber/Climber Disable");
    climberDisabledChooser.addDefaultOption("Normal Climbing", false);
    climberDisabledChooser.addOption("STOP CLIMBER", true);
  }

  private void putNTData() {
    leftBottomSwitchLogged.set(!leftBottomSwitch.get());
    leftTopSwitchLogged.set(!leftTopSwitch.get());
    rightBottomSwitchLogged.set(!rightBottomSwitch.get());
    rightTopSwitchLogged.set(!rightTopSwitch.get());
  }

  @Override
  public void periodic() {
    putNTData();

    if (Boolean.TRUE.equals(climberDisabledChooser.get())) {
      leftMotor.set(0.0);
      rightMotor.set(0.0);
      return;
    }
    
    switch (targetDirection) {
      case UP:
        if (leftTopSwitch.get()) {
          leftMotor.set(0.75);
        } else {
          leftMotor.set(0.0);
        }
        if (rightTopSwitch.get()) {
          rightMotor.set(0.75);
        } else {
          rightMotor.set(0.0);
        }
        break;
      case DOWN:
        if (leftBottomSwitch.get()) {
          leftMotor.set(-0.75);
        } else {
          leftMotor.set(0.0);
        }
        if (rightBottomSwitch.get()) {
          rightMotor.set(-0.75);
        } else {
          rightMotor.set(0.0);
        }
        break;
      case STOP:
        leftMotor.set(0);
        rightMotor.set(0);
        break;
      default:
        break;
    }
  }
}
