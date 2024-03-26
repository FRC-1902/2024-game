// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public CANSparkMax intakeMotor;
  /** Creates a new IntakeSubsystem. */
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(intakeMotor, Usage.MINIMAL);
    intakeMotor.setSmartCurrentLimit(Constants.Intake.INTAKE_CURRENT_LIMIT);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setInverted(true);
  }

  public void set(double power) {
    intakeMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
