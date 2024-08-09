// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.AnalogOutput;
import frc.robot.subsystems.Shooter;

public class AnalogCommand extends Command {
  /** Creates a new AnalogCommand. */
  Shooter shooter; 
  AnalogOutput ot1; 
  AnalogOutput ot2; 
  AnalogOutput ot3;
  public AnalogCommand(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    ot1 = new AnalogOutput(5); 
    ot2 = new AnalogOutput(6); 
    ot3 = new AnalogOutput(7); 
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ot1.setVoltage(255);
    ot2.setVoltage(95);
    ot3.setVoltage(31); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.topPieceSensorActive() || shooter.midPieceSensorActive()){
      ot1.setVoltage(0);
      ot2.setVoltage(255);
      ot3.setVoltage(0);
    }
    else{
      ot1.setVoltage(255);
      ot2.setVoltage(95);
      ot3.setVoltage(31);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ot1.setVoltage(255);
    ot2.setVoltage(95);
    ot3.setVoltage(31);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
