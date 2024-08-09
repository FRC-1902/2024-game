// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalOutput;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;


public class DigitalCommand extends Command {
  /** Creates a new DigitalCommand. */
  LED led; 
  Shooter shooter; 
  DigitalOutput output; 
  public DigitalCommand(LED led, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    this.shooter = shooter;  
    output = new DigitalOutput(5); 
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    output.set(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.topPieceSensorActive() || shooter.midPieceSensorActive()){
      output.set(true); 
    }
    else{
      output.set(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     output.set(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
