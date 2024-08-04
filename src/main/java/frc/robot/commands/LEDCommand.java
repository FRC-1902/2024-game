// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

public class LEDCommand extends Command {
  /** Creates a new HitTheLights. */
  LED ledSubby; 
  Shooter shooterSubby; 
  public LEDCommand(LED ledSubby, Shooter shooterSubby) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubby = ledSubby; 
    this.shooterSubby = shooterSubby; 
    addRequirements(ledSubby, shooterSubby);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubby.setColour(Color.kRed); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterSubby.topPieceSensorActive() && shooterSubby.midPieceSensorActive()){
      ledSubby.setColour(Color.kGreen);
    }
    else{
      ledSubby.setColour(Color.kRed); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSubby.setColour(Color.kRed); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
