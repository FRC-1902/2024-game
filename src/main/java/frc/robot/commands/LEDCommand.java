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
  LED ledSubsystem; 
  Shooter shooterSubsystem; 
  public LEDCommand(LED ledSubsystem, Shooter shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubsystem = ledSubsystem; 
    this.shooterSubsystem = shooterSubsystem; 
    addRequirements(ledSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterSubsystem.topPieceSensorActive() || shooterSubsystem.midPieceSensorActive()){
      ledSubsystem.setColour(Color.kGreen);
    }
    else{
      ledSubsystem.setColour(new Color("0xFF4000")); 
    }
  }
          
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSubsystem.setColour(new Color("0xFF4000")); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
