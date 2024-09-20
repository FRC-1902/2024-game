// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ESPController;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDCommand extends Command {
  /** Creates a new DigitalCommand. */
  ESPController esp; 
  Shooter shooter; 
  
  public LEDCommand(Shooter shooter, ESPController esp) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.esp = esp; 
    this.shooter = shooter;   
    addRequirements(esp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    esp.setLED(true); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.topPieceSensorActive() || shooter.midPieceSensorActive()){
      esp.setLED(false);
    }
    else{
      esp.setLED(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     esp.setLED(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



