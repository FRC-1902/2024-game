// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

public class LEDCommand extends Command {
  /** Creates a new DigitalCommand. */
  LED led; 
  Shooter shooter; 
  
  public LEDCommand(Shooter shooter, LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led; 
    this.shooter = shooter;   
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setLED(true); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.topPieceSensorActive() || shooter.midPieceSensorActive()){
      led.setLED(false);
    }
    else{
      led.setLED(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     led.setLED(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



