// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;


import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;


public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private AddressableLED led; 
  private AddressableLEDBuffer ledBuffer;
   
  
  public LED() {
   
    led = new AddressableLED(Constants.LED_PORT); 
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer); 
    led.start();
  }

  public void setColour(Color color){
    for(int i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setLED(i, color);
    }
    led.setData(ledBuffer);
  }

  

  public Command testCommand(Color color){
   return run(() -> {
      for(int i = 0; i < ledBuffer.getLength(); i++){
        ledBuffer.setLED(i, color);
      }
      led.setData(ledBuffer);
    });
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
