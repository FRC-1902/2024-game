// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ESPController extends SubsystemBase {
  /** Creates a new ESP controller. */
  DigitalOutput output; 
  
  /**
   * Controlls an ESP32 via digital pin
   * @param LED_PORT The ESP32's digital pin
   */
  public ESPController() {
    output = new DigitalOutput(Constants.LED_PORT); 
  }

  public void setLED(boolean isGreen){
    SmartDashboard.putBoolean("led/activeState", isGreen);
    output.set(isGreen);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
