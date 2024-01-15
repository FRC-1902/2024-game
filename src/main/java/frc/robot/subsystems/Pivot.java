// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private static Pivot instance;
  /** Creates a new Pivot. */
  private Pivot() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Pivot getInstance() {
    if (instance == null) {
      instance = new Pivot();
    }
    return instance;
  }
}
