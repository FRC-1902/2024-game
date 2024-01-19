// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Controllers.Button;
import frc.robot.subsystems.Controllers.ControllerName;

/** Add your docs here. */
public class RobotContainer {

    Swerve swerveSubsystem;
    Controllers controllers;

    public RobotContainer() {
        swerveSubsystem = new Swerve();
        controllers = Controllers.getInstance();

        swerveSubsystem.setDefaultCommand(new DriveCommand(swerveSubsystem));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        controllers.getTrigger(ControllerName.DRIVE, Button.Y).debounce(0.1)
            .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
    }
}
