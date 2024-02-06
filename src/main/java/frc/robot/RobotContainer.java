// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.subsystems.Climber;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Controllers.Button;
import frc.robot.subsystems.Controllers.ControllerName;

/*
 * Instantiates all robot subsystems and button commands.
*/
public class RobotContainer {

    public Swerve swerveSubsystem;
    Climber climberSubsystem;
    Controllers controllers;
    public AutoDriveCommands autoDriveCommands;

    public RobotContainer() {
        swerveSubsystem = new Swerve();
        climberSubsystem = new Climber();
        controllers = Controllers.getInstance();

        autoDriveCommands = new AutoDriveCommands(swerveSubsystem);

        swerveSubsystem.setDefaultCommand(new DriveCommand(swerveSubsystem));
        climberSubsystem.setDefaultCommand(new ClimbCommand(climberSubsystem));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        controllers.getTrigger(ControllerName.DRIVE, Button.Y).debounce(0.1)
            .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
    }
}
