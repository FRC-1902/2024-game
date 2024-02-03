// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoDriveBuilder;
import frc.robot.commands.AutoShootBuilder;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Controllers.Button;
import frc.robot.subsystems.Controllers.ControllerName;

/*
 * Instantiates all robot subsystems and button commands.
*/
public class RobotContainer {

    public Swerve swerveSubsystem;
    Shooter shooterSubsystem;
    Pivot pivotSubsystem;
    Controllers controllers;
    public AutoDriveBuilder autoDriveBuilder;
    AutoShootBuilder autoShootBuilder;

    public RobotContainer() {
        swerveSubsystem = new Swerve();
        shooterSubsystem = new Shooter();
        pivotSubsystem = new Pivot();
        controllers = Controllers.getInstance();

        autoDriveBuilder = new AutoDriveBuilder(swerveSubsystem);
        autoShootBuilder = new AutoShootBuilder(autoDriveBuilder, shooterSubsystem, pivotSubsystem, swerveSubsystem);

        swerveSubsystem.setDefaultCommand(new DriveCommand(swerveSubsystem));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        controllers.getTrigger(ControllerName.DRIVE, Button.Y).debounce(0.1)
            .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        controllers.getTrigger(ControllerName.MANIP, Button.RB).debounce(0.1)
            .onTrue(new InstantCommand(autoShootBuilder::startShotSequence))
            .onFalse(new InstantCommand(autoShootBuilder::cancelShotSequence));

        controllers.getTrigger(ControllerName.MANIP, Button.B).debounce(0.1)
            .onTrue(new ShootCommand(shooterSubsystem));
    }
}
