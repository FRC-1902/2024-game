// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.AutoDriveBuilder;
import frc.robot.commands.AutoShootBuilder;
import frc.robot.subsystems.Climber;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SetPivotCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.Controllers.Button;
import frc.robot.subsystems.Controllers.ControllerName;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/*
 * Instantiates all robot subsystems and button commands.
*/
public class RobotContainer {

    Swerve swerveSubsystem;
    Shooter shooterSubsystem;
    Pivot pivotSubsystem;
    Climber climberSubsystem;
    Controllers controllers;
    public AutoDriveBuilder autoDriveBuilder;
    public AutoShootBuilder autoShootBuilder;

    Command intakeCommand, outtakeCommand;

    public RobotContainer() {
        swerveSubsystem = new Swerve();
        shooterSubsystem = new Shooter();
        pivotSubsystem = new Pivot(shooterSubsystem);
        climberSubsystem = new Climber();
        controllers = Controllers.getInstance();

        autoDriveBuilder = new AutoDriveBuilder(swerveSubsystem);
        autoShootBuilder = new AutoShootBuilder(autoDriveBuilder, shooterSubsystem, pivotSubsystem, swerveSubsystem);

        intakeCommand = new ParallelCommandGroup(
            new IndexCommand(shooterSubsystem), 
            new SetPivotCommand(Rotation2d.fromRotations(0.17), pivotSubsystem)
        );

        swerveSubsystem.setDefaultCommand(new DriveCommand(swerveSubsystem));
        new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem).schedule();
        climberSubsystem.setDefaultCommand(new ClimbCommand(climberSubsystem));

        configureButtonBindings();
    }   


    /**
     * See <a href="https://docs.google.com/spreadsheets/d/1wMP4YpzC1QxRhvHqJ1PFmJ7Ox0EeoEuYVAkCYsLmFI0/edit?usp=sharing">Button Map</a> for button bindings
     */
    private void configureButtonBindings() {
        /* -------- drive code -------- */

        controllers.getTrigger(ControllerName.DRIVE, Button.Y).debounce(0.05)
            .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));


        /* -------- manip code -------- */

        // outtake
        controllers.getTrigger(ControllerName.MANIP, Button.LS).debounce(0.05)
            .whileTrue(new OuttakeCommand(shooterSubsystem));

        // intake
        controllers.getTrigger(ControllerName.MANIP, Button.A).debounce(0.05)
            .whileTrue(intakeCommand)
            .onFalse(new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem));

        // shoot
        controllers.getTrigger(ControllerName.MANIP, Button.B).debounce(0.05)
            .whileTrue(new ShootCommand(shooterSubsystem));
        
        // amp lineup // TODO: test me
        controllers.getTrigger(ControllerName.MANIP, Button.LB).debounce(0.05)
            .onTrue(new SetPivotCommand(Rotation2d.fromRotations(0.3), pivotSubsystem))
            .onFalse(new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem));
        
        // speaker lineup
        controllers.getTrigger(ControllerName.MANIP, Button.RB).debounce(0.05)
            .onTrue(new SetPivotCommand(Rotation2d.fromDegrees(111), pivotSubsystem))
            .onFalse(new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem));
    }

    public void resetPIDs() {
        pivotSubsystem.resetPIDs();
    }
}
