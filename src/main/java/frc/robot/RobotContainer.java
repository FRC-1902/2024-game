// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    public Pivot pivotSubsystem;
    public Climber climberSubsystem;
    Controllers controllers;
    public AutoDriveBuilder autoDriveBuilder;
    public AutoShootBuilder autoShootBuilder;

    Command floorIntakeCommand;
    Command outtakeCommand;
    Command hpIntakeCommand;

    public RobotContainer() {
        swerveSubsystem = new Swerve();
        shooterSubsystem = new Shooter();
        pivotSubsystem = new Pivot(shooterSubsystem);
        climberSubsystem = new Climber();
        controllers = Controllers.getInstance();

        autoDriveBuilder = new AutoDriveBuilder(swerveSubsystem);
        autoShootBuilder = new AutoShootBuilder(autoDriveBuilder, shooterSubsystem, pivotSubsystem, swerveSubsystem);

        floorIntakeCommand = new ParallelCommandGroup(
            new IndexCommand(shooterSubsystem), 
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem)
        );

        hpIntakeCommand = new ParallelCommandGroup(
            new IndexCommand(shooterSubsystem), 
            new SetPivotCommand(Rotation2d.fromRotations(0.370), pivotSubsystem)
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

        // TODO: remove tmp debug code
        // controllers.getTrigger(ControllerName.DRIVE, Button.A).debounce(0.05)
        //     .whileTrue(autoDriveBuilder.getPathFindingCommand(new Pose2d(1.86, 7.8, Rotation2d.fromDegrees(90))));

        // floor intake
        controllers.getTrigger(ControllerName.MANIP, Button.A).debounce(0.05)
            .whileTrue(floorIntakeCommand)
            .onFalse(new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem));
        
        // shoot
        controllers.getTrigger(ControllerName.MANIP, Button.B).debounce(0.05)
            .whileTrue(new ShootCommand(shooterSubsystem, pivotSubsystem));
        
        // auto shoot
        // controllers.getTrigger(ControllerName.MANIP, Button.X).debounce(0.05)
        //     .onTrue(new InstantCommand(() -> autoShootBuilder.startShotSequence()))
        //     .onFalse(new InstantCommand(() -> autoShootBuilder.cancelShotSequence()));
        
        // hp intake
        controllers.getTrigger(ControllerName.MANIP, Button.Y).debounce(0.05)
            .whileTrue(hpIntakeCommand)
            .onFalse(new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem));
        
        // amp lineup
        controllers.getTrigger(ControllerName.MANIP, Button.LB).debounce(0.05)
            .onTrue(new SetPivotCommand(Rotation2d.fromRotations(0.5), pivotSubsystem))
            .onFalse(new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem));
        
        // speaker lineup
        controllers.getTrigger(ControllerName.MANIP, Button.RB).debounce(0.05)
            .onTrue(new SetPivotCommand(Rotation2d.fromDegrees(105), pivotSubsystem))
            .onFalse(new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem));
        
    }

    public void resetPIDs() {
        pivotSubsystem.resetPIDs();
    }
}
