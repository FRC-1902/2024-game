// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.AutoDriveBuilder;
import frc.robot.commands.AutoShootBuilder;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SetPivotCommand;
import frc.robot.commands.ShootCommand;
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

    public Swerve swerveSubsystem;
    Shooter shooterSubsystem;
    Pivot pivotSubsystem;
    Controllers controllers;
    public AutoDriveBuilder autoDriveBuilder;
    AutoShootBuilder autoShootBuilder;

    public RobotContainer() {
        swerveSubsystem = new Swerve();
        shooterSubsystem = new Shooter();
        pivotSubsystem = new Pivot(shooterSubsystem);
        controllers = Controllers.getInstance();

        autoDriveBuilder = new AutoDriveBuilder(swerveSubsystem);
        autoShootBuilder = new AutoShootBuilder(autoDriveBuilder, shooterSubsystem, pivotSubsystem, swerveSubsystem);

        swerveSubsystem.setDefaultCommand(new DriveCommand(swerveSubsystem));

        configureButtonBindings();
    }   


    /**
     * See <a href="https://docs.google.com/spreadsheets/d/1wMP4YpzC1QxRhvHqJ1PFmJ7Ox0EeoEuYVAkCYsLmFI0/edit?usp=sharing">Button Map</a> for button bindings
     */
    private void configureButtonBindings() {
        controllers.getTrigger(ControllerName.DRIVE, Button.Y).debounce(0.1)
            .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        controllers.getTrigger(ControllerName.MANIP, Button.A).debounce(0.1)
            .onTrue(new SetPivotCommand(Rotation2d.fromRotations(0.18), pivotSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        controllers.getTrigger(ControllerName.MANIP, Button.B).debounce(0.1)
            .onTrue(new SetPivotCommand(Rotation2d.fromDegrees(180), pivotSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // controllers.getTrigger(ControllerName.MANIP, Button.B).debounce(0.1)
        //     .onTrue(new ShootCommand(shooterSubsystem));
        // 
        // controllers.getTrigger(ControllerName.MANIP, Button.Y).debounce(0.1)
        //     .onTrue(new SetPivotCommand(Constants.Arm.HP_PIVOT_LINEUP, pivotSubsystem))
        //     .onFalse(new SetPivotCommand(Constants.Arm.STOW_PIVOT_LINEUP, pivotSubsystem));

        // controllers.getTrigger(ControllerName.MANIP, Button.LB).debounce(0.1)
        //     .onTrue(new SetPivotCommand(Constants.Arm.AMP_PIVOT_LINEUP, pivotSubsystem))
        //     .onFalse(new SetPivotCommand(Constants.Arm.STOW_PIVOT_LINEUP, pivotSubsystem));
// 
        // controllers.getTrigger(ControllerName.MANIP, Button.RB).debounce(0.1)
        //     .onTrue(new InstantCommand(autoShootBuilder::startShotSequence))
        //     .onFalse(new InstantCommand(autoShootBuilder::cancelShotSequence));


        // XXX: tmp stuffs. remove later

        // controllers.getTrigger(ControllerName.MANIP, Button.A).debounce(0.1)
        //     .onTrue(new InstantCommand(() -> shooterSubsystem.setFlywheel(1, 0.0)))
        //     .onFalse(new InstantCommand(() -> shooterSubsystem.setFlywheel(0.0, 0.0)));
    }

    public void resetPIDs() {
        pivotSubsystem.resetPIDs();
    }
}
