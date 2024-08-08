// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDriveBuilder;
import frc.robot.commands.AutoShootBuilder;
import frc.robot.subsystems.Climber;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.SetPivotCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Controllers.Button;
import frc.robot.subsystems.Controllers.ControllerName;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LED;

/*
 * Instantiates all robot subsystems and button commands.
*/
public class RobotContainer {

    Swerve swerveSubsystem;
    Shooter shooterSubsystem;
    public Pivot pivotSubsystem;
    public Climber climberSubsystem;
    public Intake intakeSubsystem;
    Controllers controllers;
    public AutoDriveBuilder autoDriveBuilder;
    public AutoShootBuilder autoShootBuilder;
    public LED ledSubsystem; 
    Command floorIntakeCommand;
    Command outtakeCommand;
    Command hpIntakeCommand;
    Command ledCommand; 
    Command testCommand; 
    

    public RobotContainer() {
        swerveSubsystem = new Swerve();
        shooterSubsystem = new Shooter();
        pivotSubsystem = new Pivot(shooterSubsystem);
        climberSubsystem = new Climber();
        intakeSubsystem = new Intake();
        ledSubsystem = new LED(); 


        controllers = Controllers.getInstance();
        
        ledCommand = new LEDCommand(ledSubsystem, shooterSubsystem); 
        autoDriveBuilder = new AutoDriveBuilder(swerveSubsystem);
        autoShootBuilder = new AutoShootBuilder(autoDriveBuilder, shooterSubsystem, pivotSubsystem, swerveSubsystem);

        floorIntakeCommand = new SequentialCommandGroup(
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            new IntakeCommand(intakeSubsystem, shooterSubsystem),
            new IndexCommand(shooterSubsystem)
        );

        hpIntakeCommand = new ParallelCommandGroup(
            new SetPivotCommand(Rotation2d.fromRotations(0.370), pivotSubsystem),
            new SequentialCommandGroup(
                new IntakeCommand(intakeSubsystem, shooterSubsystem), 
                new IndexCommand(shooterSubsystem)
            )
        );

        swerveSubsystem.setDefaultCommand(new DriveCommand(swerveSubsystem));
        new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem).schedule();
        climberSubsystem.setDefaultCommand(new ClimbCommand(climberSubsystem));
        ledSubsystem.setDefaultCommand(ledCommand);

        configureButtonBindings();
    }   


    /**
     * See <a href="https://docs.google.com/spreadsheets/d/1wMP4YpzC1QxRhvHqJ1PFmJ7Ox0EeoEuYVAkCYsLmFI0/edit?usp=sharing">Button Map</a> for button bindings
     */
    private void configureButtonBindings() {
        /* -------- drive code -------- */

        controllers.getTrigger(ControllerName.DRIVE, Button.B).onTrue(ledSubsystem.testCommand(Color.kRed)); 
        
        // reset driver field-centric gyro offset
        controllers.getTrigger(ControllerName.DRIVE, Button.Y).debounce(0.05)
            .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        // charge at amp
        controllers.getTrigger(ControllerName.DRIVE, Button.X).debounce(0.05)
            .whileTrue(new ConditionalCommand(
                autoDriveBuilder.getPathFindingCommand(new Pose2d(1.86, 7.6, Rotation2d.fromDegrees(90))), // blue amp
                autoDriveBuilder.getPathFindingCommand(new Pose2d(14.70, 7.6, Rotation2d.fromDegrees(90))), // red amp
                this::isBlue
            ));
        // charge at speaker
        controllers.getTrigger(ControllerName.DRIVE, Button.A).debounce(0.05)
            .whileTrue(new ConditionalCommand(
                autoDriveBuilder.getPathFindingCommand(new Pose2d(1.35, 5.55, Rotation2d.fromDegrees(180))), // blue speaker
                autoDriveBuilder.getPathFindingCommand(new Pose2d(15.2, 5.55, Rotation2d.fromDegrees(0))), // red speaker
                this::isBlue
            ));


        /* -------- manip code -------- */

        // auto shoot
        controllers.getTrigger(ControllerName.MANIP, Button.RS).debounce(0.05)
            .onTrue(new InstantCommand(autoShootBuilder::startShotSequence))
            .onFalse(new InstantCommand(autoShootBuilder::cancelShotSequence));

        // auto shoot
        controllers.getTrigger(ControllerName.MANIP, Button.X).debounce(0.05)
            .onTrue(new InstantCommand(autoShootBuilder::startShotSequence))
            .onFalse(new InstantCommand(autoShootBuilder::cancelShotSequence));

        // outtake
        controllers.getTrigger(ControllerName.MANIP, Button.LS).debounce(0.05)
            .whileTrue(new OuttakeCommand(shooterSubsystem, intakeSubsystem));

        // floor intake
        controllers.getTrigger(ControllerName.MANIP, Button.A).debounce(0.05)
            .whileTrue(floorIntakeCommand)
            .onFalse(new IndexCommand(shooterSubsystem))
            .onFalse(new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem));
        
        // shoot
        controllers.getTrigger(ControllerName.MANIP, Button.B).debounce(0.05)
            .whileTrue(new ShootCommand(shooterSubsystem, pivotSubsystem));
        
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
            .onTrue(new SetPivotCommand(Rotation2d.fromRotations(0.31), pivotSubsystem))
            .onFalse(new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem));
        
    }

    private boolean isBlue() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        } else {
            return false; // true for default to blue alliance
        }
    }

    

    public void resetPIDs() {
        pivotSubsystem.resetPIDs();
    }
}
