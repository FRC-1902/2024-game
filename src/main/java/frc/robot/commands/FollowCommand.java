// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPRamseteController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class FollowCommand extends FollowPathCommand {
  Swerve swerveSubsystem = Swerve.getInstance();

  /** Creates a new FollowCommand. */
  public FollowCommand(PathPlannerPath path) {          
    super(
      path, 
      () -> Swerve.getInstance().getPose(), 
      () -> Swerve.getInstance().getChassisSpeeds(),
      (ChassisSpeeds s) -> Swerve.getInstance().drive(s),
      new PPRamseteController(), // maybe also try PPLTVController
      new ReplanningConfig(), // default config
      () -> { 
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red; // should flip path if on red alliance
        } else {
          return false;
        }
      }, 
      Swerve.getInstance()
    );
  }
}
