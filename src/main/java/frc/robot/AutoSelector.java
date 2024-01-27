package frc.robot;

import java.time.Instant;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class AutoSelector {
    private LoggedDashboardChooser<Command> autoChooser;
    RobotContainer robotContainer;
    Swerve swerveSubsystem;
    
    public AutoSelector(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        autoChooser.addOption("TestAuto1Path", getTestAuto1());
        autoChooser.addOption("TestAuto2Path", getTestAuto2());

        SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());
        swerveSubsystem = robotContainer.swerveSubsystem;
    }

    /**
     * @return The selected auto from smart dashboard
     */
    public Command getSelectedCommand() {
        DataLogManager.log("Sending command: " + autoChooser.get().toString());
        return autoChooser.get();
    }

    // Auto definitions

    private Command getTestAuto1() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(2, 2, Rotation2d.fromDegrees(0)))),
            robotContainer.autoDriveCommands.getPathFindingCommand(new Pose2d(3, 2, Rotation2d.fromDegrees(90)))
        );
    }

    private Command getTestAuto2() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.zeroGyro()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(2, 2, Rotation2d.fromDegrees(0)))),
            robotContainer.autoDriveCommands.followPathCommand(PathPlannerPath.fromPathFile("Example Path"))
        );
    }
}