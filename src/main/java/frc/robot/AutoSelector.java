package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.subsystems.Swerve;

/*
 * Publishes a network table chooser to smart dashboard to select the autonomous command. 
 * Compose the auto routines here to put in the selector.
 */
public class AutoSelector {
    private LoggedDashboardChooser<Command> autoChooser;
    RobotContainer robotContainer;
    Swerve swerveSubsystem;
    AutoDriveCommands autoDriveCommands;
    
    public AutoSelector(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        autoChooser.addOption("Amp", getAmpAuto());
        autoChooser.addOption("3 Piece", getThreePieceAuto());
        autoChooser.addOption("One Piece!", getItsRealAuto());


        SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());
        swerveSubsystem = robotContainer.swerveSubsystem;
        autoDriveCommands = robotContainer.autoDriveCommands;
    }

    /**
     * @return The selected auto from smart dashboard
     */
    public Command getSelectedCommand() {
        DataLogManager.log("Sending command: " + autoChooser.get().toString());
        return autoChooser.get();
    }

    // Auto definitions

    

    private SequentialCommandGroup getAmpAuto() {
        return new SequentialCommandGroup(
            autoDriveCommands.followPathCommand(PathPlannerPath.fromPathFile("Amp 1")),
            // TODO: add shot to amp
            autoDriveCommands.followPathCommand(PathPlannerPath.fromPathFile("Amp 2")),
            // TODO: add shot to speaker
            autoDriveCommands.followPathCommand(PathPlannerPath.fromPathFile("Amp 3"))
        );
    }

    private SequentialCommandGroup getThreePieceAuto(){
        return new SequentialCommandGroup(
            autoDriveCommands.followPathCommand(PathPlannerPath.fromPathFile("3 Piece 1")),
            // TODO: shot to speaker 
            autoDriveCommands.followPathCommand(PathPlannerPath.fromPathFile("3 Piece 2")),
            // TODO: shot to speaker 
            autoDriveCommands.followPathCommand(PathPlannerPath.fromPathFile("3 Piece 3")),
            // TODO: shot to speaker
            autoDriveCommands.followPathCommand(PathPlannerPath.fromPathFile("3 Piece 4"))
        );
    }

    private SequentialCommandGroup getItsRealAuto(){
        return new SequentialCommandGroup(
            autoDriveCommands.followPathCommand(PathPlannerPath.fromPathFile("One Piece 1")),
            // TODO: shot to speaker 
            autoDriveCommands.followPathCommand(PathPlannerPath.fromPathFile("One Piece 2"))
        ); 
    }
}