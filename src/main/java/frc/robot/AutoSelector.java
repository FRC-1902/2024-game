package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AutoDriveBuilder;
import frc.robot.subsystems.Swerve;

/*
 * Publishes a network table chooser to smart dashboard to select the autonomous command. 
 * Compose the auto routines here to put in the selector.
 */
public class AutoSelector {
    private LoggedDashboardChooser<Command> autoChooser;
    private LoggedDashboardChooser<String> alternativeSelector;

    RobotContainer robotContainer;
    Swerve swerveSubsystem;
    Pivot pivotSubsystem;
    Shooter shooterSubsystem;

    Command shootCommand;
    AutoDriveBuilder autoDriveBuilder;
    
    public AutoSelector(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        autoDriveBuilder = robotContainer.autoDriveBuilder;
        swerveSubsystem = robotContainer.swerveSubsystem;
        shooterSubsystem = robotContainer.shooterSubsystem;
        pivotSubsystem = robotContainer.pivotSubsystem;

        alternativeSelector = new LoggedDashboardChooser<>("Alternative Auto Chooser");
        alternativeSelector.addDefaultOption("Regular Side Endpoint", "a");
        alternativeSelector.addOption("Under Stage Endpoint", "b");

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        autoChooser.addOption("Amp", getAmpAuto());
        autoChooser.addOption("3 Piece", getThreePieceAuto());
        autoChooser.addOption("One Piece!", getItsRealAuto());

        

        SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());
        

        // shootCommand = new SequentialCommandGroup(
        //     new InstantCommand(robotContainer.autoShootBuilder::startShotSequence),
        //     new WaitUntilCommand(robotContainer.autoShootBuilder::isShotDone)
        // );
    }

    /**
     * @return The selected auto from smart dashboard
     */
    public Command getSelectedCommand() {
        DataLogManager.log("Sending command: " + autoChooser.get().toString());
        return autoChooser.get();
    }  

    // TODO: properly debug the alternative selector later
    private String getAlternativeAutoString() {
        String s = alternativeSelector.get();
        if (s == null) {
            s = "a";
        }
        return s;
    }

    // Auto definitions

    /**
     * Shot into the amp + single note from the ground into the speaker
     */
    private SequentialCommandGroup getAmpAuto() {
        return new SequentialCommandGroup(
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Amp 1")),
            // TODO: add shot to amp
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Amp 2")),
            // TODO: add shot to speaker
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Amp 3" + getAlternativeAutoString()))
        );
    }

    /**
     * 4 shots into the speaker (3 ground + 1 preload)
     */
    private SequentialCommandGroup getThreePieceAuto(){
        return new SequentialCommandGroup(
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("3 Piece 1")),
            // TODO: shot to speaker 
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("3 Piece 2")),
            // TODO: shot to speaker 
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("3 Piece 3")),
            // TODO: shot to speaker
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("3 Piece 4" + getAlternativeAutoString()))
        );
    }

    /**
     * Single pre-loaded
     */
    private SequentialCommandGroup getItsRealAuto(){
        return new SequentialCommandGroup(
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("One Piece 1")),
            // TODO: shot to speaker 
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("One Piece 2" + getAlternativeAutoString()))
        ); 
    }
}