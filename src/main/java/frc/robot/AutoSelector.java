package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AutoDriveBuilder;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.SetPivotCommand;
import frc.robot.commands.ShootCommand;
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
            // setup odometry
            new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(0))),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0.5, 7.75, Rotation2d.fromDegrees(0)))),
            // drive to amp
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Amp 1")),
            // shoot in amp
            new SetPivotCommand(Rotation2d.fromRotations(0.51), pivotSubsystem),
            new ShootCommand(shooterSubsystem, pivotSubsystem),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive & pick up piece
            new ParallelCommandGroup(
                autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Amp 2")),
                new IndexCommand(shooterSubsystem)
            ),
            // shoot into speaker // TODO: debug from here, up until here it is tested
            new InstantCommand(() -> shooterSubsystem.setFlywheel(1, 0)), // pre-rev
            new SetPivotCommand(Rotation2d.fromRotations(0.3), pivotSubsystem), // TODO: find good angle
            new ShootCommand(shooterSubsystem, pivotSubsystem),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive to end location
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Amp 3" + getAlternativeAutoString()))
        );
    }

    /**
     * 4 shots into the speaker (3 ground + 1 preload) // TODO: debug from here
     */
    private SequentialCommandGroup getThreePieceAuto(){
        return new SequentialCommandGroup(
            // setup odometry
            // TODO: setup me
            // shoot speaker
            new InstantCommand(() -> shooterSubsystem.setFlywheel(1, 0)), // pre-rev
            new SetPivotCommand(Rotation2d.fromRotations(0.3), pivotSubsystem), // TODO: find good angle
            new ShootCommand(shooterSubsystem, pivotSubsystem),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive & pick up to first piece
            new ParallelCommandGroup(
                autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("3 Piece 1")),
                new IndexCommand(shooterSubsystem)
            ),
            // shoot into speaker
            new InstantCommand(() -> shooterSubsystem.setFlywheel(1, 0)), // pre-rev
            new SetPivotCommand(Rotation2d.fromRotations(0.3), pivotSubsystem), // TODO: find good angle
            new ShootCommand(shooterSubsystem, pivotSubsystem),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive & pick up to second piece
            new ParallelCommandGroup(
                autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("3 Piece 2")),
                new IndexCommand(shooterSubsystem)
            ),
            // shoot into speaker
            new InstantCommand(() -> shooterSubsystem.setFlywheel(1, 0)), // pre-rev
            new SetPivotCommand(Rotation2d.fromRotations(0.3), pivotSubsystem), // TODO: find good angle
            new ShootCommand(shooterSubsystem, pivotSubsystem),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive & pick up to third piece
            new ParallelCommandGroup(
                autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("3 Piece 3")),
                new IndexCommand(shooterSubsystem)
            ),
            // shoot into speaker
            new InstantCommand(() -> shooterSubsystem.setFlywheel(1, 0)), // pre-rev
            new SetPivotCommand(Rotation2d.fromRotations(0.3), pivotSubsystem), // TODO: find good angle
            new ShootCommand(shooterSubsystem, pivotSubsystem),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive to end location
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("3 Piece 4" + getAlternativeAutoString()))
        );
    }

    /**
     * Single pre-loaded // TODO: debug from here
     */
    private SequentialCommandGroup getItsRealAuto(){
        return new SequentialCommandGroup(
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("One Piece 1")),
            // TODO: shot to speaker 
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("One Piece 2" + getAlternativeAutoString()))
        ); 
    }
}