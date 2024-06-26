package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AutoDriveBuilder;
import frc.robot.commands.AutoShootBuilder;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeCommand;
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
    Intake intakeSubsystem;

    Command shootCommand;
    AutoDriveBuilder autoDriveBuilder;
    AutoShootBuilder autoShootBuilder;
    
    public AutoSelector(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        autoDriveBuilder = robotContainer.autoDriveBuilder;
        swerveSubsystem = robotContainer.swerveSubsystem;
        shooterSubsystem = robotContainer.shooterSubsystem;
        pivotSubsystem = robotContainer.pivotSubsystem;
        intakeSubsystem = robotContainer.intakeSubsystem;
        autoShootBuilder = robotContainer.autoShootBuilder;

        alternativeSelector = new LoggedDashboardChooser<>("Alternative Auto Chooser");
        alternativeSelector.addDefaultOption("Regular Side Endpoint", "a");
        alternativeSelector.addOption("Under Stage Endpoint", "b");

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
        autoChooser.addDefaultOption("Do Nothing", getDoNothingAuto());
        autoChooser.addOption("Amp [Amp]", getAmpAuto());
        autoChooser.addOption("3 Piece [Center]", getFourPieceAuto());
        autoChooser.addOption("Source [Source]", getItsRealAuto());
        autoChooser.addOption("Single Pringle", getSinglePreLoadAmpSide());
        autoChooser.addOption("Driveback 3 Piece [Center]", driveback3Piece());
        autoChooser.addOption("Disruption [Source]", getDisruptionAuto());

        SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());
    }

    /**
     * @return The selected auto from smart dashboard
     */
    public Command getSelectedCommand() {
        DataLogManager.log("Sending command: " + autoChooser.get().toString());
        return autoChooser.get();
    }

    private boolean isBlue() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        } else {
            return false; // true for default to blue alliance
        }
    }

    // Auto definitions

    private SequentialCommandGroup getDoNothingAuto() {
        return new SequentialCommandGroup(
            new ConditionalCommand(
                new SequentialCommandGroup( // blue starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(180))))
                ),
                new SequentialCommandGroup( // red starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0))))
                ),
                this::isBlue
            )
        );
    }

    /**
     * Shot into the amp + single note from the ground into the speaker
     */
    private SequentialCommandGroup getAmpAuto() {
        return new SequentialCommandGroup(
            // setup odometry
            new ConditionalCommand(
                new SequentialCommandGroup( // blue starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0.44, 7.35, Rotation2d.fromDegrees(0))))
                ),
                new SequentialCommandGroup( // red starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(16.05, 7.35, Rotation2d.fromDegrees(180))))
                ),
                this::isBlue
            ),
            // drive to amp
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Amp 1")),
            // shoot in amp
            new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> pivotSubsystem.getAngle().getRotations() > 0.5),
                new SetPivotCommand(Rotation2d.fromRotations(0.51), pivotSubsystem)
            ),
            new ShootCommand(shooterSubsystem, pivotSubsystem),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive & pick up piece
            new ParallelDeadlineGroup(
                autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Amp 2")),
                new IntakeCommand(intakeSubsystem, shooterSubsystem)
            ),
            new IndexCommand(shooterSubsystem),
            // shoot into speaker
            autoShootBuilder.getShotSequence(),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive to end location
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Amp 3a"))
        );
    }

    /**
     * 4 shots into the speaker (3 ground + 1 preload)
     */
    private SequentialCommandGroup getFourPieceAuto(){
        return new SequentialCommandGroup(
            // setup odometry
            new ConditionalCommand(
                new SequentialCommandGroup( // blue starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(1.35, 5.55, Rotation2d.fromDegrees(180))))
                ),
                new SequentialCommandGroup( // red starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(15.2, 5.55, Rotation2d.fromDegrees(0))))
                ),
                this::isBlue
            ),
            // shoot speaker
            autoShootBuilder.getShotSequence(),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive & pick up to first piece
            new ParallelDeadlineGroup(
                autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("4 Piece 1")),
                new IntakeCommand(intakeSubsystem, shooterSubsystem)
            ),
            new IndexCommand(shooterSubsystem),
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("4 Piece 1b")),
            // shoot into speaker
            autoShootBuilder.getShotSequence(),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive & pick up to second piece
            new ParallelDeadlineGroup(
                autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("4 Piece 2")),
                new IntakeCommand(intakeSubsystem, shooterSubsystem)
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.1),
                new IntakeCommand(intakeSubsystem, shooterSubsystem)
            ),
            new IndexCommand(shooterSubsystem),
            // shoot into speaker
            autoShootBuilder.getShotSequence(),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive & pick up to third piece
            new ParallelDeadlineGroup(
                autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("4 Piece 3")),
                new IntakeCommand(intakeSubsystem, shooterSubsystem)
            ),
            new IndexCommand(shooterSubsystem),
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("4 Piece 4")),
            // shoot into speaker
            autoShootBuilder.getShotSequence(),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem)
            // drive to end location
        );
    }

    /**
     * Off to the side auto, single pre-loaded + centerline
     */
    private SequentialCommandGroup getItsRealAuto(){
        return new SequentialCommandGroup(
            // setup odometry
            new ConditionalCommand(
                new SequentialCommandGroup( // blue starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0.50, 2.10, Rotation2d.fromDegrees(0))))
                ),
                new SequentialCommandGroup( // red starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(16.05, 2.10, Rotation2d.fromDegrees(180))))
                ),
                this::isBlue
            ),
            // drive to shot location
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("One Piece 1")),
            // shoot into speaker
            autoShootBuilder.getShotSequence(),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive & pick up centerline piece
            new ParallelDeadlineGroup(
                autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("One Piece 2")),
                new IntakeCommand(intakeSubsystem, shooterSubsystem)
            ),
            new IndexCommand(shooterSubsystem),
            // drive to shot location
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("One Piece 3")),
            // shoot into speaker
            autoShootBuilder.getShotSequence(),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem)
        ); 
    }

    /**
     * Just shoot pre-loaded shot into speaker on the amp side of subwoofer
     * @return
     */
    private SequentialCommandGroup getSinglePreLoadAmpSide() {
        return new SequentialCommandGroup(
            // setup odometry
            new ConditionalCommand(
                new SequentialCommandGroup( // blue starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(240))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(240))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0.70, 6.70, Rotation2d.fromDegrees(240))))
                ),
                new SequentialCommandGroup( // red starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(300))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(120))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(15.90, 6.70, Rotation2d.fromDegrees(300))))
                ),
                this::isBlue
            ),
            // shoot speaker
            new SetPivotCommand(Rotation2d.fromRotations(0.31), pivotSubsystem),
            new ShootCommand(shooterSubsystem, pivotSubsystem),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem)
        );
    }

    private SequentialCommandGroup driveback3Piece() {
        return new SequentialCommandGroup(
            // setup odometry
            new ConditionalCommand(
                new SequentialCommandGroup( // blue starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(1.35, 5.55, Rotation2d.fromDegrees(180))))
                ),
                new SequentialCommandGroup( // red starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(15.2, 5.55, Rotation2d.fromDegrees(0))))
                ),
                this::isBlue
            ),
            // shoot speaker
            autoShootBuilder.getShotSequence(),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive & pick up to first piece
            new ParallelDeadlineGroup(
                autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Driveback 3 Piece 1")),
                new IntakeCommand(intakeSubsystem, shooterSubsystem)
            ),
            new IndexCommand(shooterSubsystem),
            // drive back to speaker
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Driveback 3 Piece 2")),
            // shoot speaker
            autoShootBuilder.getShotSequence(),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem),
            // drive & pick up to first piece
            new ParallelDeadlineGroup(
                autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Driveback 3 Piece 3")),
                new IntakeCommand(intakeSubsystem, shooterSubsystem)
            ),
            new IndexCommand(shooterSubsystem),
            // drive back to speaker
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Driveback 3 Piece 4")),
            // shoot speaker
            autoShootBuilder.getShotSequence(),
            new SetPivotCommand(pivotSubsystem.getDefaultAngle(), pivotSubsystem)
        );
    }

    /**
     * Disrupt, no shots made
     */
    private SequentialCommandGroup getDisruptionAuto(){
        return new SequentialCommandGroup(
            // setup odometry
            new ConditionalCommand(
                new SequentialCommandGroup( // blue starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0.50, 2.10, Rotation2d.fromDegrees(0))))
                ),
                new SequentialCommandGroup( // red starting point
                    new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(180))),
                    new InstantCommand(() -> IMU.getInstance().setOffset(Rotation2d.fromDegrees(0))),
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(16.05, 2.10, Rotation2d.fromDegrees(180))))
                ),
                this::isBlue
            ),
            // drive to shot location
            autoDriveBuilder.getFollowPathCommand(PathPlannerPath.fromPathFile("Disruption"))
        ); 
    }
}