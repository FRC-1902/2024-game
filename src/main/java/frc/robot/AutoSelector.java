package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/*
 * Publishes a network table chooser to smart dashboard to select the autonomous command. 
 * Compose the auto routines here to put in the selector.
 */
public class AutoSelector {
    private LoggedDashboardChooser<Command> autoChooser;
    RobotContainer robotContainer;
    Swerve swerveSubsystem;
    Pivot pivotSubsystem;
    Shooter shooterSubsystem;

    Command shootCommand;
    
    public AutoSelector(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        autoChooser.addOption("TestAuto1Path", getTestAuto1());
        autoChooser.addOption("TestAuto2Path", getTestAuto2());

        SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());
        swerveSubsystem = robotContainer.swerveSubsystem;
        shooterSubsystem = robotContainer.shooterSubsystem;
        pivotSubsystem = robotContainer.pivotSubsystem;

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

    // Auto definitions

    private Command getTestAuto1() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> IMU.getInstance().setFieldOffset(Rotation2d.fromDegrees(180))),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(1.45, 5.525, Rotation2d.fromDegrees(180))))
            // shootCommand
        );
    }

    private Command getTestAuto2() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.zeroGyro()),
            robotContainer.autoDriveBuilder.getTurnCommand(Rotation2d.fromDegrees(90))
        );
    }
}