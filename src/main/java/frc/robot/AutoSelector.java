package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

/*
 * Publishes a network table chooser to smart dashboard to select the autonomous command. 
 * Compose the auto routines here to put in the selector.
 */
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
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(2, 2, Rotation2d.fromDegrees(0))))
        );
    }

    private Command getTestAuto2() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.zeroGyro()),
            robotContainer.autoDriveBuilder.getTurnCommand(Rotation2d.fromDegrees(90))
        );
    }
}