package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoDriveCommands {
    private Swerve swerveSubsystem;

    public AutoDriveCommands(Swerve swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        AutoBuilder.configureHolonomic(
            swerveSubsystem::getPose, 
            swerveSubsystem::resetOdometry, 
            swerveSubsystem::getChassisSpeeds, 
            swerveSubsystem::drive, 
            new HolonomicPathFollowerConfig(
                Constants.AutoConstants.TRANSLATION_PID,
                Constants.AutoConstants.ROTATION_PID, 
                Constants.Swerve.MAX_SPEED, 
                Math.sqrt(Math.pow(Constants.Swerve.WHEEL_BASE / 2.0, 2) + Math.pow(Constants.Swerve.TRACK_WIDTH / 2.0, 2)), // only works like this for square/rectangular swerve
                new ReplanningConfig()
            ), 
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            swerveSubsystem
        );
    }

    public Command getPathFindingCommand(Pose2d targetPose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            Constants.Swerve.MAX_SPEED, Constants.Swerve.MAX_ACCELERATION,
            Constants.Swerve.MAX_ANGULAR_VELOCITY, Constants.Swerve.MAX_ANGULAR_ACCELERATION);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
    }

    public Command getExampleAutonomousCommand() {
        return new PathPlannerAuto("Example Auto"); // XXX: no example auto present in this project yet
    }
}
