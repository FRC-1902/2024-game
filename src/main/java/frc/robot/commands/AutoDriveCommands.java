package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Swerve;

public class AutoDriveCommands {
    private Swerve swerveSubsystem;
    private IMU imu;

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

        imu = IMU.getInstance();
    }

    /**
     * Path to pose
     * <b>NOTE: path rotation not fully enforced, only when in transit, you should probably rotate before this command</b>
     * @param targetPose
     * @return command to schedule to move to point
     */
    public Command getPathFindingCommand(Pose2d targetPose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            Constants.Swerve.MAX_SPEED, Constants.Swerve.MAX_ACCELERATION,
            Constants.Swerve.MAX_ANGULAR_VELOCITY, Constants.Swerve.MAX_ANGULAR_ACCELERATION);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            targetPose,
            constraints
        );
    }

    public Command followPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    /**
     * @return command to turn to a specific field-centric angle relative to the blue origin
     */
    public Command turnCommand(Rotation2d rot) {
        return new TurnCommand(rot);
    }

    /**
     * Turn to a specific field-centric angle relative to the blue origin
     */
    private class TurnCommand extends Command {
        private PIDController turnPID;
        private Rotation2d targetRot;

        public TurnCommand(Rotation2d targetRot) {
            this.targetRot = targetRot;

            turnPID = new PIDController(Constants.AutoConstants.TURN_KP, Constants.AutoConstants.TURN_KI, Constants.AutoConstants.TURN_KD);
            turnPID.setTolerance(Constants.AutoConstants.TURN_TOLERANCE.getRotations());
            turnPID.enableContinuousInput(0, 1);
            turnPID.setIZone(1);

            addRequirements(swerveSubsystem);
        }

        @Override
        public void initialize() {
            turnPID.reset();
            turnPID.setSetpoint(targetRot.getRotations());
        }

        @Override
        public void execute() {
            swerveSubsystem.drive(
                new Translation2d(0, 0), 
                turnPID.calculate(imu.getFieldHeading().getRotations()), 
                false, 
                false
            );
        }

        @Override
        public boolean isFinished() {
            return turnPID.atSetpoint();
        }

        @Override
        public void end(boolean interrupted) {
            swerveSubsystem.drive(
                new Translation2d(0, 0), 
                0, 
                false, 
                false
            );
        }
    }

    public Command getExampleAutonomousCommand() {
        return new PathPlannerAuto("Example Auto"); // XXX: no example auto present in this project yet
    }
}
