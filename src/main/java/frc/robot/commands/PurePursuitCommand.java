package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Swerve;

public class PurePursuitCommand extends Command{
    private IMU imu;
    private Swerve swerveSubsystem;

    private PIDController anglePID;
    private int countAtSetpoint;

    private PathPlannerTrajectory trajectory;
    private List<PathPoint> waypoints;
    private ArrayList<Pose2d> waypointsPoses;
    private Pose2d startingPose;

    private static final String LOG_KEY = "Pathing";
    
    /**
     * Pathing command factory
     * @param waypoints waypoints path to follow
     */
    public PurePursuitCommand(PathPlannerPath path){
        this.trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds());
        this.waypoints = path.getAllPathPoints();

        imu = IMU.getInstance();
        swerveSubsystem = Swerve.getInstance();

        anglePID = new PIDController(Constants.AutoConstants.ANGLE_KP, Constants.AutoConstants.ANGLE_KI, Constants.AutoConstants.ANGLE_KD);
        anglePID.setTolerance(Constants.AutoConstants.TARGET_ANGLE_DELTA);
        
        waypointsPoses = new ArrayList<>(waypoints.size());

        // initialize waypointsPoses and fill in rotations without null target rotations
        for (int i = 0; i < waypoints.size(); i++) {
            Rotation2d rotation = null;
            if (waypoints.get(i).holonomicRotation == null) {
                for (int j = i; j < waypoints.size(); j++) {
                    if(waypoints.get(j).holonomicRotation != null){
                        rotation = waypoints.get(j).holonomicRotation;
                        break;
                    }
                }
            } else {
                rotation = waypoints.get(i).holonomicRotation;
            }
            waypointsPoses.add(new Pose2d(waypoints.get(i).position, rotation));
        }
    }

    /**
     * Call to activate the next waypoint
     */
    @Override
    public void initialize() {
        startingPose = swerveSubsystem.getPose();

        countAtSetpoint = 0;
        
        Logger.recordOutput(LOG_KEY, "Starting Next Path");
    }

    /**
     * Stops the drivetrain
     */
    @Override 
    public void end(boolean wasInterrupted) {
        swerveSubsystem.drive(
            new Translation2d(0.0, 0.0),
            0.0,
            true,
            false
        );

        if (!wasInterrupted) {
            Logger.recordOutput(LOG_KEY, "Finished Queued Path");
        } else {
            Logger.recordOutput(LOG_KEY, "Path Was Interrupted");
        }
    }

    @Override
    public boolean isFinished() {
        double endDistance = getEndpoint().position.getDistance(
            getLocalPose().getTranslation()
        );

        if(endDistance < Constants.AutoConstants.TARGET_END_DELTA) {// && anglePID.atSetpoint()) { // TODO: Reimplement when angle pid fixed
            countAtSetpoint += 1;
            if (countAtSetpoint >= Constants.AutoConstants.TARGET_COUNT_AT_SETPIONT) {
                return true;
            }
        } else {
            countAtSetpoint = 0;
        }
        return false;
    }

    /**
     * Call every robot loop until finished
     */
    @Override
    public void execute() {
        // get targets
        double targetVelocity = findClosestVelocity(getLocalPose());
        Rotation2d targetFacingAngle = findClosestPose(getLocalPose()).getRotation();
        Rotation2d currentFacingAngle = imu.getHeading();
        Translation2d lookahead = findLookahead(getLocalPose(), Constants.AutoConstants.SEARCH_DISTANCE);
        if(lookahead == null)
            lookahead = findClosestPose(getLocalPose()).getTranslation();

        // rotation between current and target
        Rotation2d driveAngle = facePoint(getLocalPose().getTranslation(), lookahead);

        // XXX: may need to rexamine later, possibly removing initial acceleration from waypoint generation
        targetVelocity = Math.abs(targetVelocity);
        targetVelocity = Math.max(targetVelocity, Constants.AutoConstants.MIN_VELOCITY);

        // try and hit targets with swerve
        Translation2d driveTarget = new Translation2d(targetVelocity, driveAngle);
        swerveSubsystem.drive(
            driveTarget,
            angleControl(currentFacingAngle, targetFacingAngle),
            true,
            false
        );
    }

    /**
     * Cascading PID angle controller with error limiting to avoid instability
     * @param current
     * @param target
     * @return
     */
    private double angleControl(Rotation2d current, Rotation2d target) {
        Rotation2d error = current.minus(target);
        double adjustedError = Math.abs(error.getDegrees()) < Constants.AutoConstants.ANGLE_ERROR_LIMIT ?
            error.getDegrees() : Constants.AutoConstants.ANGLE_ERROR_LIMIT * Math.signum(error.getDegrees());
        return anglePID.calculate(adjustedError, 0.0);
    }

    /**
     * 
     */
    private Pose2d getLocalPose() {
        return swerveSubsystem.getPose().relativeTo(startingPose);
    }

    /**
     * @param currentPosition estimated current robot position
     * @param searchDistance max meters from waypoint to path to
     * @return last Pose2d in range of search distance, otherwise null
     */
    public Translation2d findLookahead(Pose2d currentPosition, double searchDistance) {
        // loop backward through pose list to get last element that is within range
        for(int i = waypoints.size(); i-- > 0; ) {
            if(currentPosition.getTranslation().getDistance(waypoints.get(i).position) <= searchDistance)
                return waypoints.get(i).position;
        }
        return null;
    }

    /** Find closest Pose2d of waypoint in relation to the current estimated position */
    public Pose2d findClosestPose(Pose2d currentPosition) {
        return currentPosition.nearest(waypointsPoses);
    }

    /** Find closest velocity of waypoint in relation to the current estimated position */
    public double findClosestVelocity(Pose2d currentPosition) {
        double velocity = trajectory.getState(0).velocityMps;
        double minDist = trajectory.getState(0).positionMeters.getDistance(currentPosition.getTranslation());
        
        for(State state : trajectory.getStates()) {
            double dist = state.positionMeters.getDistance(currentPosition.getTranslation());
            if(minDist > dist){
                minDist = dist;
                velocity = state.velocityMps;

                // speed up start pos when within end delta of start position and velocity less than min velocity
                if (trajectory.getState(0).positionMeters.getDistance(state.positionMeters) < Constants.AutoConstants.TARGET_END_DELTA &&
                    velocity < Constants.AutoConstants.MIN_VELOCITY) {
                        velocity = Constants.AutoConstants.MIN_VELOCITY * 2; 
                }
            }
        }

        return velocity;
    }

    /**
     * @param currentPosition
     * @param targetPosition
     * @return Rotation2d to face the target position
     */
    public Rotation2d facePoint(Translation2d currentPosition, Translation2d targetPosition) {
        return new Rotation2d(
            targetPosition.getX() - currentPosition.getX(),
            targetPosition.getY() - currentPosition.getY()
        );
    }

    /**
     * Get the desired final pose from waypoints
     * @return last Pose2d
     */
    public PathPoint getEndpoint() {
        return waypoints.get(waypoints.size()-1);
    }
}