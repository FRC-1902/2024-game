package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.sensors.LimelightHelpers;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private SwerveDrivePoseEstimator swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private IMU imu;

    private Field2d field;

    PhotonPoseEstimator leftPhotonPoseEstimator, rightPhotonPoseEstimator;

    public Swerve() {
        imu = IMU.getInstance();
        zeroGyro();
        

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         * Commentented out, but may need to be re-added if issue comes up
         */
        // Timer.delay(1.0);
        // resetModulesToAbsolute();

        swerveOdometry = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            imu.getHeading(), 
            getModulePositions(), 
            new Pose2d(0.0, 0.0, imu.getHeading()) // XXX: starting position on the field
        );

        field = new Field2d();

        // Photonvision things
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        
        PhotonCamera leftCamera = new PhotonCamera("Arducam_OV9281_USB_Camera_Left");
        PhotonCamera rightCamera = new PhotonCamera("Arducam_OV9281_USB_Camera_Right");

        leftCamera.setDriverMode(false);
        rightCamera.setDriverMode(false);

        Transform3d leftRobotToCam = Constants.Swerve.LEFT_CAMERA_OFFSET;
        Transform3d rightRobotToCam = Constants.Swerve.RIGHT_CAMERA_OFFSET;

        leftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamera, leftRobotToCam);
        rightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamera, rightRobotToCam);

        leftPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        rightPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    private void logPeriodic() {
        Logger.recordOutput("Swerve Estimated Pose", swerveOdometry.getEstimatedPosition());
    }

    /**
     * Drive the swerve drive by inputing velocities, and specifying control modes
     * @param translation X and Y velocities as Translation2D object (a velocity vector)
     * @param rotation angular velocity
     * @param fieldRelative set to drive relative to the robot or relative to the field
     * @param isOpenLoop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds speeds = 
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    imu.getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation);
        drive(speeds, isOpenLoop);
    }

    /**
     * Drives with closed-loop control, useful for ChassisSpeeds consumers in auto
     * @param speeds ChassisSpeeds
     */
    public void drive(ChassisSpeeds speeds) {
        drive(speeds, false);
    }

    /**
     * @param speeds ChassisSpeeds
     * @param isOpenLoop open or closed loop control
     */
    public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], isOpenLoop);
        }
    }    

    /**
     * Get estimated position from odometry
     * @return
     */
    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    /**
     * @param pose estimated position to reset to
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(imu.getFieldHeading(), getModulePositions(), pose);
    }

    /**
     * Gets current SwerveModuleStates from all swerve modules
     * @return SwerveModuleState[]
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    /**
     * Gets current SwerveModulePositions from all swerve modules
     * @return SwerveModulePositions[]
     */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Gets current ChassisSpeeds, derived from robot-relative movement
     */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void zeroGyro(){
        imu.setOffset(imu.getOffset().minus(imu.getHeading()));
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }


    @Override
    public void periodic(){
        final Optional<EstimatedRobotPose> optionalEstimatedPoseRight = rightPhotonPoseEstimator.update();
        if (optionalEstimatedPoseRight.isPresent()) {
            final EstimatedRobotPose estimatedPose = optionalEstimatedPoseRight.get();          
            swerveOdometry.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
            // SmartDashboard.putNumber("R X", estimatedPose.estimatedPose.getTranslation().getX());
            // SmartDashboard.putNumber("R Y", estimatedPose.estimatedPose.getTranslation().getY());
        }

        final Optional<EstimatedRobotPose> optionalEstimatedPoseLeft = leftPhotonPoseEstimator.update();
        if (optionalEstimatedPoseLeft.isPresent()) {
            final EstimatedRobotPose estimatedPose = optionalEstimatedPoseLeft.get();      
            swerveOdometry.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
            // SmartDashboard.putNumber("L X", estimatedPose.estimatedPose.getTranslation().getX());
            // SmartDashboard.putNumber("L Y", estimatedPose.estimatedPose.getTranslation().getY());
        }

        

        swerveOdometry.update(imu.getFieldHeading(), getModulePositions());
        field.setRobotPose(swerveOdometry.getEstimatedPosition());
        
        SmartDashboard.putData("Field", field);
        logPeriodic();
    }
}