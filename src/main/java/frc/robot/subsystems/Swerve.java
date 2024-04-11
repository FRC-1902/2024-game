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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private SwerveDrivePoseEstimator swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private IMU imu;

    private Field2d field;

    PhotonCamera backCamera;
    PhotonCamera frontCamera;

    PhotonPoseEstimator backPhotonPoseEstimator;
    PhotonPoseEstimator frontPhotonPoseEstimator;

    public Swerve() {
        imu = IMU.getInstance();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            imu.getHeading(), 
            getModulePositions(), 
            new Pose2d(0.0, 0.0, imu.getHeading())
        );

        field = new Field2d();

        // Photonvision things
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        
        backCamera = new PhotonCamera("Arducam_OV9281_USB_Camera_Left");
        frontCamera = new PhotonCamera("Arducam_OV9281_USB_Camera_Right");

        backCamera.setDriverMode(false);
        frontCamera.setDriverMode(false);

        Transform3d backRobotToCam  = new Transform3d();// Constants.Swerve.BACK_CAMERA_OFFSET;
        Transform3d frontRobotToCam = new Transform3d();// Constants.Swerve.FRONT_CAMERA_OFFSET;

        backPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera, backRobotToCam);
        frontPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera, frontRobotToCam);

        backPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        frontPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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
        // front vision
        final Optional<EstimatedRobotPose> optionalEstimatedPoseFront = frontPhotonPoseEstimator.update();
        if (optionalEstimatedPoseFront.isPresent()) {
            final EstimatedRobotPose estimatedPose = optionalEstimatedPoseFront.get();
            Pose3d estimatedPoseTransformed = estimatedPose.estimatedPose.plus(new Transform3d(new Translation3d(), new Rotation3d(0,-estimatedPose.estimatedPose.getRotation().getY(),0)));
            estimatedPoseTransformed = estimatedPoseTransformed.plus(Constants.Swerve.FRONT_CAMERA_OFFSET);
            swerveOdometry.addVisionMeasurement(estimatedPoseTransformed.toPose2d(), estimatedPose.timestampSeconds);
            SmartDashboard.putNumber("R X", estimatedPoseTransformed.getTranslation().getX());
            SmartDashboard.putNumber("R Y", estimatedPoseTransformed.getTranslation().getY());
            // SmartDashboard.putNumber("R Z", estimatedPoseTransformed.getTranslation().getZ());
        }
        // back vision
        final Optional<EstimatedRobotPose> optionalEstimatedPoseBack = backPhotonPoseEstimator.update();
        if (optionalEstimatedPoseBack.isPresent()) {
            final EstimatedRobotPose estimatedPose = optionalEstimatedPoseBack.get();
            Pose3d estimatedPoseTransformed = estimatedPose.estimatedPose.plus(new Transform3d(new Translation3d(), new Rotation3d(0,-estimatedPose.estimatedPose.getRotation().getY(), 0)));
            estimatedPoseTransformed = estimatedPoseTransformed.plus(Constants.Swerve.BACK_CAMERA_OFFSET);
            swerveOdometry.addVisionMeasurement(
                new Pose2d(
                    estimatedPoseTransformed.toPose2d().getTranslation(), 
                    estimatedPoseTransformed.toPose2d().getRotation().plus(Rotation2d.fromDegrees(180))), 
                estimatedPose.timestampSeconds
            );
            SmartDashboard.putNumber("L X", estimatedPoseTransformed.getTranslation().getX());
            SmartDashboard.putNumber("L Y", estimatedPoseTransformed.getTranslation().getY());
            // SmartDashboard.putNumber("L Z", estimatedPoseTransformed.getTranslation().getZ());
        }

        // process & log odometry
        swerveOdometry.update(imu.getFieldHeading(), getModulePositions());
        Pose2d odometryPose = swerveOdometry.getEstimatedPosition();
        field.setRobotPose(odometryPose);
        Logger.recordOutput("Swerve/Pose", odometryPose);

        SmartDashboard.putNumber("X", odometryPose.getX());
        SmartDashboard.putNumber("Y", odometryPose.getY());
        SmartDashboard.putData("Field", field);
    }
}