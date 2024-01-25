package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.sensors.LimelightHelpers;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private SwerveDrivePoseEstimator swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private IMU imu;

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
         */
        // Timer.delay(1.0); //XXX: may need to be re-added
        // resetModulesToAbsolute();

        swerveOdometry = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            imu.getHeading(), 
            getModulePositions(), 
            new Pose2d(0.0, 0.0, imu.getHeading()) // XXX: starting position on the field
        );
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
        
        setModuleStates(swerveModuleStates);
    }

    private void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
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
        swerveOdometry.resetPosition(imu.getHeading(), getModulePositions(), pose);
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
        // vision odometry // TODO: test me
        // Pose2d limelightEstimate = LimelightHelpers.getBotPose3d("").toPose2d();
        // update odometry if vision position deviates by less than 1 meter from current estimate (as per documentation estimate)
        // if (limelightEstimate.getTranslation().getDistance(swerveOdometry.getEstimatedPosition().getTranslation()) < 1) { XXX: maybe reimplement me
        
        // swerveOdometry.addVisionMeasurement(
        //     LimelightHelpers.getBotPose3d("").toPose2d(), 
        //     Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Capture("")/1000.0) - (LimelightHelpers.getLatency_Pipeline("")/1000.0)
        // ); 

        swerveOdometry.update(imu.getHeading(), getModulePositions());
        
        logPeriodic();
    }
}