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
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private static Swerve instance;
    private SwerveDrivePoseEstimator swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private IMU imu;

    private Swerve() {
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
        Timer.delay(1.0); //TODO: see if I still need this with rev
        resetModulesToAbsolute();

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

    public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
        // Logger.recordOutput("Swerve Desired Module States", swerveModuleStates);
    }

    public void drive(ChassisSpeeds speeds) {
        drive(speeds, false);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    /**
     * @param pose //TODO: finish documenting this
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(imu.getHeading(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        // TODO: fix me
        imu.setOffset(imu.getHeading().minus(imu.getOffset()));
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(imu.getHeading(), getModulePositions());  
        logPeriodic();
    }

    public static Swerve getInstance(){
        if(instance == null){
            instance = new Swerve();
        }
        return instance;
    }
}