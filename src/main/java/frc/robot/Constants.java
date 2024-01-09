package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public class Constants {
    private Constants() {}
	//Controllers
	public static final int DRIVE_CONTROLLER_PORT = 0;
	public static final int MANIP_CONTROLLER_PORT = 1;

	public static final double STICK_DEADBAND = 0.1;

    public static final class Swerve {
        private Swerve() {}

        public static final boolean GYRO_INVERT = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  
            COTSFalconSwerveConstants.SDSMK3(COTSFalconSwerveConstants.DriveGearRatios.SDSMK3_STANDARD);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(21); 
        public static final double WHEEL_BASE = Units.inchesToMeters(21); 
        public static final double WHEEL_CIRCUMFERENCE = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = chosenModule.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = chosenModule.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERT = chosenModule.canCoderInvert;
		
		/* Swerve Voltage Compensation */
		public static final double VOLTAGE_COMP = 12.0;

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 30;
        public static final int DRIVE_CURRENT_LIMIT = 40;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = chosenModule.angleKP;
        public static final double ANGLE_KI = chosenModule.angleKI;
        public static final double ANGLE_KD = chosenModule.angleKD;
        public static final double ANGLE_KF = chosenModule.angleKF;

        /* Drive Motor PID Values */ //This must be tuned to specific robot
        public static final double DRIVE_KP = 0.25; 
        public static final double DRIVE_KI = 0.001;
        public static final double DRIVE_KD = 0.5;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values */ //This must be tuned to specific robot
        // divide by 12 to convert from volts to 1 to -1 power range
        public static final double DRIVE_KS = 0.126;
        public static final double DRIVE_KV = 2.72;
        public static final double DRIVE_KA = 0.28;

		/* Drive Motor Conversion Factors */
		public static final double DRIVE_CONVERSION_POSITION_FACTOR =
        	(chosenModule.wheelDiameter * Math.PI) / DRIVE_GEAR_RATIO;
    	public static final double DRIVE_CONVERSION_VELOCITY_FACTOR = DRIVE_CONVERSION_POSITION_FACTOR / 60.0;
    	public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

        /* Swerve Profiling Values */
        /* Must be max drivetrain speeds for open loop control */
        /** Meters per Second */ 
        public static final double MAX_SPEED = 3.23; //TODO: test me, Kevin values
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 9.933; //TODO: set me, Kevin values

        /* Neutral (Idle) Modes */
        public static final IdleMode ANGLE_NEURTRAL_MODE = IdleMode.kCoast;
        public static final IdleMode DRIVE_NEUTRAL_MODE = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            private Mod0() {}
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CAN_CODER_ID = 0;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(19);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            private Mod1() {}
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 4;
            public static final int CAN_CODER_ID = 1;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(70);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            private Mod2() {}
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 2;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(230);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            private Mod3() {}
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CAN_CODER_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(155);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants { 
        private AutoConstants() {}

        public static final double SEARCH_DISTANCE = 0.1; //TODO: tune me
        public static final double MIN_VELOCITY = 0.02; // stops deadlocking at 0 m/s on path //TODO: tune me
        public static final double TARGET_END_DELTA = 0.01; // meters from end waypoint
        public static final double TARGET_ANGLE_DELTA = 0.5;
        public static final int TARGET_COUNT_AT_SETPIONT = 5; //number of robot loops to maintain at setpoint

        // Cascading control into velocity PID, be careful
        public static final double ANGLE_KP = 0.065;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;
        public static final double ANGLE_ERROR_LIMIT = 20.0; // cap max angle to avoid instability from cascading control, in degrees
    }
}
