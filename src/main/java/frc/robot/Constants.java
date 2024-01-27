package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.DriveGearRatios.SDSMK4_L3); // TODO: check ratio

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(20.504);
        public static final double WHEEL_BASE = Units.inchesToMeters(20.504);
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
        public static final int DRIVE_CURRENT_LIMIT = 45;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = chosenModule.angleKP;
        public static final double ANGLE_KI = chosenModule.angleKI;
        public static final double ANGLE_KD = chosenModule.angleKD;
        public static final double ANGLE_KF = chosenModule.angleKF;

        /* Drive Motor PID Values */  // TODO: tune me on real robot now
        public static final double DRIVE_KP = 0.25; 
        public static final double DRIVE_KI = 0.001;
        public static final double DRIVE_KD = 0.5;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values */ // TODO: tune me on real robot now
        // divide by 12 to convert from volts to 1 to -1 power range
        public static final double DRIVE_KS = 0.338492;
        public static final double DRIVE_KV = 1.533248;
        public static final double DRIVE_KA = 0.305292;

		/* Drive Motor Conversion Factors */
		public static final double DRIVE_CONVERSION_POSITION_FACTOR =
        	(chosenModule.wheelDiameter * Math.PI) / DRIVE_GEAR_RATIO;
    	public static final double DRIVE_CONVERSION_VELOCITY_FACTOR = DRIVE_CONVERSION_POSITION_FACTOR / 60.0;
    	public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

        /* Swerve Profiling Values */
        /* Must be max drivetrain speeds for open loop control */
        /** Meters per Second */ 
        public static final double MAX_SPEED = Units.feetToMeters(13.73); // XXX: theoretical Kevin values
        /** Meters per Second squared */
        public static final double MAX_ACCELERATION = Units.feetToMeters(24.16); // XXX: theoretical Kevin values
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(243.79) * 60; // XXX: theoretical Kevin values
        /** Radians per Second squared*/
        public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(428.99) * 60; //TODO: set new Kevin values
        
        /* Neutral (Idle) Modes */
        public static final IdleMode ANGLE_NEURTRAL_MODE = IdleMode.kCoast;
        public static final IdleMode DRIVE_NEUTRAL_MODE = IdleMode.kBrake;

        /* Note: helium cancoders have a reset for angle offset integrated onto them */

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            private Mod0() {}
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            private Mod1() {}
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 5;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            private Mod2() {}
            public static final int DRIVE_MOTOR_ID = 6;
            public static final int ANGLE_MOTOR_ID = 7;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            private Mod3() {}
            public static final int DRIVE_MOTOR_ID = 8;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants { 
        private AutoConstants() {}
        // TODO: make me
        // PID contstants
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0, 0); // TODO: tune me
        public static final PIDConstants ROTATION_PID = new PIDConstants(0, 0, 0, 0); // TODO: tune me

    }
}
