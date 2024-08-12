package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public class Constants {
    private Constants() {}
    // Controllers
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int MANIP_CONTROLLER_PORT = 1;

    public static final double STICK_DEADBAND = 0.1;

    public static final int LED_PORT = 5; 
    

    public static final class Climber{
        private Climber() {}
        // CAN ids
        public static final int LEFT_MOTOR_ID = 4; 
        public static final int RIGHT_MOTOR_ID = 7;

        // DIO ports
        public static final int LB_SWITCH_PORT = 0; 
        public static final int LT_SWITCH_PORT = 1; 
        public static final int RB_SWITCH_PORT = 2; 
        public static final int RT_SWITCH_PORT = 3; 

        // Power Budgeting
        public static final int CLIMBER_CURRENT_LIMIT = 40;
    }

    public static final class Intake {
        private Intake() {}

        public static final int INTAKE_MOTOR_ID = 3;
        public static final int INTAKE_CURRENT_LIMIT = 40;
    }

    public static final class Arm {
        private Arm() {}

        // CAN IDs
        public static final int PIVOT_MOTOR_1_ID = 5;
        public static final int PIVOT_MOTOR_2_ID = 8;
        public static final int INDEXER_MOTOR_ID = 13;
        public static final int TOP_SHOOTER_MOTOR_ID = 14; 
        public static final int BOTTOM_SHOOTER_MOTOR_ID = 15;

        // Pivot Encoder Offset
    /****NOTE**** ----------- Will need to be reset on every change of the pivot. Look until pivot encoder is 0.5 when straight up ----------- ****NOTE****/
        public static final Rotation2d PIVOT_ANGLE_OFFSET = Rotation2d.fromRotations(0.420);

        // PID Values - Old arm values: 1.8, 0.2, 0.015, 0.1
        public static final double PIVOT_KP = 1.8;
        public static final double PIVOT_KI = 0.4;
        public static final double PIVOT_KD = 0.01;
        public static final double PIVOT_KF = 0.06;// gravity compensation feedforward

        // Pivot Positions
        public static final Rotation2d PIVOT_MIN_ROTATION = Rotation2d.fromDegrees(50);
        public static final Rotation2d PIVOT_MAX_ROTATION = Rotation2d.fromDegrees(195.84);
        public static final Rotation2d PIVOT_DEGREES_TOLERANCE = Rotation2d.fromRotations(0.008); // tuned in just above mechanical slop

        // Power Considerations
        public static final double SHOOTER_VOLTAGE_COMPENSATION = 12.0;
        public static final int SHOOTER_CURRENT_LIMIT = 50;
        public static final int INDEX_CURRENT_LIMIT = 60;
        public static final int PIVOT_CURRENT_LIMIT = 60; //60

        // Arm lengths 
        public static final double PIVOT_LENGTH = Units.inchesToMeters(21.07);
        public static final double ARM_LENGTH = Units.inchesToMeters(20.82);
        public static final double WRIST_LENGTH = Units.inchesToMeters(12.06);
        public static final Rotation2d WRIST_OFFSET = Rotation2d.fromDegrees(61.72);

        // cubic curve magic numbers to relate pivot angle and distance from speaker
        public static final double SHOOTER_MAGIC_A = 1.38;
        public static final double SHOOTER_MAGIC_B = -13.080;
        public static final double SHOOTER_MAGIC_C = 45.704;
        public static final double SHOOTER_MAGIC_D = 71.0; // 72.0

        // max distance to shoot into the speaker in meters
        public static final double SHOOTER_MAX_DISTANCE = 2.7;

        // position to line up pivot with things on the field (0 is straight down)
        public static final Rotation2d AMP_PIVOT_LINEUP = Rotation2d.fromDegrees(177.27);
        public static final Rotation2d HP_PIVOT_LINEUP = Rotation2d.fromDegrees(144.39);
        public static final Rotation2d STOW_PIVOT_LINEUP = Rotation2d.fromDegrees(65.0);
        public static final Rotation2d INTAKE_PIVOT_LINEUP = Rotation2d.fromDegrees(53.76);

        public static final int IR_PIECE_SENSOR_PORT = 4;
    }

    public static final class Swerve {
        private Swerve() {} 
 
        public static final boolean GYRO_INVERT = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.DriveGearRatios.SDSMK4_L3);

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
        public static final boolean ABS_ENCODER_INVERT = chosenModule.canCoderInvert;
        
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

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.25; 
        public static final double DRIVE_KI = 0.001;
        public static final double DRIVE_KD = 0.5;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values */
        // divide by 12 to convert from volts to 1 to -1 power range
        public static final double DRIVE_KS = 0.501892;
        public static final double DRIVE_KV = 1.874719;
        public static final double DRIVE_KA = 0.296733;

        /* Drive Motor Conversion Factors */
        public static final double DRIVE_CONVERSION_POSITION_FACTOR =
            (chosenModule.wheelDiameter * Math.PI) / DRIVE_GEAR_RATIO;
        public static final double DRIVE_CONVERSION_VELOCITY_FACTOR = DRIVE_CONVERSION_POSITION_FACTOR / 60.0;
        public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

        /* Swerve Profiling Values */
        /* Must be max drivetrain speeds for open loop control */
        /** Meters per Second */ 
        public static final double MAX_SPEED = Units.feetToMeters(13.73);
        /** Meters per Second squared */
        public static final double MAX_ACCELERATION = Units.feetToMeters(24.16);
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(243.79) * 60;
        /** Radians per Second squared*/
        public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(428.99) * 60;
        
        /* Neurtral (Idle) Modes */
        public static final IdleMode ANGLE_NEURTRAL_MODE = IdleMode.kCoast;
        public static final IdleMode DRIVE_NEUTRAL_MODE = IdleMode.kBrake;

        /* Note: helium cancoders have a reset for angle offset integrated onto them */

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            private Mod0() {}
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 1;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            private Mod1() {}
            public static final int DRIVE_MOTOR_ID = 10;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            private Mod2() {}
            public static final int DRIVE_MOTOR_ID = 19;
            public static final int ANGLE_MOTOR_ID = 20;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            private Mod3() {}
            public static final int DRIVE_MOTOR_ID = 12;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET);
        }

        /* Photonvision camera transforms */
        /* NOTE: DO NOT SET ROTATION, this is set elsewhere in the swerve code */
        public static final Transform3d FRONT_CAMERA_OFFSET = new Transform3d(
            new Translation3d(-0.292, Units.inchesToMeters(6.12), -0.666), 
            new Rotation3d(0,0,0)
        );
        public static final Transform3d BACK_CAMERA_OFFSET = new Transform3d(
            new Translation3d(-0.15, Units.inchesToMeters(-6.12), -0.666), 
            new Rotation3d(0,0,0)
        );
    }

    public static final class AutoConstants { 
        private AutoConstants() {}
        // PID contstants for pathplannerlib // 0.15
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(2.8, 0.0, 0.0, 0.5);
        public static final PIDConstants ROTATION_PID = new PIDConstants(2.6, 0.0, 0, 0.1);

        // turn in place command
        public static final Rotation2d TURN_TOLERANCE = Rotation2d.fromDegrees(0.4);
        public static final double TURN_KP = 19;
        public static final double TURN_KI = 0.02;
        public static final double TURN_KD = 0.0;
    }
}
