package frc.robot.modes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.OperationMode;
import frc.robot.Constants;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Controllers.Axis;
import frc.robot.subsystems.Controllers.Button;
import frc.robot.subsystems.Controllers.ControllerName;

public class TeleOpMode implements OperationMode{
    private final Swerve swerveSubsystem;
    private Controllers controllers;
    
    public TeleOpMode(){
        controllers = Controllers.getInstance();
        swerveSubsystem = Swerve.getInstance();
    }

    @Override
    public void enter() {} // XXX: maybe make drive controller shake?

    @Override
    public void exit() {
        swerveSubsystem.drive(
            new Translation2d(0, 0), 
            0.0, 
            true, 
            true
        );
    }

    @Override
    public void periodic() {
        handleDriving();
    }

    private void handleDriving() {
        // get drive values from controller
        double translationVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.LY), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.LX), Constants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(-controllers.get(ControllerName.DRIVE, Axis.RX), Constants.STICK_DEADBAND);
        boolean isFieldRelative = !controllers.get(ControllerName.DRIVE, Button.LB);

        // cube controls for better handling, and scale down for softer pre-season movement
        translationVal = Math.pow(translationVal, 3.0);
        translationVal *= 0.5;
        strafeVal = Math.pow(strafeVal, 3.0);
        strafeVal *= 0.5;
        rotationVal = Math.pow(rotationVal, 3.0);
        rotationVal *= 0.5;

        // drive
        swerveSubsystem.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
            rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY, 
            isFieldRelative, 
            true
        );

        // check for reset field oriented mode
        if (controllers.getPressed(ControllerName.DRIVE, Button.Y)) {
            swerveSubsystem.zeroGyro();
            System.out.println(IMU.getInstance().getDriverHeading());
        }
    }
}
