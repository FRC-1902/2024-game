package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.ModuleStateOptimizer;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import frc.lib.util.CANSparkMaxUtil;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;

public class SwerveModule {
  private int moduleNumber;
  private Rotation2d angleOffset;
  private Rotation2d lastAngle;
  private double desiredSpeed;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;
  
  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private SparkAbsoluteEncoder angleEncoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);
  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Helium Cancoder Config - PLUG INTO ANGLE MOTOR*/
    angleEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    configAngleEncoder();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = ModuleStateOptimizer.optimize(desiredState, getState().angle); 
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  // use to calculate feed forward values
  /*/
  private double tmpTime = System.currentTimeMillis();
  private double tmpVel = 0.0;
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    double voltage = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed * 12.0;
    driveMotor.setVoltage(voltage);

    double accel = (driveEncoder.getVelocity()-tmpVel)/((System.currentTimeMillis()-tmpTime)/1000.0);

    if (moduleNumber == 0 && voltage > 0 && accel > 0 && driveEncoder.getVelocity() > 0) {
      double err = voltage - (Constants.Swerve.driveKA + Constants.Swerve.driveKV * driveEncoder.getVelocity() + Constants.Swerve.driveKA * accel);
      System.out.format("Voltage: %.3f Velocity: %.4f Acceleration: %.5f Err: %.3f\n", voltage, driveEncoder.getVelocity(), accel, err);
    }
    tmpTime = System.currentTimeMillis();
    tmpVel = driveEncoder.getVelocity();
    
  }*/

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
    desiredSpeed = desiredState.speedMetersPerSecond;
    if(isOpenLoop){
      // TODO: maybe use feedforward here instead
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
      driveMotor.set(percentOutput);
    }
    else {
      driveController.setReference(
        desiredState.speedMetersPerSecond,
        CANSparkMax.ControlType.kVelocity,
        0,
        feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState){
    //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; 
    
    angleController.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    lastAngle = angle;
  }

  private Rotation2d getAngle(){
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  // XXX: Not actually pulling from cancoder, maybe rename?
  public Rotation2d getCanCoder(){
    return (Constants.Swerve.CAN_CODER_INVERT) ? Rotation2d.fromRotations(1 - angleEncoder.getPosition()) : Rotation2d.fromRotations(angleEncoder.getPosition());
  }

  public double getDesiredSpeed() {
    return desiredSpeed;
  }

  public void resetToAbsolute(){
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();

    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder(){     
    angleEncoder.setAverageDepth(8); // bit sampling depth (must be a power of 2 up to 128 )
    angleEncoder.setPositionConversionFactor(1); // angle encoder is directly on the correct shaft
    angleEncoder.setZeroOffset(angleOffset.getRotations());
  }

  private void configAngleMotor(){
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.POSITION_ONLY);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_CURRENT_LIMIT);
    angleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
    angleMotor.setIdleMode(Constants.Swerve.ANGLE_NEURTRAL_MODE);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.ANGLE_CONVERSION_FACTOR);
    angleController.setP(Constants.Swerve.ANGLE_KP);
    angleController.setI(Constants.Swerve.ANGLE_KI);
    angleController.setD(Constants.Swerve.ANGLE_KD);
    angleController.setFF(Constants.Swerve.ANGLE_KF);
    angleMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
    angleMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor(){        
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.ALL);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_CURRENT_LIMIT);
    driveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
    driveMotor.setIdleMode(Constants.Swerve.DRIVE_NEUTRAL_MODE);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR);
    driveController.setP(Constants.Swerve.DRIVE_KP);
    driveController.setI(Constants.Swerve.DRIVE_KI);
    driveController.setD(Constants.Swerve.DRIVE_KD);
    driveController.setFF(Constants.Swerve.DRIVE_KF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public int getModuleNumber() {
    return moduleNumber;
  }
}