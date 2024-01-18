package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.sensors.BNO055;
import frc.robot.Constants;

public class IMU extends SubsystemBase{

  private static IMU instance = new IMU();

  // only velocity or acceleration can be used at once, can't use both at the same time
  private final BNO055 bno055Euler = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
    BNO055.vector_type_t.VECTOR_EULER);

  private Rotation2d driverOffset;
  private Rotation2d fieldOffset;

  private IMU() {
    driverOffset = Rotation2d.fromDegrees(0);
    initializeLogger();
  }

  private void initializeLogger() {
    Logger.recordOutput("IMU heading", getDriverHeading().getDegrees());
    Logger.recordOutput("IMU roll", getRoll());
    Logger.recordOutput("IMU pitch", getPitch());
    Logger.recordOutput("IMU turn", getTurns());
    Logger.recordOutput("IMU offset", driverOffset.getDegrees());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("IMU heading", getDriverHeading().getDegrees());
    Logger.recordOutput("IMU roll", getRoll());
    Logger.recordOutput("IMU pitch", getPitch());
    Logger.recordOutput("IMU turn", getTurns());
  }

  /**
   * @return returns the imu's x scalar (heading/yaw) as a Rotation2d object for field-relative driver usage
   */
  public Rotation2d getDriverHeading() {
    double[] xyz = bno055Euler.getVector();
    return (Constants.Swerve.GYRO_INVERT) ? Rotation2d.fromDegrees(360 - xyz[0]).plus(driverOffset) : Rotation2d.fromDegrees(xyz[0]).plus(driverOffset);
  }

  /**
   * @return returns the imu's x scalar (heading/yaw) as a Rotation2d object for field-relative driver usage
   */
  public Rotation2d getFieldHeading() {
    double[] xyz = bno055Euler.getVector();
    return (Constants.Swerve.GYRO_INVERT) ? Rotation2d.fromDegrees(360 - xyz[0]).plus(fieldOffset) : Rotation2d.fromDegrees(xyz[0]).plus(fieldOffset);
  }

  /**
   * @return returns the imu's y scalar (roll) representing an angle from -90 to
   *         90 degrees
   */
  public double getRoll() {
    double[] xyz = bno055Euler.getVector();
    return xyz[1];
  }

  /**
   * @return returns the imu's z scalar (pitch) representing an angle from -180 to
   *         180 degrees
   */
  public double getPitch() {
    double[] xyz = bno055Euler.getVector();
    return xyz[2];
  }

  /**
   * @return the signed sum of the amount of full rotations the BNO has taken
   */
  public long getTurns() {
    return bno055Euler.getTurns();
  }

  /**
   * @param offset sets imu x heading offset for field-relative driver usage
   */
  public void setDriverOffset(Rotation2d offset) {
    this.driverOffset = offset;
    Logger.recordOutput("IMU offset", offset.getDegrees());
  }

  /**
   * @return imu x heading offset for field-relative driver usage
   */
  public Rotation2d getDriverOffset() {
    return driverOffset;
  }

  /**
   * @param offset sets imu x heading offset for field-relative auto & pose estimator usage
   */
  public void setFieldOffset(Rotation2d offset) {
    this.fieldOffset = offset;
    Logger.recordOutput("IMU offset", offset.getDegrees());
  }

  /**
   * @return imu x heading offset for field-relative auto & pose estimator usage
   */
  public Rotation2d getFieldOffset() {
    return fieldOffset;
  }

  /**
   * resets imu x heading to default offset
   */
  public void resetHeading() {
    bno055Euler.resetHeading();
  }

  /**
   * @return true if the sensor is found on the I2C bus
   */
  public boolean isSensorPresent() {
    return bno055Euler.isSensorPresent();
  }

  /**
   * @return true when the sensor is initialized.
   */
  public boolean isInitialized() {
    return bno055Euler.isInitialized();
  }

  /**
   * @return true if calibration is complete for all sensors required for the
   *         mode the sensor is currently operating in.
   */
  public boolean isCalibrated() {
    return bno055Euler.isCalibrated();
  }

  /**
   * @reutrn imu instance
   */
  public static IMU getInstance() {
    if (instance == null) {
      instance = new IMU();
    }
    return instance;
  }
}