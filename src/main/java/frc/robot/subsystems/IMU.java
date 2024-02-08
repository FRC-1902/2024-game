package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.sensors.BNO055;
import frc.robot.Constants;

public class IMU extends SubsystemBase{

  private static IMU instance = new IMU();

  // only velocity or acceleration can be used at once, can't use both at the same time
  private final BNO055 bno055Euler = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
    BNO055.vector_type_t.VECTOR_EULER, I2C.Port.kOnboard, BNO055.BNO055_ADDRESS_A);

  private Rotation2d offset, fieldOffset;

  private IMU() {
    offset = Rotation2d.fromDegrees(0);
    fieldOffset = Rotation2d.fromDegrees(0);
    initializeLogger();
  }

  private void initializeLogger() {
    Logger.recordOutput("IMU heading", getHeading().getDegrees());
    Logger.recordOutput("IMU roll", getRoll());
    Logger.recordOutput("IMU pitch", getPitch());
    Logger.recordOutput("IMU turn", getTurns());
    Logger.recordOutput("IMU offset", offset.getDegrees());
    Logger.recordOutput("IMU field offset", fieldOffset.getDegrees());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("IMU heading", getHeading().getDegrees());
    Logger.recordOutput("IMU roll", getRoll());
    Logger.recordOutput("IMU pitch", getPitch());
    Logger.recordOutput("IMU turn", getTurns());
  }

  /**
   * @return returns the imu's x scalar (heading/yaw) as a Rotation2d object
   */
  public Rotation2d getHeading() {
    double[] xyz = bno055Euler.getVector();
    return (Constants.Swerve.GYRO_INVERT) ? Rotation2d.fromDegrees(360 - xyz[0]).plus(offset) : Rotation2d.fromDegrees(xyz[0]).plus(offset);
  }

  /**
   * @return returns the imu's x scalar (heading/yaw) as a Rotation2d object for blue field origin
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
   * @param offset sets imu x heading offset
   */
  public void setOffset(Rotation2d offset) {
    this.offset = offset;
    Logger.recordOutput("IMU offset", offset.getDegrees());
  }

  /**
   * @param offset sets imu x heading offset for blue field origin
   */
  public void setFieldOffset(Rotation2d fieldOffset) {
    this.fieldOffset = fieldOffset;
    Logger.recordOutput("IMU field offset", fieldOffset.getDegrees());
  }

  /**
   * @return imu x heading offset
   */
  public Rotation2d getOffset() {
    return offset;
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