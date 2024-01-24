
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Controllers {
  private CommandXboxController commandDriveController;
  private CommandXboxController commandManipController;
  private XboxController driveController;
  private XboxController manipController;

  private static Controllers instance;

  public enum Button{
    A(1), B(2), X(3), Y(4), LB(5), RB(6), LS(9), RS(10);

    public final int id; 
    Button(int id) {
      this.id = id;
    }
  }

  public enum Axis{
    LX(0), LY(1), RX(4), RY(5), LT(2), RT(3);

    public final int id;
    Axis(int id) {
      this.id = id;
    }
  }

  public enum Action{
    PRESSED,
    RELEASED
  }

  public enum ControllerName{
    DRIVE, MANIP
  }

  private Controllers(){
    commandDriveController = new CommandXboxController(Constants.DRIVE_CONTROLLER_PORT);
    commandManipController = new CommandXboxController(Constants.MANIP_CONTROLLER_PORT);
    driveController = commandDriveController.getHID();
    manipController = commandManipController.getHID();
  }

  /**Checks if specified button is depressed
   * @param name Controller name DRIVE/MANIP
   * @param button Button name
   * @return boolean if button is pressed.
   * If controller is specified incorrectly, returns false
   */
  public boolean get(ControllerName name, Button b){
    switch(name){
    case DRIVE:
      return driveController.getRawButton(b.id);
    case MANIP:
      return manipController.getRawButton(b.id);
    default:
      return false;
    }
  }

  /**Checks the value of the specified axis
   * @param name Controller name DRIVE/MANIP
   * @param axis Axis name
   * @return double of axis value, between -1 and 1
   * If controller is specified incorrectly, returns 0
   */
  public double get(ControllerName name, Axis a){
    switch(name){
    case DRIVE:
      return driveController.getRawAxis(a.id);
    case MANIP:
      return manipController.getRawAxis(a.id);
    default:
      return 0.0;
    }
  }

  public Trigger getTrigger(ControllerName name, Button b){
    switch(name){
    case DRIVE:
      return commandDriveController.button(b.id);
    case MANIP:
      return commandManipController.button(b.id);
    default:
      return null;
    }
  }

  /**Returns DPAD's POV degree value
   * @param name Controller name DRIVE/MANIP
   * @return integer of DPAD value
   * If controller is specified incorrectly, returns 0
   */
  public int getDPAD(ControllerName name) {
    switch(name) {
      case DRIVE:
        return driveController.getPOV();
      case MANIP:
        return manipController.getPOV();
      default:
        return 0;
    }
  }

  public static Controllers getInstance(){
    if(instance==null){
      instance = new Controllers();
    }
    return instance;
  }
}
