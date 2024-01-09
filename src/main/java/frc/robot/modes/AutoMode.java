package frc.robot.modes;

import frc.lib.util.OperationMode;
import frc.robot.AutoSelector;

public class AutoMode implements OperationMode{
    AutoSelector autoSelector;

    public AutoMode(){
        autoSelector = new AutoSelector();
    }

    @Override
    public void enter() {
        autoSelector.getSelectedCommand().schedule();
    }

    @Override
    public void exit() {}

    @Override
    public void periodic() {}
}
