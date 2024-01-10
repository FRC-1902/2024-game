package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSelector {
    private LoggedDashboardChooser<Command> autoChooser;
    
    public AutoSelector() {
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
        autoChooser.addDefaultOption("Do Nothing", new SequentialCommandGroup());
        // autoChooser.addOption("TestAuto1Path", getTestAuto());
        SmartDashboard.putData("Auto Choices", autoChooser.getSendableChooser());
    }

    /**
     * @return The selected auto from smart dashboard
     */
    public Command getSelectedCommand() {
        DataLogManager.log("Sending command: " + autoChooser.get().toString());
        return autoChooser.get();
    }

    // Auto definitions

    // private SequentialCommandGroup getTestAuto() {
        // TODO: implement better path finding through PathPlannerLib
    // }
}