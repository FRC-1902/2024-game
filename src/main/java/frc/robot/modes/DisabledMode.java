package frc.robot.modes;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.OperationMode;
import frc.robot.subsystems.Swerve;

public class DisabledMode implements OperationMode{
    private Swerve swerveSubsystem;

    public DisabledMode(){
        swerveSubsystem = Swerve.getInstance();
    }

    @Override
    public void enter() {
        swerveSubsystem.drive(
            new Translation2d(0, 0), 
            0.0, 
            false, 
            true
        );
    }

    @Override
    public void exit() {}

    @Override
    public void periodic() {}
}
