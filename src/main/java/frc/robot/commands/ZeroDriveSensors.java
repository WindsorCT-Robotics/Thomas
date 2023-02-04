package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ZeroDriveSensors extends CommandBase {
    Drivetrain drivetrain;

    public ZeroDriveSensors() {
        drivetrain = Drivetrain.getInstance();
    }

    @Override
    public void execute() {
        drivetrain.zeroSensors();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
