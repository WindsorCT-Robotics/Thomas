package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RotateToHeading extends CommandBase {
    private Drivetrain drivetrain;
    private double targetHeading;

    public RotateToHeading(double targetHeading) {
        this.targetHeading = targetHeading;
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setTargetHeading(targetHeading);
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
