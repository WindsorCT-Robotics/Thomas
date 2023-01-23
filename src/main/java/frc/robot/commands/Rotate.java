package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Rotate extends CommandBase {
    private DoubleSupplier joystickX;
    private Drivetrain drivetrain;
    private final int turnSpeedScale = 1;

    public Rotate(DoubleSupplier joystickX) {
        this.joystickX = joystickX;
        drivetrain = Drivetrain.getInstance();

    }

    @Override
    public void initialize() {
        drivetrain.setPositionLock(true);
    }

    @Override
    public void execute() {
        double turnStrength = joystickX.getAsDouble();
        double rotation = Math.abs(turnStrength) * turnStrength * turnSpeedScale;
        double heading = drivetrain.getAbsoluteHeading();
        drivetrain.setAbsoluteHeading(heading + rotation);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setPositionLock(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
