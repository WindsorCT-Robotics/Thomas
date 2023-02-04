package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    private DoubleSupplier joystickX;
    private DoubleSupplier joystickY;

    private Drivetrain drivetrain;

    public Drive(DoubleSupplier joystickX, DoubleSupplier joystickY) {
        this.joystickX = joystickX;
        this.joystickY = joystickY;

        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        drivetrain.setPositionLock(false);
    }

    @Override
    public void execute() {
        double acceleration = joystickY.getAsDouble();
        double speed = drivetrain.getSpeed();
        drivetrain.setSpeed(speed + acceleration);
        final double degreesPerSecond = 90; // adjust to change curve speed

        double turnStrength = joystickX.getAsDouble();
        double heading = drivetrain.getTargetHeading();
        double rotation = (turnStrength * degreesPerSecond) / speed; // the faster we go, the slower we turn
        drivetrain.setTargetHeading(heading + rotation);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
