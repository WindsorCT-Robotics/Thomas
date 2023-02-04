package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NineAxis;

public class Drive extends CommandBase {
    private final DoubleSupplier joystickX;
    private final DoubleSupplier joystickY;

    private final Drivetrain drivetrain;
    private final NineAxis nineAxis;

    public Drive(Drivetrain drivetrain, NineAxis nineAxis, DoubleSupplier joystickX, DoubleSupplier joystickY) {
        this.joystickX = joystickX;
        this.joystickY = joystickY;
        this.drivetrain = drivetrain;
        this.nineAxis = nineAxis;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> requirements = new HashSet<>();
        requirements.add(drivetrain);
        return requirements;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = joystickY.getAsDouble() * 0.8;
        drivetrain.setSpeed(speed);
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
