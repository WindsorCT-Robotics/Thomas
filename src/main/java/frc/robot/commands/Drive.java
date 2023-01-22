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
        // TODO Auto-generated method stub
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub

    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
