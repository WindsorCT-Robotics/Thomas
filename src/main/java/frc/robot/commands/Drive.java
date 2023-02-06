package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Types.MotorPower;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    private final DoubleSupplier move;
    private final DoubleSupplier rotate;

    private final Drivetrain drivetrain;

    public Drive(Drivetrain drivetrain, DoubleSupplier rotate, DoubleSupplier move) {
        this.rotate = rotate;
        this.move = move;
        this.drivetrain = drivetrain;
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
        Pair<MotorPower, MotorPower> motorPower = turn();
        drivetrain.setMotorPower(motorPower.getFirst(), motorPower.getSecond());
    }

    private Pair<MotorPower, MotorPower> turn () {
        Pair<MotorPower, MotorPower> motorPower =
            new Pair<MotorPower,MotorPower>(
                new MotorPower(move.getAsDouble()),
                new MotorPower(move.getAsDouble()));
        Double rotation = rotate.getAsDouble();
        boolean isTurningRight = rotation > 0.0d;
        boolean isTurningLeft = rotation < 0.0d;
        
        if (isTurningLeft) {
            motorPower = new Pair<MotorPower,MotorPower>(
                motorPower.getFirst(),
                MotorPower.add(motorPower.getSecond(), -rotation)
            );
        }
        else if (isTurningRight) {
            motorPower = new Pair<MotorPower, MotorPower>(
                MotorPower.add(motorPower.getFirst(), -rotation),
                motorPower.getSecond()
            );
        }

        return motorPower;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
