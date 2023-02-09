package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Types.MetersPerSecond;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    private final DoubleSupplier move;
    private final SlewRateLimiter moveLimiter;
    private final DoubleSupplier turn;
    private final SlewRateLimiter turnLimiter;
    private final DifferentialDriveKinematics kinematics;

    private final Drivetrain drivetrain;

    public Drive(Drivetrain drivetrain, DoubleSupplier turn, SlewRateLimiter rotateLimiter, DoubleSupplier move,
            SlewRateLimiter moveLimiter) {
        this.turn = turn;
        this.turnLimiter = rotateLimiter;
        this.move = move;
        this.moveLimiter = moveLimiter;
        this.drivetrain = drivetrain;

        kinematics = new DifferentialDriveKinematics(Drivetrain.TRACK_WIDTH.getMeters());
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
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics
                .toWheelSpeeds(new ChassisSpeeds(
                        moveLimiter.calculate(move.getAsDouble() * Drivetrain.MAX_SPEED.getMetersPerSecond()),
                        0, // our robot can't fly
                        turnLimiter.calculate(turn.getAsDouble())
                                * Drivetrain.MAX_ANGULAR_SPEED.getDegreesPerSecond()));
        drivetrain.setMotorSpeed(new MetersPerSecond(wheelSpeeds.leftMetersPerSecond),
                new MetersPerSecond(wheelSpeeds.rightMetersPerSecond));

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
