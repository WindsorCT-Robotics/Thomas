package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Types.MetersPerSecond;
import frc.robot.Types.PID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NineAxis;

public class Drive extends CommandBase {
    private final DoubleSupplier move;
    private final DoubleSupplier turn;

    private final DifferentialDriveKinematics kinematics;
    private final PID turnPid;
    private final PIDController turnController;

    private final Drivetrain drivetrain;
    private final NineAxis pidgey;

    private static PIDController initTurnController(PID pid, Rotation2d minimumInput, Rotation2d maximumInput) {
        PIDController turnController = new PIDController(pid.getP(), pid.getI(), pid.getD());
        turnController.reset();
        turnController.enableContinuousInput(minimumInput.getRadians(), maximumInput.getRadians());

        return turnController;
    }

    public Drive(Drivetrain drivetrain, NineAxis pidgey, DoubleSupplier turn, DoubleSupplier move) {
        this.turn = turn;
        this.move = move;

        this.turnPid = new PID(1, 0, 0); // TODO: Placeholder values
        this.turnController = initTurnController(turnPid, Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(180));

        this.drivetrain = drivetrain;
        this.pidgey = pidgey;

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
        Rotation2d turnAdjustment = Rotation2d.fromRadians(turnController.calculate(pidgey.getYaw().getRadians()));
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics
                .toWheelSpeeds(new ChassisSpeeds(
                        move.getAsDouble() * Drivetrain.MAX_SPEED.getMetersPerSecond(),
                        0, // our robot can't fly
                        turn.getAsDouble() * Drivetrain.MAX_ANGULAR_SPEED.getRadiansPerSecond()
                                + turnAdjustment.getRadians()));

        drivetrain.setMotorSpeed(new MetersPerSecond(wheelSpeeds.leftMetersPerSecond),
                new MetersPerSecond(wheelSpeeds.rightMetersPerSecond));

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
