package frc.robot.commands;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Types.MetersPerSecond;
import frc.robot.Types.PID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NineAxis;

public class Drive extends CommandBase {
    private final Supplier<MetersPerSecond> targetSpeed;
    private final Supplier<Rotation2d> targetAngle;

    private final DifferentialDriveKinematics kinematics;
    private final PID turnPid;
    private final PIDController turnController;

    private final Drivetrain drivetrain;
    private final NineAxis pidgey;

    private static PIDController initTurnController(PID pid, Rotation2d minimumInput, Rotation2d maximumInput,
            Rotation2d tolerance) {
        PIDController turnController = new PIDController(pid.getP(), pid.getI(), pid.getD());
        turnController.reset();
        turnController.setTolerance(tolerance.getRadians());
        turnController.enableContinuousInput(minimumInput.getRadians(), maximumInput.getRadians());

        return turnController;
    }

    public Drive(Drivetrain drivetrain, NineAxis pidgey, Supplier<MetersPerSecond> targetSpeed,
            Supplier<Rotation2d> targetAngle) {
        this.targetAngle = targetAngle;
        this.targetSpeed = targetSpeed;

        turnPid = new PID(1, 0, 0); // TODO: Placeholder values
        Rotation2d tolerance = Rotation2d.fromDegrees(3); // TODO: Placeholder value
        turnController = initTurnController(turnPid, Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(180),
                tolerance);
        SendableRegistry.setName(turnController, "Drivetrain", "Turn PID Controller");

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
        Rotation2d turnAdjustment = Rotation2d
                .fromRadians(turnController.calculate(pidgey.getYaw().getRadians(), targetAngle.get().getRadians()));
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                targetSpeed.get().getMetersPerSecond(),
                0, // our robot can't sidle
                turnAdjustment.getRadians());
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        wheelSpeeds.desaturate(Drivetrain.MAX_VELOCITY.getMetersPerSecond());

        drivetrain.setMotorSpeed(wheelSpeeds);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
