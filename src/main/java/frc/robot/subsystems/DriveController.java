package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Types.MetersPerSecond;

public class DriveController extends SubsystemBase {
    private MetersPerSecond speed = new MetersPerSecond(0);
    private final DoubleSupplier move;
    private final SlewRateLimiter moveLimiter;

    private Rotation2d angle = Rotation2d.fromDegrees(0);
    private final DoubleSupplier turn;
    private final SlewRateLimiter turnLimiter;

    private final CommandXboxController controller;
    public final double deadzone;

    public DriveController(int port, double deadzone, double positiveRateLimit, double negativeRateLimit) {
        controller = new CommandXboxController(port);
        this.deadzone = deadzone;

        moveLimiter = new SlewRateLimiter(positiveRateLimit, negativeRateLimit, 0);
        move = deadzoneModifier(() -> moveLimiter.calculate(controller.getLeftY()), this.deadzone);

        turnLimiter = new SlewRateLimiter(positiveRateLimit, negativeRateLimit, 0);
        turn = deadzoneModifier(() -> turnLimiter.calculate(controller.getRightX()), this.deadzone);
    }

    @Override
    public void periodic() {
        angle = angle.plus(
                Rotation2d.fromDegrees(turn.getAsDouble() * Drivetrain.MAX_ANGULAR_VELOCITY.getRadiansPerSecond()));
        speed = new MetersPerSecond(move.getAsDouble() * Drivetrain.MAX_VELOCITY.getMetersPerSecond());
    }

    public Rotation2d getAngle() {
        return angle;
    }

    public MetersPerSecond getSpeed() {
        return speed;
    }

    private static DoubleSupplier deadzoneModifier(DoubleSupplier rawValueSupplier, double deadzone) {
        return () -> {
            double rawValue = rawValueSupplier.getAsDouble();
            double result;
            double validRange = 1 - deadzone;
            double value = Math.abs(rawValue);

            if (value > deadzone) {
                result = (value - deadzone) / validRange;
            } else {
                result = 0;
            }

            return rawValue < 0 ? -result : result;
        };
    }
}
