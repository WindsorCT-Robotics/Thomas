package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Types.JoystickAxis;
import frc.robot.Types.MetersPerSecond;
import frc.robot.Types.RadiansPerSecond;

public class DriveController extends SubsystemBase {
    private MetersPerSecond speed = new MetersPerSecond(0);
    private final Supplier<JoystickAxis> move;

    private Rotation2d angle = Rotation2d.fromDegrees(0);
    private final Supplier<JoystickAxis> turn;

    private final CommandXboxController controller;
    public final double deadzone;

    public DriveController(int port, double deadzone, double positiveRateLimit, double negativeRateLimit) {
        controller = new CommandXboxController(port);
        this.deadzone = deadzone;

        move = deadzoneModifier(() -> new JoystickAxis(controller.getLeftY()), this.deadzone);

        turn = deadzoneModifier(() -> new JoystickAxis(controller.getRightX()), this.deadzone);
    }

    @Override
    public void periodic() {
        angle = calculateAngle(angle, turn.get().getAsDouble(), Drivetrain.MAX_ANGULAR_VELOCITY);
        speed = calculateSpeed(move.get().getAsDouble(), Drivetrain.MAX_VELOCITY);
    }
    
    private Rotation2d calculateAngle (Rotation2d startingAngle, double turningAxisValue, RadiansPerSecond maxAngularVelocity) {
        return startingAngle.plus(Rotation2d.fromRadians(turningAxisValue * maxAngularVelocity.getRadiansPerSecond()));
    }

    private MetersPerSecond calculateSpeed (double movementAxisValue, MetersPerSecond maxVelocity) {
        return new MetersPerSecond(movementAxisValue * maxVelocity.getMetersPerSecond());
    }

    public Rotation2d getAngle() {
        return angle;
    }

    public MetersPerSecond getSpeed() {
        return speed;
    }

    private static Supplier<JoystickAxis> deadzoneModifier(Supplier<JoystickAxis> rawValueSupplier, double deadzone) {
        return () -> {
            double rawValue = rawValueSupplier.get().getAsDouble();
            double result;
            double validRange = 1 - deadzone;
            double value = Math.abs(rawValue);

            if (value > deadzone) {
                result = (value - deadzone) / validRange;
            } else {
                result = 0.0d;
            }

            int intValue = (int)(result / 100);

            intValue = Math.max(0, intValue);
            intValue = Math.min(100, intValue);

            return new JoystickAxis(rawValue < 0.0d ? -intValue : intValue);
        };
    }
}
