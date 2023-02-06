package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.robot.Exceptions.PigeonInitializationException;
import frc.robot.Types.Degrees;
import frc.robot.Types.Milliseconds;

public class NineAxis {
    private final WPI_Pigeon2 pidgey;

    public NineAxis(WPI_Pigeon2 pigeon2, Milliseconds timeout) throws PigeonInitializationException {
        pidgey = pigeon2;
        pidgey.reset();

        ErrorCode result = pidgey.configFactoryDefault(timeout.getMilliseconds());

        if (result != ErrorCode.OK) {
            throw new PigeonInitializationException(result);
        }
    }

    public NineAxis(WPI_Pigeon2 pigeon2) throws PigeonInitializationException {
        this(pigeon2, new Milliseconds(0));
    }

    public void reset() {
        pidgey.reset();
    }

    public Degrees getYaw() {
        return new Degrees(pidgey.getYaw());
    }
}
