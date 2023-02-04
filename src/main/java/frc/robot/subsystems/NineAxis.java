package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.robot.Mathematics.Degrees;

public class NineAxis {
    private final WPI_Pigeon2 pidgey;

    public NineAxis(WPI_Pigeon2 pigeon2) throws Exception {
        pidgey = pigeon2;
        pidgey.reset();

        if (pidgey.configFactoryDefault() != ErrorCode.OK) {
            throw new Exception("Error initializing Pigeon2");
        }
    }

    public void reset() {
        pidgey.reset();
    }

    public Degrees getYaw() {
        return new Degrees(pidgey.getYaw());
    }
}
