package frc.robot.Exceptions;

import com.ctre.phoenix.ErrorCode;

public class PigeonInitializationException extends RuntimeException {
    public PigeonInitializationException(ErrorCode error) {
        super(String.format("Failed to initialize Pigeon IMU: %d", error.value));
    }
}
