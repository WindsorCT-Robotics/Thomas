package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Exceptions.PigeonInitializationException;
import frc.robot.Types.Milliseconds;

public class NineAxis extends SubsystemBase {
    private final WPI_Pigeon2 pidgey;

    public NineAxis(WPI_Pigeon2 pigeon2, Milliseconds timeout) throws PigeonInitializationException {
        pidgey = pigeon2;
        pidgey.reset();
        addChild("Pigeon", pidgey);
        

        ErrorCode result = pidgey.configFactoryDefault(timeout.getMilliseconds());

        if (result != ErrorCode.OK) {
            throw new PigeonInitializationException(result);
        }
    }

    @Override
    public void periodic() {
        doTelemetry();
    }

    /**
     * Output telemetry data
     */
    private void doTelemetry() {
        SmartDashboard.putNumber("Pigeon Yaw (degrees)", getYaw().getDegrees());
    }

    public NineAxis(WPI_Pigeon2 pigeon2) throws PigeonInitializationException {
        this(pigeon2, new Milliseconds(0));
    }

    public void reset() {
        pidgey.reset();
    }

    /**
     * Get robot yaw as Rotation2d
     * @return pigeon yaw
     */
    public Rotation2d getYaw() {
        return pidgey.getRotation2d();
    }
}
