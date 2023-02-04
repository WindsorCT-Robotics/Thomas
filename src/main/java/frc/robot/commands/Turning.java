package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Mathematics.Degrees;
import frc.robot.Mathematics.PID;

public class Turning {
    private final PID pid;

    private final PIDController pidController;
    private final WPI_Pigeon2 pidgey;

    public Turning (PID pid, PIDController pidController, WPI_Pigeon2 pidgey) {
        this.pid = pid;
        this.pidController = pidController;
        this.pidgey = pidgey;
    }

    public Degrees turn (Degrees currentHeading, Degrees targetHeading) {
        pidController.setPID(pid.getP(), pid.getI(), pid.getD());

        pidController.calculate()
    }

}
