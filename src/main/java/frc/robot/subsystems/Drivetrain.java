package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    // Motors
    private final WPI_TalonFX leftMaster;
    private final WPI_TalonFX leftFollower;
    private final WPI_TalonFX rightMaster;
    private final WPI_TalonFX rightFollower;

    // Gyroscope
    private final WPI_Pigeon2 pidgey;

    // Control variables
    private double speed = 0.0;
    private boolean positionLock = false;
    private boolean antiDrift = false;

    private double rotation = 0.0;
    private boolean rotationLock = false;

    private static Drivetrain drive;

    private static WPI_TalonFX initMotor(int deviceNumber) {
        WPI_TalonFX motor = new WPI_TalonFX(deviceNumber);
        motor.configFactoryDefault();

        return motor;
    }

    public Drivetrain() {
        leftMaster = initMotor(1);
        leftFollower = initMotor(2);
        rightMaster = initMotor(3);
        rightFollower = initMotor(4);

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        pidgey = new WPI_Pigeon2(20);
        pidgey.configFactoryDefault();

    }

    public static synchronized Drivetrain getInstance() {
        if (drive == null) {
            drive = new Drivetrain();
        }
        return drive;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        leftMaster.setNeutralMode(neutralMode);
        rightMaster.setNeutralMode(neutralMode);
    }

    public boolean isPositionLock() {
        return positionLock;
    }

    public void setPositionLock(boolean positionLock) {
        this.positionLock = positionLock;
    }

    public boolean isRotationLock() {
        return rotationLock;
    }

    public void setRotationLock(boolean rotationLock) {
        this.rotationLock = rotationLock;
    }

    public boolean isAntiDrift() {
        return antiDrift;
    }

    public void setAntiDrift(boolean antiDrift) {
        this.antiDrift = antiDrift;
    }

    public double getRotation() {
        return rotation;
    }

    public void setRotation(double rotation) {
        this.rotation = rotation;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
}
