package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    // Constants
    // Pidgeon hardware constant
    public static final int PIGEON_UNITS_PER_ROTATION = 8192;
    // Use 3600 units per rotation
    public static final int TURN_TRAVEL_UNITS_PER_ROTATION = 3600;
    // Allowable deadband
    public static final double NEUTRAL_DEADBAND = .001;
    // PID gains
    // We should be able to adjust these from the smart dashboard
    private int P_GAIN = 0;
    private int I_GAIN = 0;
    private int D_GAIN = 0;
    private int F_GAIN = 0;

    // Closed loop variables
    private boolean positionLockEngaged = false;
    private boolean rotationLockEngaged = false;
    private double rotation = 0;

    // Motors
    private final WPI_TalonFX leftMaster;
    private final WPI_TalonFX leftFollower;
    private final WPI_TalonFX rightMaster;
    private final WPI_TalonFX rightFollower;

    // Motor Configs
    TalonFXConfiguration leftConfig;
    TalonFXConfiguration rightConfig;

    private final WPI_Pigeon2 pidgey;

    private static WPI_TalonFX initMotor(int deviceNumber) {
        WPI_TalonFX motor = new WPI_TalonFX(deviceNumber);
        motor.configFactoryDefault();

        return motor;
    }

    private void initShuffleboard() {
        addChild("LeftMasterMotor", leftMaster);
        addChild("LeftFollowerMotor", leftFollower);
        addChild("RightMasterMotor", rightMaster);
        addChild("RightFollowerMotor", rightFollower);
    }

    public Drivetrain() {
        // initialize motors
        leftMaster = initMotor(1);
        leftFollower = initMotor(2);
        rightMaster = initMotor(3);
        rightFollower = initMotor(4);
        initShuffleboard();

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        // initialize pigeon
        pidgey = new WPI_Pigeon2(20);
        pidgey.configFactoryDefault();

        // Configure motor peak outputs
        leftConfig.peakOutputForward = 1;
        leftConfig.peakOutputReverse = -1;
        rightConfig.peakOutputForward = 1;
        rightConfig.peakOutputReverse = -1;

        // Right side is the ultimate master controller
        // Set pigeon as remote sensor
        rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw;
        rightConfig.remoteFilter0.remoteSensorDeviceID = pidgey.getDeviceID();

        // Use pigeon as the right talon's selected sensor
        rightConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();

        // Scale the pigeon's units to 3600 per rotation
        rightConfig.auxiliaryPID.selectedFeedbackCoefficient = TURN_TRAVEL_UNITS_PER_ROTATION
                / PIGEON_UNITS_PER_ROTATION;

        // We shouldn't need to set status frame periods unless we run into issues
        // Probably

        // Configure the neutral deadband
        // This probably sets the allowable error
        // If not, I blame the Phoenix docs
        rightConfig.neutralDeadband = NEUTRAL_DEADBAND;
        leftConfig.neutralDeadband = NEUTRAL_DEADBAND;

    }

    @Override
    public void periodic() {

    }

    public boolean isRotationLockEngaged() {
        return rotationLockEngaged;
    }

    public void setRotationLockEngaged(boolean rotationLockEngaged) {
        this.rotationLockEngaged = rotationLockEngaged;
    }

    public double getRotation() {
        return rotation;
    }

    public void setRotation(double rotation) {
        this.rotation = rotation;
    }

    public boolean isPositionLockEngaged() {
        return positionLockEngaged;
    }

    public void setPositionLockEngaged(boolean positionLockEngaged) {
        this.positionLockEngaged = positionLockEngaged;
    }

}
