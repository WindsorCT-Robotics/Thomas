package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Types.MotorPower;

public class Drivetrain extends SubsystemBase {
    // Motors
    private final WPI_TalonFX leftMaster;
    private final WPI_TalonFX leftFollower;
    private final WPI_TalonFX rightMaster;
    private final WPI_TalonFX rightFollower;

    // Motor directions
    private final TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise;
    private final TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise;

    /**
     * Convenience method for initializing motors
     * 
     * @param deviceNumber CAN ID of motor
     * @return The configured motor
     */
    private static WPI_TalonFX initMotor(int deviceNumber) {
        WPI_TalonFX motor = new WPI_TalonFX(deviceNumber);
        motor.configFactoryDefault();

        return motor;
    }

    public Drivetrain() {
        // Initialize motors
        leftMaster = initMotor(1);
        leftFollower = initMotor(2);
        rightMaster = initMotor(3);
        rightFollower = initMotor(4);

        // Set master and follower motors
        leftFollower.follow(leftMaster);
        leftFollower.setInverted(InvertType.FollowMaster);
        rightFollower.follow(rightMaster);
        rightFollower.setInverted(InvertType.FollowMaster);

        // Set motor turn directions
        leftMaster.setInverted(leftInvert);
        rightMaster.setInverted(rightInvert);

        stop();
    }

    @Override
    public void periodic() {
        SmartDashboard.updateValues();

        // leftMaster.set(TalonFXControlMode.PercentOutput, rotationLock ? speed +
        // controllerAdjustment : speed);
        // rightMaster.set(TalonFXControlMode.PercentOutput, rotationLock ? speed -
        // controllerAdjustment : speed);
        SmartDashboard.putNumber("Left Motor Power",  leftMaster.getMotorOutputPercent()  * 100);
        SmartDashboard.putNumber("Right Motor Power", rightMaster.getMotorOutputPercent() * 100);

    }

    /**
     * Stop this subsystem
     */
    public void stop() {
        rightMaster.set(TalonFXControlMode.PercentOutput, 0);
        leftMaster.set(TalonFXControlMode.PercentOutput, 0);
    }

    /**
     * Set neutral mode of the left and right drivetrain motors
     * 
     * @param neutralMode Target neutral mode - brake or coast - of drivetrain
     */
    public void setNeutralMode(NeutralMode neutralMode) {
        leftMaster.setNeutralMode(neutralMode);
        rightMaster.setNeutralMode(neutralMode);
    }

    /**
     * Get the current motor power for the left motor in percentage from [0.0 to 1.0]
     * 
     * @return Current left motor power
     */
    public MotorPower getLeftMotorPower() {
        return new MotorPower(leftMaster.getMotorOutputPercent());
    }

    /**
     * Get the current motor power for the Right motor in percentage from [0.0 to 1.0]
     * 
     * @return Current right motor power
     */
    public MotorPower getRightMotorPower() {
        return new MotorPower(rightMaster.getMotorOutputPercent());
    }

    /**
     * Set the current motor power for the left motor in percentage from [0.0 to 1.0]
     * @param power New Target Motor Power
     */
    public void setLeftMotorPower(MotorPower power) {
        leftMaster.set(ControlMode.PercentOutput, power.getValue());
    }

    /**
     * Set the current motor power for the Right motor in percentage from [0.0 to 1.0]
     * @param power New Target Motor Power
     */
    public void setRightMotorPower(MotorPower power) {
        rightMaster.set(ControlMode.PercentOutput, power.getValue());
    }

    public void setMotorPower (MotorPower leftMotorPower, MotorPower righMotorPower) {
        setLeftMotorPower (leftMotorPower);
        setRightMotorPower(righMotorPower);
    }
}
