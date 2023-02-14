package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Types.FeedForwardGains;
import frc.robot.Types.Meters;
import frc.robot.Types.MetersPerSecond;
import frc.robot.Types.MotorPower;
import frc.robot.Types.PID;
import frc.robot.Types.RadiansPerSecond;

/**
 * This subsystem controls four motors; two on each side of the robot.
 * 
 * One motor on each side will be the Leader and the other the Follower.
 * 
 * The Left follower will not be inverted; the right motor will.
 */
public class Drivetrain extends SubsystemBase {

    // Speed values
    public static final MetersPerSecond MAX_VELOCITY = new MetersPerSecond(3.0); // TODO: placeholder value
    public static final RadiansPerSecond MAX_ANGULAR_VELOCITY = new RadiansPerSecond(2 * Math.PI); // TODO: placeholder
                                                                                                   // value

    // Motors
    private final MotorSubsystem leftMotor;
    private final MotorSubsystem rightMotor;

    /**
     * Convenience method for initializing motors
     * 
     * @param deviceNumber CAN ID of motor
     * @return The configured motor
     */
    public static WPI_TalonFX initMotor(int deviceNumber) {
        WPI_TalonFX motor = new WPI_TalonFX(deviceNumber);
        motor.configFactoryDefault();
        motor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);

        FilterConfiguration filterConfig = new FilterConfiguration();
        filterConfig.remoteSensorSource = RemoteSensorSource.CANCoder;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.remoteFilter0 = filterConfig;

        return motor;
    }

    /**
     * Create a new drivetrain control subsystem.
     */
    public Drivetrain(MotorSubsystem leftMotor, MotorSubsystem rightMotor, NineAxis pigeon) {
        
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        
        stop();
    }

    @Override
    public void periodic() {
        SmartDashboard.updateValues();
    }

    /**
     * Stop driving the robot.
     */
    public void stop() {
        rightMotor.stop();
        leftMotor.stop();
    }

    /**
     * Get the current motor power for the left motor in percentage from [-1.0 to
     * 1.0]
     * 
     * @return Current left motor power
     */
    public MotorPower getLeftMotorPower() {
        return new MotorPower(leftMaster.getMotorOutputPercent());
    }

    /**
     * Get the current motor power for the Right motor in percentage from [-1.0 to
     * 1.0]
     * 
     * @return Current right motor power
     */
    public MotorPower getRightMotorPower() {
        return new MotorPower(rightMaster.getMotorOutputPercent());
    }

    /**
     * Set the current motor power for the left motor in percentage from [-1.0 to
     * 1.0]
     * 
     * @param power New Target Motor Power
     */
    public void setLeftMotorPower(MotorPower power) {
        leftMaster.set(ControlMode.PercentOutput, power.getValue());
    }

    /**
     * Set the current motor power for the Right motor in percentage from [-1.0 to
     * 1.0]
     * 
     * @param power New Target Motor Power
     */
    public void setRightMotorPower(MotorPower power) {
        rightMaster.set(ControlMode.PercentOutput, power.getValue());
    }

    /**
     * Set the motor power in percentage from [-1.0 to 1.0] for both motors at once.
     * 
     * @param leftMotorPower  New target motor power for left motor.
     * @param rightMotorPower New targe motor power for right motor.
     */
    public void setMotorPower(MotorPower leftMotorPower, MotorPower righMotorPower) {
        setLeftMotorPower(leftMotorPower);
        setRightMotorPower(righMotorPower);
    }

    /**
     * Set the current motor speed for the left motor in meters per second
     * 
     * @param speed New Target Motor Speed
     */
    public void setLeftMotorSpeed(MetersPerSecond speed) {

    }

    /**
     * Set the current motor speed for the right motor in meters per second
     * 
     * @param speed New Target Motor Speed
     */
    public void setRightMotorSpeed(MetersPerSecond speed) {
        final double feedForward = feedforward.calculate(speed.getMetersPerSecond());

        final MotorPower output = new MotorPower(rightPidController.calculate(
                getEncoderVelocity(rightMaster.getSelectedSensorVelocity()).getMetersPerSecond(),
                speed.getMetersPerSecond()));

        rightMaster.set(ControlMode.PercentOutput, output.getValue() + feedForward);
    }

    /**
     * Set the motor speed in meters per second
     * 
     * @param leftMotorSpeed  New target motor speed for left motor.
     * @param rightMotorSpeed New targe motor speed for right motor.
     */
    public void setMotorSpeed(MetersPerSecond leftMotorSpeed, MetersPerSecond righMotorSpeed) {
        setLeftMotorSpeed(leftMotorSpeed);
        setRightMotorSpeed(righMotorSpeed);
    }

    /**
     * Set the motor speed in meters per second
     * 
     * @param wheelSpeeds New target motor speeds for drivetrain.
     */
    public void setMotorSpeed(DifferentialDriveWheelSpeeds wheelSpeeds) {
        setMotorSpeed(new MetersPerSecond(wheelSpeeds.leftMetersPerSecond),
                new MetersPerSecond(wheelSpeeds.rightMetersPerSecond));
    }

}
