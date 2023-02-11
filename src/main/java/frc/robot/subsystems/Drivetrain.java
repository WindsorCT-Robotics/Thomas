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
    // Motor directions
    public final TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise;
    public final TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise;

    // Wheel geometry
    public static final Meters TRACK_WIDTH = new Meters(.568325);
    public static final Meters WHEEL_RADIUS = new Meters(.1524);
    public static final Meters WHEEL_CIRCUMFERENCE = new Meters(2 * Math.PI * WHEEL_RADIUS.getMeters());

    // PID gains
    public static final PID LEFT_GAINS = new PID(1, 0, 0); // TODO: placeholder value
    public static final PID RIGHT_GAINS = new PID(1, 0, 0); // TODO: placeholder value

    // 10.7:1 gear ratio: 1 gearbox revolution for every 10.7 motor revolutions
    public static final double GEAR_RATIO = 1 / 10.7;

    // Encoder pulses per rotation
    public static final int ENCODER_RESOLUTION = 2048;

    // Speed values
    public static final MetersPerSecond MAX_VELOCITY = new MetersPerSecond(3.0); // TODO: placeholder value
    public static final RadiansPerSecond MAX_ANGULAR_VELOCITY = new RadiansPerSecond(2 * Math.PI); // TODO: placeholder
                                                                                                   // value

    // Motors
    private final WPI_TalonFX leftMaster;
    private final WPI_TalonFX rightMaster;

    private final SimpleMotorFeedforward feedforward;

    // PID controllers
    private final PIDController leftPidController;
    private final PIDController rightPidController;

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

    private static PIDController initDrivePIDController(PID pid, MetersPerSecond tolerance) {
        PIDController driveController = new PIDController(pid.getP(), pid.getI(), pid.getD());
        driveController.reset();
        driveController.setTolerance(tolerance.getMetersPerSecond());

        return driveController;
    }

    private static void setFollower(WPI_TalonFX leader, WPI_TalonFX follower) {
        follower.follow(leader);
        follower.setInverted(InvertType.FollowMaster);
    }

    /**
     * Get the wheel's velocity in meters per second
     * 
     * @param encoderVelocity sensor units per 100 ms, as provided by
     *                        {@link com.ctre.phoenix.motorcontrol.can.BaseMotorController#getSelectedSensorPosition()
     *                        getSelectedSensorPosition}
     * @return Wheel velocity in meters per second
     */
    private static MetersPerSecond getEncoderVelocity(double encoderVelocity) {
        final double encoderUnitsPerMeter = ENCODER_RESOLUTION * GEAR_RATIO * WHEEL_CIRCUMFERENCE.getMeters();
        // Tenths of a second to seconds
        final double tenthsToSeconds = 10;
        return new MetersPerSecond(encoderVelocity * (tenthsToSeconds / encoderUnitsPerMeter));
    }

    /**
     * Create a new drivetrain control subsystem.
     */
    public Drivetrain(WPI_TalonFX leftMaster, WPI_TalonFX leftFollower, WPI_TalonFX rightMaster,
            WPI_TalonFX rightFollower, NineAxis pigeon) {
        // Initialize motors
        this.leftMaster = leftMaster;
        this.rightMaster = rightMaster;

        addChild("Left Master Motor", leftMaster);
        addChild("Left Follower Motor", leftFollower);
        addChild("Right Master Motor", rightMaster);
        addChild("Right Follower Motor", rightFollower);

        // Set master and follower motors
        setFollower(leftMaster, leftFollower);
        setFollower(rightMaster, rightFollower);

        // Set motor turn directions
        leftMaster.setInverted(leftInvert);
        rightMaster.setInverted(rightInvert);

        // Feedforward gains
        feedforward = new SimpleMotorFeedforward(0.18157, 2.3447, 0.54597);

        // initialize PID controllers
        MetersPerSecond tolerance = new MetersPerSecond(1.2601);
        leftPidController = initDrivePIDController(LEFT_GAINS, tolerance);
        rightPidController = initDrivePIDController(RIGHT_GAINS, tolerance);

        addChild("Left PID controller", leftPidController);
        addChild("Right PID controller", rightPidController);

        stop();
    }

    @Override
    public void periodic() {
        SmartDashboard.updateValues();
        doTelemetry();

    }

    /**
     * Output telemetry data
     */
    private void doTelemetry() {
        SmartDashboard.putNumber("Left Motor Power", getLeftMotorPower().getValue() * 100);
        SmartDashboard.putNumber("Right Motor Power", getRightMotorPower().getValue() * 100);

        SmartDashboard.putNumber("Left Motor Speed", getLeftMotorSpeed().getMetersPerSecond());
        SmartDashboard.putNumber("Right Motor Speed", getRightMotorSpeed().getMetersPerSecond());
    }

    /**
     * Stop driving the robot.
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
     * Get the current motor speed in meters per second for the left motor
     * 
     * @return Current left motor speed
     */
    public MetersPerSecond getLeftMotorSpeed() {
        return getEncoderVelocity(leftMaster.getSelectedSensorVelocity());
    }

    /**
     * Get the current motor speed in meters per second for the right motor
     * 
     * @return Current right motor speed
     */
    public MetersPerSecond getRightMotorSpeed() {
        return getEncoderVelocity(rightMaster.getSelectedSensorVelocity());
    }

    /**
     * Set the current motor speed for the left motor in meters per second
     * 
     * @param speed New Target Motor Speed
     */
    public void setLeftMotorSpeed(MetersPerSecond speed) {
        final double feedForward = feedforward.calculate(speed.getMetersPerSecond());

        final MotorPower output = new MotorPower(leftPidController.calculate(
                getEncoderVelocity(leftMaster.getSelectedSensorVelocity()).getMetersPerSecond(),
                speed.getMetersPerSecond()));

        leftMaster.set(ControlMode.PercentOutput, output.getValue() + feedForward);
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
