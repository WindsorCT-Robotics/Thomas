package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Types.DegreesPerSecond;
import frc.robot.Types.Meters;
import frc.robot.Types.MetersPerSecond;
import frc.robot.Types.MotorPower;
import frc.robot.Types.PID;

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
    public static final PID LEFT_GAINS = new PID(1, 0, 0);
    public static final PID RIGHT_GAINS = new PID(1, 0, 0);

    // Gear ratio
    public static final double GEAR_RATIO = 4 / 3; // TODO: placeholder value

    // Encoder pulses per rotation
    public static final int ENCODER_RESOLUTION = 2048;

    // Speed values
    private static final MetersPerSecond MAX_SPEED = new MetersPerSecond(3.0); // TODO: placeholder value
    private static final DegreesPerSecond MAX_ANGULAR_SPEED = new DegreesPerSecond(360); // TODO: placeholder value

    // Motors
    private final WPI_TalonFX leftMaster;
    private final WPI_TalonFX rightMaster;

    // Nine-axis motion sensor
    private final NineAxis pidgey;

    private final DifferentialDriveKinematics kinematics;

    private final DifferentialDriveOdometry odometry;

    private final SimpleMotorFeedforward feedforward;

    // PID controllers
    private final PIDController lefPidController;
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

        return motor;
    }

    private static void setFollower(WPI_TalonFX leader, WPI_TalonFX follower) {
        follower.follow(leader);
        follower.setInverted(InvertType.FollowMaster);
    }

    private static Meters getEncoderDistance(double encoderDistance) {
        return new Meters(WHEEL_CIRCUMFERENCE.getMeters() / encoderDistance * GEAR_RATIO);
    }

    /**
     * Get the wheel's velocity in meters per second
     * 
     * @param encoderVelocity sensor units per 100 ms, as provided by
     *                        {@link com.ctre.phoenix.motorcontrol.can.BaseMotorController#getSelectedSensorPosition()}
     * @return Wheel velocity in meters per second
     */
    private static MetersPerSecond getEncoderVelocity(double encoderVelocity) {
        double unitsPerMeter = ENCODER_RESOLUTION * GEAR_RATIO * WHEEL_CIRCUMFERENCE.getMeters();
        return new MetersPerSecond(encoderVelocity / (10 * unitsPerMeter));
    }

    /**
     * Create a new drivetrain control subsystem.
     */
    public Drivetrain(WPI_TalonFX leftLeader, WPI_TalonFX leftFollower, WPI_TalonFX rightLeader,
            WPI_TalonFX rightFollower, NineAxis pigeon) {
        // Initialize motors
        leftMaster = leftLeader;
        rightMaster = rightLeader;

        // Set master and follower motors
        setFollower(leftMaster, leftFollower);
        setFollower(rightMaster, rightFollower);

        // Set motor turn directions
        leftMaster.setInverted(leftInvert);
        rightMaster.setInverted(rightInvert);

        // Initialize Pigeon 2.0
        pidgey = pigeon;

        // Feedforward gains
        feedforward = new SimpleMotorFeedforward(1, 3); // TODO: placeholder value

        // Initialize kinematics
        kinematics = new DifferentialDriveKinematics(TRACK_WIDTH.getMeters());

        // initialize odometry
        odometry = new DifferentialDriveOdometry(
                pidgey.getYaw(), leftMaster.getSelectedSensorPosition(), rightMaster.getSelectedSensorPosition());

        // initialize PID controllers
        lefPidController = new PIDController(LEFT_GAINS.getP(), LEFT_GAINS.getI(), LEFT_GAINS.getD());
        rightPidController = new PIDController(RIGHT_GAINS.getP(), RIGHT_GAINS.getI(), RIGHT_GAINS.getD());

        stop();
    }

    @Override
    public void periodic() {
        SmartDashboard.updateValues();

        SmartDashboard.putNumber("Left Motor Power", leftMaster.getMotorOutputPercent() * 100);
        SmartDashboard.putNumber("Right Motor Power", rightMaster.getMotorOutputPercent() * 100);

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
}
