package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Types.FeedForwardGains;
import frc.robot.Types.Meters;
import frc.robot.Types.MetersPerSecond;
import frc.robot.Types.MotorPower;
import frc.robot.Types.PID;

public class MotorSubsystem extends PIDSubsystem {
    private WPI_TalonFX motor;
    private SimpleMotorFeedforward feedforward;
    
    // 10.7:1 gear ratio: 1 gearbox revolution for every 10.7 motor revolutions
    public static final double GEAR_RATIO = 1 / 10.7;

    // Encoder pulses per rotation
    public static final int ENCODER_RESOLUTION = 2048;
    
    // Wheel geometry
    private static final Meters WHEEL_RADIUS = new Meters(.1524);
    private static final Meters WHEEL_CIRCUMFERENCE = new Meters(2 * Math.PI * WHEEL_RADIUS.getMeters());

    // controller
    PIDController controller;

    private final DoubleEntry proportional;
    private final DoubleEntry integral;
    private final DoubleEntry differential;
    private final DoubleEntry setpoint;
    private final DoubleEntry motorPowerOutputPercent;
    private final DoubleEntry motorSpeed;
    private final BooleanPublisher atSetpoint;
    private final BooleanEntry isEnabled;

    private boolean isSubscribed;

    public MotorSubsystem (NetworkTableInstance nt_instance, String name, PID pid, TalonFXInvertType invertType, MetersPerSecond tolerance, FeedForwardGains gains, WPI_TalonFX master, WPI_TalonFX... followers) {
        super(pid.toPIDController());
        controller = super.getController();

        // Feedforward gains
        feedforward = gains.toSimpleMotorFeedforward();

        setName(name);
        controller.reset();
        controller.setTolerance(tolerance.getMetersPerSecond());
        
        NetworkTable motorTable = nt_instance.getTable(name);

        proportional = motorTable.getDoubleTopic("Proportional").getEntry(controller.getP());
        integral = motorTable.getDoubleTopic("Integral").getEntry(controller.getI());
        differential = motorTable.getDoubleTopic("Differential").getEntry(controller.getD());
        setpoint = motorTable.getDoubleTopic("Setpoint").getEntry(controller.getSetpoint());
        atSetpoint = motorTable.getBooleanTopic("Is At Setpoint?").publish(PubSubOption.topicsOnly(true));
        motorPowerOutputPercent = motorTable.getDoubleTopic("Motor Power Output (%)").getEntry(0);
        motorSpeed = motorTable.getDoubleTopic("Motor Speed (Meters/sec)").getEntry(0);
        isEnabled = motorTable.getBooleanTopic("Enabled").getEntry(m_enabled);

        master.setInverted(invertType);
        
        for (WPI_TalonFX motor:followers) {
            motor.follow(master);
            motor.setInverted(TalonFXInvertType.FollowMaster);
        }

        this.motor = master;
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

    @Override
    protected double getMeasurement() {
        return getEncoderVelocity(motor.getSelectedSensorPosition()).getMetersPerSecond();
    }

    @Override
    public void periodic() {
        if (isSubscribed) {
            m_enabled = isEnabled.getAsBoolean();
            controller.setP(proportional.getAsDouble());
            controller.setI(integral.getAsDouble());
            controller.setD(differential.getAsDouble());
            controller.setSetpoint(setpoint.getAsDouble());
            motor.set(TalonFXControlMode.PercentOutput, motorPowerOutputPercent.getAsDouble());
            setSpeed(new MetersPerSecond(motorSpeed.getAsDouble()));
        }

        isEnabled.set(m_enabled);
        
        proportional.set(controller.getP());
        integral.set(controller.getI());
        differential.set(controller.getD());
        setpoint.set(controller.getSetpoint());
        super.periodic();

        atSetpoint.set(controller.atSetpoint());
        motorPowerOutputPercent.set((motor.getMotorOutputPercent()));
        motorSpeed.set(getMeasurement());
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        final MotorPower feedForward = new MotorPower(feedforward.calculate(setpoint));

        final MotorPower speed = new MotorPower(output);

        motor.set(ControlMode.PercentOutput, MotorPower.add(feedForward, speed).getValue());
        
    }

    public void setSpeed(MetersPerSecond speed) {
        getController().setSetpoint(speed.getMetersPerSecond());
    }

    public void stop() {
        motor.set(TalonFXControlMode.PercentOutput, 0);
    }

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
     * Allows NetworkTable values to update class values. This is generally only set in Test Mode.
     * @param subscribe If true, changes in the network table will reflect in the class.
     */
    public void SubscribeToNetworkTables(boolean subscribe) {
        isSubscribed = subscribe;
    }
}
