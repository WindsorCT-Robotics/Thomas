package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    public MotorSubsystem (String name, PID pid, TalonFXInvertType invertType, MetersPerSecond tolerance, FeedForwardGains gains, WPI_TalonFX master, WPI_TalonFX... followers) {
        super(pid.toPIDController());

        // Feedforward gains
        feedforward = gains.toSimpleMotorFeedforward();

        setName(name);
        getController().reset();
        getController().setTolerance(tolerance.getMetersPerSecond());

        for (WPI_TalonFX motor:followers) {
            motor.follow(master);
        }

        master.setInverted(invertType);

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

    /**
     * Output telemetry data
     */
    private void doTelemetry() {
        SmartDashboard.putNumber(getName() + " Power", motor.getMotorOutputPercent());
        SmartDashboard.putNumber(getName() + " Speed", getEncoderVelocity(motor.getSelectedSensorVelocity()).getMetersPerSecond());
    }

    @Override
    protected double getMeasurement() {
        return getEncoderVelocity(motor.getSelectedSensorPosition()).getMetersPerSecond();
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.updateValues();
        doTelemetry();
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


}
