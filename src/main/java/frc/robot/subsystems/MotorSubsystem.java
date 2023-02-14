package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
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
    private static final double GEAR_RATIO = 1 / 10.7;

    // Encoder pulses per rotation
    private static final int ENCODER_RESOLUTION = 2048;
    
    // Wheel geometry
    private static final Meters TRACK_WIDTH = new Meters(.568325);
    private static final Meters WHEEL_RADIUS = new Meters(.1524);
    private static final Meters WHEEL_CIRCUMFERENCE = new Meters(2 * Math.PI * WHEEL_RADIUS.getMeters());

    public MotorSubsystem (String name, PID pid, TalonFXInvertType invertType, MetersPerSecond tolerance, FeedForwardGains gains, WPI_TalonFX master, WPI_TalonFX... followers) {
        super(pid.toPIDController());

        // Feedforward gains
        feedforward = new SimpleMotorFeedforward(gains.getStaticGain(), gains.getVelocityGain(), gains.getAccelerationGain());

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
        SmartDashboard.putNumber(getName() + "Power", motor.getMotorOutputPercent());
        SmartDashboard.putNumber(getName() + "Speed", getEncoderVelocity(motor.getSelectedSensorVelocity()).getMetersPerSecond());
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.updateValues();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        
    }

    public void stop() {
        motor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void setLeftMotorSpeed(MetersPerSecond speed) {
        final double feedForward = feedforward.calculate(speed.getMetersPerSecond());

        final MotorPower output = new MotorPower(leftPidController.calculate(
                getEncoderVelocity(leftMaster.getSelectedSensorVelocity()).getMetersPerSecond(),
                speed.getMetersPerSecond()));

        leftMaster.set(ControlMode.PercentOutput, output.getValue() + feedForward);    }

}
