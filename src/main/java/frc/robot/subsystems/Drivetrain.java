package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Types.Meters;
import frc.robot.Types.MetersPerSecond;
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
    public static final Meters TRACK_WIDTH = new Meters(.568325);

    // Motors
    private final MotorSubsystem leftMotor;
    private final MotorSubsystem rightMotor;

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
     * Set the current motor speed for the left motor in meters per second
     * 
     * @param speed New Target Motor Speed
     */
    public void setLeftMotorSpeed(MetersPerSecond speed) {
        leftMotor.setSpeed(speed);
    }

    /**
     * Set the current motor speed for the right motor in meters per second
     * 
     * @param speed New Target Motor Speed
     */
    public void setRightMotorSpeed(MetersPerSecond speed) {
        rightMotor.setSpeed(speed);
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
