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

    private double heading = 0.0;
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

    /**
     * Get the drivetrain instance
     * 
     * @return drivetrain instance
     */
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
     * Get position lock status
     * The position lock should lock the robot in place
     * This feature is intended for the Rotate command
     * 
     * @return Position lock status
     */
    public boolean isPositionLock() {
        return positionLock;
    }

    /**
     * Turn the position lock on/off
     * The position lock should lock the robot in place
     * This feature is intended for the Rotate command
     * 
     * @param positionLock
     */
    public void setPositionLock(boolean positionLock) {
        this.positionLock = positionLock;
    }

    /**
     * Get rotation lock status
     * The rotation lock should close the loop on heading
     * 
     * @return Rotation lock status
     */
    public boolean isRotationLock() {
        return rotationLock;
    }

    /**
     * Turn rotation lock on/off
     * The rotation lock should close the loop on heading
     * 
     * @param rotationLock
     */
    public void setRotationLock(boolean rotationLock) {
        this.rotationLock = rotationLock;
    }

    /**
     * Get anti-drift status
     * The anti-drift system uses the strafe wheel to prevent drifting on turns
     * 
     * @return Anti-drift status
     */
    public boolean isAntiDrift() {
        return antiDrift;
    }

    /**
     * Turn anti-drift system on/off
     * The anti-drift system uses the strafe wheel to prevent drifting on turns
     * 
     * @param antiDrift
     */
    public void setAntiDrift(boolean antiDrift) {
        this.antiDrift = antiDrift;
    }

    /**
     * Get the current heading in degrees
     * 
     * @return the current heading in degrees
     */
    public double getHeading() {
        return heading;
    }

    /**
     * Set the target robot heading
     * 
     * @param heading
     */
    public void setHeading(double heading) {
        this.heading = heading;
    }

    /**
     * Get the current speed in m/s
     * 
     * @return Current speed in m/s
     */
    public double getSpeed() {
        return speed;
    }

    /**
     * Set the target robot speed in m/s
     * 
     * @param speed Target robot speed in m/s
     */
    public void setSpeed(double speed) {
        this.speed = speed;
    }
}
