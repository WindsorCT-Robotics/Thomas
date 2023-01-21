package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    // Motors
    private final WPI_TalonFX leftMaster;
    private final WPI_TalonFX leftFollower;
    private final WPI_TalonFX rightMaster;
    private final WPI_TalonFX rightFollower;

    private final TalonFXConfiguration leftConfig;
    private final TalonFXConfiguration rightConfig;

    private final TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise;
    private final TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise;

    // Gyroscope
    private final WPI_Pigeon2 pidgey;

    // Control variables
    private double speed = 0.0;
    private boolean positionLock = false;
    private boolean antiDrift = false;

    private double yaw = 0.0;
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

        leftConfig = new TalonFXConfiguration();
        rightConfig = new TalonFXConfiguration();

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        leftMaster.setInverted(leftInvert);
        rightMaster.setInverted(rightInvert);

        pidgey = new WPI_Pigeon2(20);
        pidgey.configFactoryDefault();

        leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

        rightConfig.remoteFilter0.remoteSensorDeviceID = leftMaster.getDeviceID();
        rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;

        setRobotDistanceConfigs(rightInvert, rightConfig);

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
     * Determine if we need a Sum or Difference.
     * 
     * The auxiliary Talon FX will always be positive
     * in the forward direction because it's a selected sensor
     * over the CAN bus.
     * 
     * The master's native integrated sensor may not always be positive when forward
     * because
     * sensor phase is only applied to *Selected Sensors*, not native
     * sensor sources. And we need the native to be combined with the
     * aux (other side's) distance into a single robot distance.
     * 
     * @param masterInvertType Rotation direction of the master motor
     * @param masterConfig Config of master motor
     */
    void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig) {
        /*
         * THIS FUNCTION should not need to be modified.
         * This setup will work regardless of whether the master
         * is on the Right or Left side since it only deals with
         * distance magnitude.
         */

        /* Check if we're inverted */
        if (masterInvertType == TalonFXInvertType.Clockwise) {
            /*
             * If master is inverted, that means the integrated sensor
             * will be negative in the forward direction.
             * If master is inverted, the final sum/diff result will also be inverted.
             * This is how Talon FX corrects the sensor phase when inverting
             * the motor direction. This inversion applies to the *Selected Sensor*,
             * not the native value.
             * Will a sensor sum or difference give us a positive total magnitude?
             * Remember the Master is one side of your drivetrain distance and
             * Auxiliary is the other side's distance.
             * Phase | Term 0 | Term 1 | Result
             * Sum: -((-)Master + (+)Aux )| NOT OK, will cancel each other out
             * Diff: -((-)Master - (+)Aux )| OK - This is what we want, magnitude will be
             * correct and positive.
             * Diff: -((+)Aux - (-)Master)| NOT OK, magnitude will be correct but negative
             */

            masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local Integrated
                                                                                                // Sensor
            masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
            masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); // Diff0
                                                                                                                        // -
                                                                                                                        // Diff1
        } else {
            /* Master is not inverted, both sides are positive so we can sum them. */
            masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); // Aux Selected Sensor
            masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); // Local IntegratedSensor
            masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); // Sum0
                                                                                                                 // +
                                                                                                                 // Sum1
        }

        /*
         * Since the Distance is the sum of the two sides, divide by 2 so the total
         * isn't double
         * the real-world value
         */
        masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
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
     * @param rotationLock rotation lock on/off
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
     * @param antiDrift anti-drift on/off
     */
    public void setAntiDrift(boolean antiDrift) {
        this.antiDrift = antiDrift;
    }

    /**
     * Get the current yaw in degrees
     * 
     * @return the current yaw in degrees
     */
    public double getYaw() {
        return yaw;
    }

    /**
     * Set the target robot yaw
     * 
     * @param yaw
     */
    public void setYaw(double yaw) {
        this.yaw = yaw;
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
