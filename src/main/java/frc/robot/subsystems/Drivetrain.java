package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Gains;

public class Drivetrain extends SubsystemBase {
    // Motors
    private final WPI_TalonFX leftMaster;
    private final WPI_TalonFX leftFollower;
    private final WPI_TalonFX rightMaster;
    private final WPI_TalonFX rightFollower;

    // Motor configs
    private final TalonFXConfiguration leftConfig;
    private final TalonFXConfiguration rightConfig;

    // Motor directions
    private final TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise;
    private final TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise;

    // Gyroscope
    private final WPI_Pigeon2 pidgey;

    // Neutral deadband
    private final double neutralDeadband = .001;

    // Timeout ms for sensors
    private final int timeoutMs = 30;

    // Named hardware slots
    private final static int SLOT_DISTANCE = 0;
    private final static int SLOT_TURNING = 1;

    // We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
    // is auxiliary
    private final static int PID_PRIMARY = 0;
    private final static int PID_TURN = 1;

    // Control variables
    private double speed = 0.0;
    private boolean positionLock = false;
    private boolean antiDrift = false;

    private double absoluteHeading = 0.0;
    private boolean rotationLock = false;

    private static Drivetrain drive;

    private final Gains gainsDistance;
    private final Gains gainsTurning;

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

        // Create motor configs
        leftConfig = new TalonFXConfiguration();
        rightConfig = new TalonFXConfiguration();

        // Set master and follower motors
        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        // Set motor turn directions
        leftMaster.setInverted(leftInvert);
        rightMaster.setInverted(rightInvert);

        // Initialize Pigeon 2.0, a 9-axis combined accelerometer, gyroscope, and
        // magnetometer
        pidgey = new WPI_Pigeon2(20);
        pidgey.configFactoryDefault();

        // Disable all motors
        stop();

        // Set motors to brake mode
        setNeutralMode(NeutralMode.Brake);

        // Set left talon's encoder as its primary feedback device
        leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

        // Set left feedback sensor as right's remote sensor
        rightConfig.remoteFilter0.remoteSensorDeviceID = leftMaster.getDeviceID();
        rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;

        setRobotDistanceConfigs(rightInvert, rightConfig);

        // set pid distance gains
        gainsDistance = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
        rightConfig.slot3.kP = gainsDistance.kP;
        rightConfig.slot3.kI = gainsDistance.kI;
        rightConfig.slot3.kD = gainsDistance.kD;
        rightConfig.slot3.kF = gainsDistance.kF;
        rightConfig.slot3.integralZone = gainsDistance.integralZone;
        rightConfig.slot3.closedLoopPeakOutput = gainsDistance.peakOutput;

        // Yaw configs
        rightConfig.remoteFilter1.remoteSensorDeviceID = pidgey.getDeviceID(); // Pigeon Device ID
        // This is for a Pigeon over CAN
        rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw;
        // Set as the Aux Sensor
        rightConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();
        // Convert Yaw to tenths of a degree (8192 pigeon units per rotation)
        rightConfig.auxiliaryPID.selectedFeedbackCoefficient = 3600.0 / 8192;

        // Yaw gains
        gainsTurning = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.00);
        rightConfig.slot1.kF = gainsTurning.kF;
        rightConfig.slot1.kP = gainsTurning.kP;
        rightConfig.slot1.kI = gainsTurning.kI;
        rightConfig.slot1.kD = gainsTurning.kD;
        rightConfig.slot1.integralZone = gainsTurning.integralZone;
        rightConfig.slot1.closedLoopPeakOutput = gainsTurning.peakOutput;

        // Set peak output for all modes
        leftConfig.peakOutputForward = +1.0;
        leftConfig.peakOutputReverse = -1.0;
        rightConfig.peakOutputForward = +1.0;
        rightConfig.peakOutputReverse = -1.0;
        // Neutral Deadband
        // This is the allowable error
        leftConfig.neutralDeadband = neutralDeadband;
        rightConfig.neutralDeadband = neutralDeadband;

        // Loop speed
        int closedLoopTimeMs = 1;
        rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
        rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
        rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
        rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

        // Apply settings to master motors
        leftMaster.configAllSettings(leftConfig);
        rightMaster.configAllSettings(rightConfig);

        // Set status frame periods to avoid flooding the CAN bus
        rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, timeoutMs);
        rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, timeoutMs);
        rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, timeoutMs);
        rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, timeoutMs);
        leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, timeoutMs);
        pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5, timeoutMs);

        zeroSensors();
        zeroDistance();

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
        double targetAngle = absoluteHeading;
        double targetSensorUnits = speed * 2048 * 6;

        rightMaster.set(ControlMode.MotionMagic, targetSensorUnits, DemandType.AuxPID, targetAngle);
        leftMaster.follow(rightMaster, FollowerType.AuxOutput1);

    }

    /**
     * Zeroes all drivetrain sensors
     */
    private void zeroSensors() {
        leftMaster.getSensorCollection().setIntegratedSensorPosition(0, timeoutMs);
        rightMaster.getSensorCollection().setIntegratedSensorPosition(0, timeoutMs);
        pidgey.setYaw(0, timeoutMs);
        pidgey.setAccumZAngle(0, timeoutMs);
    }

    /**
     * Zeroes encoder distance
     */
    private void zeroDistance() {
        leftMaster.getSensorCollection().setIntegratedSensorPosition(0, timeoutMs);
        rightMaster.getSensorCollection().setIntegratedSensorPosition(0, timeoutMs);
    }

    /**
     * Copied from CTRE example project:
     *
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
     * @param masterConfig     Config of master motor
     */
    private static void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig) {
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
     * Get the current absolute heading in degrees
     * 
     * @return the current absolute heading in degrees
     */
    public double getAbsoluteHeading() {
        return absoluteHeading;
    }

    /**
     * Set the target robot absolute heading
     * 
     * @param absoluteHeading
     */
    public void setAbsoluteHeading(double absoluteHeading) {
        this.absoluteHeading = absoluteHeading;
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
