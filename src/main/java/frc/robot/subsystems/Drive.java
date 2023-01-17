package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final WPI_TalonFX leftMaster;
    private final WPI_TalonFX leftFollower;
    private final WPI_TalonFX rightMaster;
    private final WPI_TalonFX rightFollower;
    private final WPI_Pigeon2 pidgey;

    private final DifferentialDrive drive;

    private static WPI_TalonFX initMotor(int deviceNumber) {
        WPI_TalonFX motor = new WPI_TalonFX(deviceNumber);
        motor.configFactoryDefault();

        return motor;
    }

    private void initShuffleboard() {
        addChild("LeftMasterMotor", leftMaster);
        addChild("LeftFollowerMotor", leftFollower);
        addChild("RightMasterMotor", rightMaster);
        addChild("RightFollowerMotor", rightFollower);
    }

    public Drive() {
        leftMaster = initMotor(1);
        leftFollower = initMotor(2);
        rightMaster = initMotor(3);
        rightFollower = initMotor(4);
        initShuffleboard();

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        pidgey = new WPI_Pigeon2(20);

        drive = new DifferentialDrive(leftMaster, rightMaster);

        
    }
}
