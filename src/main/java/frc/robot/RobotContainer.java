// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Types.FeedForwardGains;
import frc.robot.Types.MetersPerSecond;
import frc.robot.Types.Milliseconds;
import frc.robot.Types.PID;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.NineAxis;

public class RobotContainer {
        // Subsystems
        private final NineAxis pidgey;
        private final Drivetrain drive;
        private final MotorSubsystem leftMotors;
        private final MotorSubsystem rightMotors;

        // Controller
        private final DriveController driveController;
        private final double DEADZONE = 0.25; // TODO: placeholder value
        private final double POSITIVE_RATE_LIMIT = 0.3; // TODO: placeholder value
        private final double NEGATIVE_RATE_LIMIT = 0.1; // TODO: placeholder value

        // Drivetrain values
        private final MetersPerSecond threshold = new MetersPerSecond(0.01);
        private final FeedForwardGains gains = new FeedForwardGains(0.18157, 2.3447, 0.54597);

        public RobotContainer() {

                // Initialize subsystems
                // TODO: Placeholder PID values
                leftMotors = new MotorSubsystem("Left Motors", new PID(1, 0, 0), TalonFXInvertType.Clockwise,
                                threshold, gains,
                                MotorSubsystem.initMotor(1),
                                MotorSubsystem.initMotor(2));

                // TODO: Placeholder PID values
                rightMotors = new MotorSubsystem("Right Motors", new PID(1, 0, 0), TalonFXInvertType.CounterClockwise,
                                threshold, gains,
                                MotorSubsystem.initMotor(3),
                                MotorSubsystem.initMotor(4));

                pidgey = new NineAxis(new WPI_Pigeon2(20), new Milliseconds((30)));

                drive = new Drivetrain(
                                leftMotors,
                                rightMotors,
                                pidgey);

                driveController = new DriveController(0, DEADZONE, POSITIVE_RATE_LIMIT, NEGATIVE_RATE_LIMIT);

                Drive driveCommand = new Drive(
                                drive,
                                pidgey,
                                () -> driveController.getSpeed(),
                                () -> driveController.getAngle());

                configureBindings();

                drive.setDefaultCommand(driveCommand);

        }

        private void configureBindings() {
        }

        public Command getAutonomousCommand() {
                return Commands.print("No autonomous command configured");
        }

}
