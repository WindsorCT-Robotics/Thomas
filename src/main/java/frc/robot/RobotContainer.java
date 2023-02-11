// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Types.Milliseconds;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NineAxis;

public class RobotContainer {
    // Subsystems
    private final NineAxis pidgey;
    private final Drivetrain drive;

    // Controller
    private final DriveController driveController;
    private final double DEADZONE = 0.25; // TODO: placeholder value
    private final double POSITIVE_RATE_LIMIT = 0.3; // TODO: placeholder value
    private final double NEGATIVE_RATE_LIMIT = 0.1; // TODO: placeholder value

    public RobotContainer() {

        pidgey = new NineAxis(new WPI_Pigeon2(20), new Milliseconds((30)));

        drive = new Drivetrain(
                Drivetrain.initMotor(1),
                Drivetrain.initMotor(2),
                Drivetrain.initMotor(3),
                Drivetrain.initMotor(4),
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
