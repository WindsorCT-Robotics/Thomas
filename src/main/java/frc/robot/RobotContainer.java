// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Types.Milliseconds;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NineAxis;

public class RobotContainer {
    private final Drivetrain drive;
    private final CommandXboxController driveController;
    private final NineAxis pidgey;

  public RobotContainer() {
    double controllerDeadzonePercent = 0.3;

    drive = new Drivetrain(
        Drivetrain.initMotor(1),
        Drivetrain.initMotor(2),
        Drivetrain.initMotor(3),
        Drivetrain.initMotor(4)
    );

    driveController = new CommandXboxController(0);

    pidgey = new NineAxis(new WPI_Pigeon2(20), new Milliseconds((30)));

    Drive driveCommand = new Drive(
        drive,
        deadzoneModifier(() -> driveController.getRightX(), controllerDeadzonePercent),
        deadzoneModifier(() -> driveController.getLeftY(),  controllerDeadzonePercent));

    configureBindings();

    drive.setDefaultCommand(
        driveCommand);
        
  }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private static DoubleSupplier deadzoneModifier(DoubleSupplier rawValueSupplier, double deadzone) {
        return () -> {
            double rawValue = rawValueSupplier.getAsDouble();
            double result;
            double validRange = 1 - deadzone;
            double value = Math.abs(rawValue);

            if (value > deadzone) {
                result = (value - deadzone) / validRange;
            } else {
                result = 0;
            }

            return rawValue < 0 ? -result : result;
        };
    }
}
