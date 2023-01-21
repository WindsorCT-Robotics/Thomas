// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  public final Drivetrain drive = Drivetrain.getInstance();
  private final XboxController driveController = new XboxController(0);

  public RobotContainer() {
    configureBindings();
    drive.setDefaultCommand(new Drive());
  }

  private void configureBindings() {
    // Robot state while moving
    final Trigger driveState = new Trigger(() -> applyJoystickDeadzone(driveController.getLeftY(), 0.05) != 0);
    driveState.whileTrue(new Drive());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private static double applyJoystickDeadzone(double rawValue, double deadzone) {
    double result;
    double validRange = 1 - deadzone;
    double value = Math.abs(rawValue);

    if (value > deadzone) {
      result = (value - deadzone) / validRange;
    } else {
      result = 0;
    }

    return rawValue < 0 ? -result : result;
  }
}
