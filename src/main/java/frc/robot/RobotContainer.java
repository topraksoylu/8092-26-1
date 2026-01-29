// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Commands.DriveCommand;
import frc.robot.Constants.*;

public class RobotContainer {
  private DriveSubsystem driveSubsystem = new DriveSubsystem(
      MotorConstants.REAR_LEFT_MOTOR_ID,
      MotorConstants.FRONT_LEFT_MOTOR_ID,
      MotorConstants.REAR_RIGHT_MOTOR_ID,
      MotorConstants.FRONT_RIGHT_MOTOR_ID
  );

  private Joystick driverJoystick = new Joystick(0);

  public RobotContainer() {
    configureBindings();
    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            () -> driverJoystick.getY(),
            () -> driverJoystick.getX(),
            () -> driverJoystick.getZ(),
            driveSubsystem
        )
    );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
