// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Commands.DriveCommand;
import frc.robot.Constants.*;

public class RobotContainer {
  private DriveSubsystem driveSubsystem = new DriveSubsystem(
      MotorConstants.REAR_LEFT_MOTOR_ID,
      MotorConstants.FRONT_LEFT_MOTOR_ID,
      MotorConstants.REAR_RIGHT_MOTOR_ID,
      MotorConstants.FRONT_RIGHT_MOTOR_ID,
      new Pose2d()
  );

  private Joystick driverJoystick = new Joystick(0);

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            () -> driverJoystick.getY(),
            () -> driverJoystick.getX(),
            () -> -driverJoystick.getZ(),
            driveSubsystem
        )
    );

    try {
      autoChooser = AutoBuilder.buildAutoChooser();
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner autos: " + e.getMessage(), false);
      autoChooser = new SendableChooser<>();
      // Add a default "None" option
      autoChooser.setDefaultOption("None", null);
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  private void configureBindings() {
    // Button 1: Run all motors forward for testing
    new JoystickButton(driverJoystick, 1)
        .whileTrue(new RunCommand(() -> driveSubsystem.runAllMotors(0.3), driveSubsystem));

    // Button 2: Run Rear Left Motor (ID 4)
    new JoystickButton(driverJoystick, 2)
        .whileTrue(new RunCommand(() -> driveSubsystem.runRearLeftMotor(0.3), driveSubsystem));

    // Button 3: Run Rear Right Motor (ID 2) - inverted in hardware
    new JoystickButton(driverJoystick, 3)
        .whileTrue(new RunCommand(() -> driveSubsystem.runRearRightMotor(0.3), driveSubsystem));

    // Button 4: Run Front Right Motor (ID 3) - inverted in hardware
    new JoystickButton(driverJoystick, 4)
        .whileTrue(new RunCommand(() -> driveSubsystem.runFrontRightMotor(0.3), driveSubsystem));
  }

  public void resetSensors() {
    driveSubsystem.zeroHeading();
    driveSubsystem.resetEncoders();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
