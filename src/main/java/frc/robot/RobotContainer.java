// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.TurretTrackCommand;
import frc.robot.Commands.AlignToTargetCommand;
import frc.robot.Commands.MotorTestCommand;
import frc.robot.Constants.*;

public class RobotContainer {
  // Pass motor IDs in the order expected by the DriveSubsystem constructor:
  // frontLeft, frontRight, rearLeft, rearRight
  private DriveSubsystem driveSubsystem = new DriveSubsystem(
    MotorConstants.FRONT_LEFT_MOTOR_ID,
    MotorConstants.FRONT_RIGHT_MOTOR_ID,
    MotorConstants.REAR_LEFT_MOTOR_ID,
    MotorConstants.REAR_RIGHT_MOTOR_ID,
    new Pose2d()
  );

  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private TurretSubsystem turretSubsystem = new TurretSubsystem();
  private VisionSubsystem visionSubsystem = new VisionSubsystem();

  private PS4Controller driverController = new PS4Controller(OIConstants.DRIVER_CONTROLLER_PORT);
  private Joystick operatorJoystick = new Joystick(OIConstants.OPERATOR_CONTROLLER_PORT);

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            () -> driverController.getRawAxis(OIConstants.DRIVER_Y_AXIS),
            () -> driverController.getRawAxis(OIConstants.DRIVER_X_AXIS),
            () -> driverController.getRawAxis(OIConstants.DRIVER_Z_AXIS),
            driveSubsystem
        )
    );
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  // Add simple toggles on SmartDashboard so you can trigger motor tests.
  SmartDashboard.putBoolean("MotorTest/FrontLeft", false);
  SmartDashboard.putBoolean("MotorTest/FrontRight", false);
  SmartDashboard.putBoolean("MotorTest/RearLeft", false);
  SmartDashboard.putBoolean("MotorTest/RearRight", false);

  // Manual NavX yaw validation controls/status
  SmartDashboard.putBoolean("NavXTest/Run", false);
  SmartDashboard.putString("NavXTest/Status", "IDLE");
  SmartDashboard.putString("NavXTest/ErrorCode", "OK");
  SmartDashboard.putNumber("NavXTest/LastYawStartDeg", 0.0);
  SmartDashboard.putNumber("NavXTest/LastYawEndDeg", 0.0);
  SmartDashboard.putNumber("NavXTest/DeltaDeg", 0.0);

  }

  private void configureBindings() {
    // Driver controls
    new JoystickButton(driverController, OIConstants.INTAKE_BUTTON)
        .whileTrue(new IntakeCommand(intakeSubsystem));

    new JoystickButton(driverController, OIConstants.ALIGN_BUTTON)
        .whileTrue(new AlignToTargetCommand(driveSubsystem, turretSubsystem, visionSubsystem));

    new JoystickButton(driverController, OIConstants.TURRET_TRACK_BUTTON)
        .whileTrue(new TurretTrackCommand(turretSubsystem, visionSubsystem));

    // Operator controls
    if (DriverStation.isJoystickConnected(OIConstants.OPERATOR_CONTROLLER_PORT)) {
      new JoystickButton(operatorJoystick, OIConstants.SHOOT_BUTTON)
          .whileTrue(new ShootCommand(shooterSubsystem, 2.0)); // Shoot for 2 seconds
    }
  }

  public void resetSensors() {
    driveSubsystem.zeroHeading();
    driveSubsystem.resetEncoders();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
