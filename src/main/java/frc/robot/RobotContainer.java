// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.AlignToAprilTagCommand;
import frc.robot.Commands.TrackAprilTagCommand;
import frc.robot.Commands.RotateToAprilTag360Command;
import frc.robot.Constants.*;

public class RobotContainer {
  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private GenericHID driverController = new GenericHID(OIConstants.DRIVER_JOYSTICK_PORT);

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Instantiate vision subsystem first
    visionSubsystem = new VisionSubsystem();

    // Pass vision subsystem to drive subsystem for pose estimation
    driveSubsystem = new DriveSubsystem(
      MotorConstants.FRONT_LEFT_MOTOR_ID,
      MotorConstants.FRONT_RIGHT_MOTOR_ID,
      MotorConstants.REAR_LEFT_MOTOR_ID,
      MotorConstants.REAR_RIGHT_MOTOR_ID,
      new Pose2d(),
      visionSubsystem
    );
    configureBindings();

    // Note: PS4 controller must be connected via USB and show as "Wireless Controller" in Driver Station
    // Check Driver Station Joystick tab to verify connection on port 0

    driveSubsystem.setDefaultCommand(
        new DriveCommand(
            () -> driverController.getRawAxis(OIConstants.DRIVER_X_AXIS),  // X axis (strafe) - first param
            () -> driverController.getRawAxis(OIConstants.DRIVER_Y_AXIS),  // Y axis (forward/back) - second param
            () -> -driverController.getRawAxis(OIConstants.DRIVER_Z_AXIS), // Z axis (rotation)
            driveSubsystem
        )
    );

    try {
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner autos: " + e.getMessage(), true);
      e.printStackTrace();
      autoChooser = new SendableChooser<>();
      // Add a default "None" option
      autoChooser.setDefaultOption("None (PathPlanner Error)", null);
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // Log controller status on startup
    System.out.println("===================================================");
    System.out.println("ROBOT CONTAINER INITIALIZED");
    System.out.println("Driver Controller Port: " + OIConstants.DRIVER_JOYSTICK_PORT);
    System.out.println("Make sure PS4 controller is connected via USB!");
    System.out.println("===================================================");
  }

  /**
   * This method is called periodically (every 20ms)
   * Use it to check controller status and log button states
   */
  public void periodic() {
    // Read raw button states for debugging
    boolean button1 = driverController.getRawButton(1);
    boolean button2 = driverController.getRawButton(2);
    boolean button3 = driverController.getRawButton(3);
    boolean button4 = driverController.getRawButton(4);
    boolean button5 = driverController.getRawButton(5);
    boolean button6 = driverController.getRawButton(6);
    boolean button7 = driverController.getRawButton(7);
    boolean button8 = driverController.getRawButton(8);
    boolean button9 = driverController.getRawButton(9);
    boolean button10 = driverController.getRawButton(10);
    boolean button11 = driverController.getRawButton(11);
    boolean button12 = driverController.getRawButton(12);

    // Update SmartDashboard with button states (first 6 main buttons)
    SmartDashboard.putBoolean("DriverStation/Button1-Square", button1);
    SmartDashboard.putBoolean("DriverStation/Button2-Cross", button2);
    SmartDashboard.putBoolean("DriverStation/Button3-Circle", button3);
    SmartDashboard.putBoolean("DriverStation/Button4-Triangle", button4);
    SmartDashboard.putBoolean("DriverStation/Button5-L1", button5);
    SmartDashboard.putBoolean("DriverStation/Button6-R1", button6);
    SmartDashboard.putBoolean("DriverStation/Button7-L2", button7);
    SmartDashboard.putBoolean("DriverStation/Button8-R2", button8);
    SmartDashboard.putBoolean("DriverStation/Button9-Share", button9);
    SmartDashboard.putBoolean("DriverStation/Button10-Options", button10);
    SmartDashboard.putBoolean("DriverStation/Button11-L3", button11);
    SmartDashboard.putBoolean("DriverStation/Button12-R3", button12);

    // Read all axis values (PS4 has 6 axes)
    double axis0 = driverController.getRawAxis(0);  // Left Stick Y (forward/back)
    double axis1 = driverController.getRawAxis(1);  // Left Stick X (strafe)
    double axis2 = driverController.getRawAxis(2);  // Left Trigger (L2)
    double axis3 = driverController.getRawAxis(3);  // Right Stick X
    double axis4 = driverController.getRawAxis(4);  // Right Stick Y
    double axis5 = driverController.getRawAxis(5);  // Right Trigger (R2)

    SmartDashboard.putNumber("DriverStation/Axis0-LeftY", axis0);  // Forward/back
    SmartDashboard.putNumber("DriverStation/Axis1-LeftX", axis1);  // Strafe
    SmartDashboard.putNumber("DriverStation/Axis2-L2", axis2);
    SmartDashboard.putNumber("DriverStation/Axis3-RightX", axis3);
    SmartDashboard.putNumber("DriverStation/Axis4-RightY", axis4);
    SmartDashboard.putNumber("DriverStation/Axis5-R2", axis5);

    // Read POV (D-pad) values
    int pov = driverController.getPOV();
    SmartDashboard.putNumber("DriverStation/POV", pov);

    // Log button presses
    if (button1 || button2 || button3 || button4 || button5 || button6 ||
        button7 || button8 || button9 || button10 || button11 || button12) {
      String pressedButtons = "";
      if (button1) pressedButtons += "1(Square) ";
      if (button2) pressedButtons += "2(Cross) ";
      if (button3) pressedButtons += "3(Circle) ";
      if (button4) pressedButtons += "4(Triangle) ";
      if (button5) pressedButtons += "5(L1) ";
      if (button6) pressedButtons += "6(R1) ";
      if (button7) pressedButtons += "7(L2) ";
      if (button8) pressedButtons += "8(R2) ";
      if (button9) pressedButtons += "9(Share) ";
      if (button10) pressedButtons += "10(Options) ";
      if (button11) pressedButtons += "11(L3) ";
      if (button12) pressedButtons += "12(R3) ";

      SmartDashboard.putString("DriverStation/PressedButtons", pressedButtons);
    } else {
      SmartDashboard.putString("DriverStation/PressedButtons", "None");
    }

    // Build axis display (left stick: axis0=Y/forward, axis1=X/strafe)
    String axisInfo = String.format("L:(%.2f,%.2f) R:(%.2f,%.2f) LT:%.2f RT:%.2f",
        axis0, axis1, axis3, axis4, axis2, axis5);
    SmartDashboard.putString("DriverStation/AxesInfo", axisInfo);

    // POV display
    String povInfo = "POV: " + (pov == -1 ? "Center" :
                       pov == 0 ? "Up" :
                       pov == 45 ? "Up-Right" :
                       pov == 90 ? "Right" :
                       pov == 135 ? "Down-Right" :
                       pov == 180 ? "Down" :
                       pov == 225 ? "Down-Left" :
                       pov == 270 ? "Left" :
                       pov == 315 ? "Up-Left" : pov);
    SmartDashboard.putString("DriverStation/POVInfo", povInfo);
  }

  private void configureBindings() {
    // Button Square (1): Run Front Left Motor (ID 2) - normal direction
    new JoystickButton(driverController, 1)
        .onTrue(new RunCommand(() -> {
            System.out.println("===========================================");
            System.out.println("BUTTON 1 (SQUARE) PRESSED");
            System.out.println("Action: Running Front Left Motor (CAN ID 2)");
            System.out.println("===========================================");
            SmartDashboard.putString("DriverStation/LastButton", "Square (1) - Front Left");
        }, driveSubsystem))
        .whileTrue(new RunCommand(() -> driveSubsystem.runFrontLeftMotor(0.3), driveSubsystem));

    // Button Cross (2): Run Rear Left Motor (ID 3) - normal direction
    new JoystickButton(driverController, 2)
        .onTrue(new RunCommand(() -> {
            System.out.println("===========================================");
            System.out.println("BUTTON 2 (CROSS) PRESSED");
            System.out.println("Action: Running Rear Left Motor (CAN ID 3)");
            System.out.println("===========================================");
            SmartDashboard.putString("DriverStation/LastButton", "Cross (2) - Rear Left");
        }, driveSubsystem))
        .whileTrue(new RunCommand(() -> driveSubsystem.runRearLeftMotor(0.3), driveSubsystem));

    // Button Circle (3): Run Rear Right Motor (ID 1) - inverted in hardware
    new JoystickButton(driverController, 3)
        .onTrue(new RunCommand(() -> {
            System.out.println("===========================================");
            System.out.println("BUTTON 3 (CIRCLE) PRESSED");
            System.out.println("Action: Running Rear Right Motor (CAN ID 1)");
            System.out.println("===========================================");
            SmartDashboard.putString("DriverStation/LastButton", "Circle (3) - Rear Right");
        }, driveSubsystem))
        .whileTrue(new RunCommand(() -> driveSubsystem.runRearRightMotor(0.3), driveSubsystem));

    // Button Triangle (4): Run Front Right Motor (ID 4) - inverted in hardware
    new JoystickButton(driverController, 4)
        .onTrue(new RunCommand(() -> {
            System.out.println("===========================================");
            System.out.println("BUTTON 4 (TRIANGLE) PRESSED");
            System.out.println("Action: Running Front Right Motor (CAN ID 4)");
            System.out.println("===========================================");
            SmartDashboard.putString("DriverStation/LastButton", "Triangle (4) - Front Right");
        }, driveSubsystem))
        .whileTrue(new RunCommand(() -> driveSubsystem.runFrontRightMotor(0.3), driveSubsystem));

    // Button L1 (5): Reset pose from vision (AprilTag detection)
    new JoystickButton(driverController, 5)
        .onTrue(new InstantCommand(() -> {
            System.out.println("===========================================");
            System.out.println("BUTTON 5 (L1) PRESSED");
            System.out.println("Action: Resetting pose from AprilTag vision");
            System.out.println("===========================================");
            SmartDashboard.putString("DriverStation/LastButton", "L1 (5) - Pose Reset");
            driveSubsystem.resetPoseFromVision();
        }, driveSubsystem));

    // Button R1 (6): Align to AprilTag (vision alignment)
    // Aligns robot to face and drive toward AprilTag 1
    // Hold button to track, release to stop
    new JoystickButton(driverController, 6)
        .whileTrue(new AlignToAprilTagCommand(
            driveSubsystem,
            visionSubsystem,
            1,      // Target AprilTag ID
            1.0     // Stop 1 meter in front of tag
        ));

    // Button L2 (7): Rotation-only 360 scan to face AprilTag 1 directly
    // Press once: rotate up to 360 degrees, lock when tag is centered, then stop
    new JoystickButton(driverController, 7)
        .onTrue(new RotateToAprilTag360Command(
            driveSubsystem,
            visionSubsystem,
            1       // Target AprilTag ID
        ));

    // Button R2 (8): Track AprilTag continuously
    // Hold to continuously track and follow AprilTag as you move it
    // Maintains 1.0 meter distance from the tag
    new JoystickButton(driverController, 8)
        .onTrue(new RunCommand(() -> {
            System.out.println("===========================================");
            System.out.println("BUTTON 8 (R2) PRESSED");
            System.out.println("Action: Starting continuous AprilTag tracking");
            System.out.println("Target: Tag ID 1, Distance: 1.5 meters");
            System.out.println("Hold to track, release to stop");
            System.out.println("===========================================");
            SmartDashboard.putString("DriverStation/LastButton", "R2 (8) - Track Tag");
        }, driveSubsystem))
        .whileTrue(new TrackAprilTagCommand(
            driveSubsystem,
            visionSubsystem,
            1,      // Target AprilTag ID
            1.0     // Maintain 1.0 meter distance
        ));
  }

  public void resetSensors() {
    driveSubsystem.zeroHeading();
    driveSubsystem.resetEncoders();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Get the vision subsystem instance
   * @return VisionSubsystem
   */
  public VisionSubsystem getVisionSubsystem() {
    return visionSubsystem;
  }

}
