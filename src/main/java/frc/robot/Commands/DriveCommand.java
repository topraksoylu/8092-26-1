// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.Subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  @FunctionalInterface
  public interface DriveOutput {
    void drive(double xSpeed, double ySpeed, double zRotation);
  }

  private final DriveOutput driveOutput;
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier ySpeed;
  private final DoubleSupplier zRotation;
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveControlConstants.TRANSLATION_SLEW_RATE);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveControlConstants.TRANSLATION_SLEW_RATE);
  private final SlewRateLimiter zLimiter = new SlewRateLimiter(DriveControlConstants.ROTATION_SLEW_RATE);


  public DriveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zRotation, DriveSubsystem driveSubsystem) {
    this(xSpeed, ySpeed, zRotation, driveSubsystem::drive, driveSubsystem);
  }

  DriveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zRotation, DriveOutput driveOutput, Subsystem... requirements) {
    this.driveOutput = driveOutput;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;

    if (requirements != null && requirements.length > 0) {
      addRequirements(requirements);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Driver requested orientation correction:
    // - forward input should command backward compared to previous behavior
    // - left/right strafe should be mirrored compared to previous behavior
    double y = shapeAxis(ySpeed.getAsDouble(), DriveControlConstants.TRANSLATION_SCALE);
    double x = shapeAxis(-xSpeed.getAsDouble(), DriveControlConstants.STRAFE_SCALE);
    // Rotation also uses squared curve for smoother low-speed turning
    double z = shapeAxis(zRotation.getAsDouble(), DriveControlConstants.ROTATION_SCALE);

    double xCommand = xLimiter.calculate(x);
    double yCommand = yLimiter.calculate(y);
    double zCommand = zLimiter.calculate(z);

    // Debug output
    SmartDashboard.putNumber("DriveDebug/xSpeed_in", xSpeed.getAsDouble());
    SmartDashboard.putNumber("DriveDebug/ySpeed_in", ySpeed.getAsDouble());
    SmartDashboard.putNumber("DriveDebug/xCommand", xCommand);
    SmartDashboard.putNumber("DriveDebug/yCommand", yCommand);

    driveOutput.drive(xCommand, yCommand, zCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  static double shapeAxis(double rawInput, double scale) {
    double deadbanded = MathUtil.applyDeadband(rawInput, DriveControlConstants.DEADBAND);
    return Math.copySign(deadbanded * deadbanded, deadbanded) * scale;
  }
}
