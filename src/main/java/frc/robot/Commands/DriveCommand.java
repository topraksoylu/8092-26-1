// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.Subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier ySpeed;
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveControlConstants.TRANSLATION_SLEW_RATE);
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveControlConstants.TRANSLATION_SLEW_RATE);
  private final SlewRateLimiter zLimiter = new SlewRateLimiter(DriveControlConstants.ROTATION_SLEW_RATE);


  public DriveCommand(DoubleSupplier ySpeed, DoubleSupplier xSpeed, DoubleSupplier zRotation, DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.ySpeed = ySpeed;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Forward should be Y axis negative on this setup.
    double y = -ySpeed.getAsDouble();
    double x = xSpeed.getAsDouble();
    double z = zRotation.getAsDouble();

    y = MathUtil.applyDeadband(y, DriveControlConstants.DEADBAND);
    x = MathUtil.applyDeadband(x, DriveControlConstants.DEADBAND);
    z = MathUtil.applyDeadband(z, DriveControlConstants.DEADBAND);

    // Input shaping gives finer low-speed control without losing top-end command.
    // All axes use squared curve for smooth, precise control at low speeds
    y = Math.copySign(y * y, y) * DriveControlConstants.TRANSLATION_SCALE;
    x = Math.copySign(x * x, x) * DriveControlConstants.STRAFE_SCALE;
    // Rotation also uses squared curve for smoother low-speed turning
    z = Math.copySign(z * z, z) * DriveControlConstants.ROTATION_SCALE;

    double yCommand = yLimiter.calculate(y);
    double xCommand = xLimiter.calculate(x);
    double zCommand = zLimiter.calculate(z);

    driveSubsystem.drive(yCommand, xCommand, zCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
