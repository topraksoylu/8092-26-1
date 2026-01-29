// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier ySpeed;
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;


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
    driveSubsystem.drive(ySpeed.getAsDouble(), xSpeed.getAsDouble(), zRotation.getAsDouble());
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
