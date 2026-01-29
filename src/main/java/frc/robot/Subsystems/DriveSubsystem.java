// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class DriveSubsystem extends SubsystemBase {
  private MecanumDrive mecanumDrive;

  private SparkMax rearLeftMotor;
  private SparkMax frontLeftMotor;
  private SparkMax rearRightMotor;
  private SparkMax frontRightMotor;
 
  public DriveSubsystem(int rearLeftMotorID, int frontLeftMotorID, int rearRightMotorID, int frontRightMotorID) {
    rearLeftMotor = new SparkMax(rearLeftMotorID, MotorType.kBrushless);
    frontLeftMotor = new SparkMax(frontLeftMotorID, MotorType.kBrushless);
    rearRightMotor = new SparkMax(rearRightMotorID, MotorType.kBrushless);
    frontRightMotor = new SparkMax(frontRightMotorID, MotorType.kBrushless);

    SparkMaxConfig reversedConfig = new SparkMaxConfig();
    reversedConfig.inverted(true);
    SparkMaxConfig nonReversedConfig = new SparkMaxConfig();
    nonReversedConfig.inverted(false);
    
    rearLeftMotor.configure(MotorConstants.REAR_LEFT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontLeftMotor.configure(MotorConstants.FRONT_LEFT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rearRightMotor.configure(MotorConstants.REAR_RIGHT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(MotorConstants.FRONT_RIGHT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
  }

  public void drive(double ySpeed, double xSpeed, double zRotation) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  public void drive(double ySpeed, double xSpeed, double zRotation, Rotation2d gyroAngle) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, gyroAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
