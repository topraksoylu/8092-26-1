// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.FieldConstants;

public class DriveSubsystem extends SubsystemBase {
  private MecanumDrive mecanumDrive;

  private SparkMax rearLeftMotor;
  private SparkMax frontLeftMotor;
  private SparkMax rearRightMotor;
  private SparkMax frontRightMotor;

    private AHRS navx;
    private double simulatedHeading = 0.0; // For simulation

  private MecanumDriveKinematics kinematics;

  private MecanumDriveOdometry odometry;

  private Field2d field;

  // No-op: motor-test controls handled in periodic via SmartDashboard toggles

 
  public DriveSubsystem(int frontLeftMotorID, int frontRightMotorID, int rearLeftMotorID, int rearRightMotorID, Pose2d initialPose) {
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

    // Initialize gyro based on robot mode
    if (RobotBase.isReal()) {
      navx = new AHRS(NavXComType.kMXP_SPI);
    } else {
      // Simulation mode - no physical NavX
      navx = null;
    }
    
    kinematics = new MecanumDriveKinematics(
      DriveConstants.WHEEL_POSITIONS[0],
      DriveConstants.WHEEL_POSITIONS[1],
      DriveConstants.WHEEL_POSITIONS[2],
      DriveConstants.WHEEL_POSITIONS[3]
    );

    odometry = new MecanumDriveOdometry(kinematics, getHeading(), getWheelPositions(), initialPose);

    field = new Field2d();

    FieldConstants.setupField(field);

    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
              new PIDConstants(5.0, 0, 0, 0),
              new PIDConstants(5.0, 0, 0, 0)
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this
    );

  SmartDashboard.putData(field);
    
  }

  public void drive(double ySpeed, double xSpeed, double zRotation) {
    // WPILib expects (xSpeed, ySpeed, zRotation).
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  public void drive(double ySpeed, double xSpeed, double zRotation, Rotation2d gyroAngle) {
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, gyroAngle);
  }

  public void driveFieldOriented(double ySpeed, double xSpeed, double zRotation) {
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, getHeading());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    setSpeeds(wheelSpeeds);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {    
    double frontLeftOutput = wheelSpeeds.frontLeftMetersPerSecond / DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double frontRightOutput = wheelSpeeds.frontRightMetersPerSecond / DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rearLeftOutput = wheelSpeeds.rearLeftMetersPerSecond / DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rearRightOutput = wheelSpeeds.rearRightMetersPerSecond / DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    
    frontLeftMotor.set(frontLeftOutput);
    frontRightMotor.set(frontRightOutput);
    rearLeftMotor.set(rearLeftOutput);
    rearRightMotor.set(rearRightOutput);
  }

  // Test helpers: set individual motors directly for bench testing.
  public void testSetFrontLeft(double speed) {
    if (frontLeftMotor != null) frontLeftMotor.set(speed);
  }

  public void testSetFrontRight(double speed) {
    if (frontRightMotor != null) frontRightMotor.set(speed);
  }

  public void testSetRearLeft(double speed) {
    if (rearLeftMotor != null) rearLeftMotor.set(speed);
  }

  public void testSetRearRight(double speed) {
    if (rearRightMotor != null) rearRightMotor.set(speed);
  }

  public void stopAllMotors() {
    if (frontLeftMotor != null) frontLeftMotor.set(0);
    if (frontRightMotor != null) frontRightMotor.set(0);
    if (rearLeftMotor != null) rearLeftMotor.set(0);
    if (rearRightMotor != null) rearRightMotor.set(0);
  }

    @Override
    public void periodic() {
        // Only update odometry in autonomous mode to prevent drift issues in teleop
        if (DriverStation.isAutonomous()) {
            if (RobotBase.isReal()) {
                odometry.update(getHeading(), getWheelPositions());
            } else {
                // Simulation - update heading based on wheel movements
                // This is a simple approximation for simulation
                // Approximate rotation based on velocity difference (simple sim)
                simulatedHeading += (getVelocity(frontRightMotor.getEncoder()) - getVelocity(frontLeftMotor.getEncoder())) * 0.01;
                odometry.update(getHeading(), getWheelPositions());
            }
        }

        field.setRobotPose(odometry.getPoseMeters());

        // Add robot visualization elements
        field.getObject("Robot_Outline").setPoses(
            new Pose2d(-0.3, -0.3, new Rotation2d()).relativeTo(getPose()),
            new Pose2d(0.3, -0.3, new Rotation2d()).relativeTo(getPose()),
            new Pose2d(0.3, 0.3, new Rotation2d()).relativeTo(getPose()),
            new Pose2d(-0.3, 0.3, new Rotation2d()).relativeTo(getPose()),
            new Pose2d(-0.3, -0.3, new Rotation2d()).relativeTo(getPose())
        );

        SmartDashboard.putNumber("Robot X", getPose().getX());
        SmartDashboard.putNumber("Robot Y", getPose().getY());
        SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
        SmartDashboard.putBoolean("Drive/FieldOrientedEnabled", true);

  // Publish motor outputs for debugging (if using speed-based control)
  SmartDashboard.putNumber("Drive/FrontLeftOutput", frontLeftMotor != null ? frontLeftMotor.get() : 0.0);
  SmartDashboard.putNumber("Drive/FrontRightOutput", frontRightMotor != null ? frontRightMotor.get() : 0.0);
  SmartDashboard.putNumber("Drive/RearLeftOutput", rearLeftMotor != null ? rearLeftMotor.get() : 0.0);
  SmartDashboard.putNumber("Drive/RearRightOutput", rearRightMotor != null ? rearRightMotor.get() : 0.0);

  // Read motor test toggles from SmartDashboard and schedule/cancel commands
  boolean flToggle = SmartDashboard.getBoolean("MotorTest/FrontLeft", false);
  boolean frToggle = SmartDashboard.getBoolean("MotorTest/FrontRight", false);
  boolean rlToggle = SmartDashboard.getBoolean("MotorTest/RearLeft", false);
  boolean rrToggle = SmartDashboard.getBoolean("MotorTest/RearRight", false);

  // Only allow manual motor test outputs while robot is disabled (safe bench testing)
  if (DriverStation.isDisabled()) {
    if (flToggle) {
      testSetFrontLeft(0.2);
    } else {
      testSetFrontLeft(0.0);
    }

    if (frToggle) {
      testSetFrontRight(0.2);
    } else {
      testSetFrontRight(0.0);
    }

    if (rlToggle) {
      testSetRearLeft(0.2);
    } else {
      testSetRearLeft(0.0);
    }

    if (rrToggle) {
      testSetRearRight(0.2);
    } else {
      testSetRearRight(0.0);
    }
  }
    }

  public Rotation2d getHeading() {
    if (RobotBase.isReal() && navx != null) {
      return Rotation2d.fromDegrees(navx.getYaw());
    } else {
      // Simulation - return simulated heading
      return Rotation2d.fromDegrees(simulatedHeading);
    }
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
      getDistance(frontLeftMotor.getEncoder()),
      getDistance(frontRightMotor.getEncoder()),
      getDistance(rearLeftMotor.getEncoder()),
      getDistance(rearRightMotor.getEncoder())
    );
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      getVelocity(frontLeftMotor.getEncoder()),
      getVelocity(frontRightMotor.getEncoder()),
      getVelocity(rearLeftMotor.getEncoder()),
      getVelocity(rearRightMotor.getEncoder())
    );
  }

  public double getDistance(RelativeEncoder encoder) {
    return encoder.getPosition() / DriveConstants.GEARBOX_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE;
  }

  public double getVelocity(RelativeEncoder encoder) {
    return encoder.getVelocity() / DriveConstants.GEARBOX_RATIO * DriveConstants.WHEEL_CIRCUMFERENCE / 60.0;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(getHeading(), getWheelPositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void resetEncoders() {
    frontLeftMotor.getEncoder().setPosition(0);
    frontRightMotor.getEncoder().setPosition(0);
    rearLeftMotor.getEncoder().setPosition(0);
    rearRightMotor.getEncoder().setPosition(0);
  }

  public void zeroHeading() {
    if (RobotBase.isReal() && navx != null) {
      navx.reset();
    } else {
      simulatedHeading = 0.0;
    }
  }

}
