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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.NavXTestConstants;
import frc.robot.FieldConstants;

public class DriveSubsystem extends SubsystemBase {
  private enum NavXValidationState {
    IDLE,
    ZEROING,
    TURN_CW,
    TURN_CCW,
    DONE_PASS,
    DONE_FAIL
  }

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
  private NavXValidationState navXValidationState = NavXValidationState.IDLE;
  private boolean navXRunRequestedFromApi = false;
  private boolean navXRunDashboardPrevious = false;
  private double navXPhaseStartYawDeg = 0.0;
  private double navXPhaseEndYawDeg = 0.0;
  private double navXPhaseDeltaDeg = 0.0;
  private double navXCwDeltaDeg = 0.0;
  private double navXCcwDeltaDeg = 0.0;
  private double navXPhaseStartTimeSec = 0.0;
  private long navXZeroSettleEndMs = 0;
  private String navXValidationStatus = "IDLE";
  private String navXValidationErrorCode = "OK";

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

  public void requestNavXYawValidation() {
    navXRunRequestedFromApi = true;
    SmartDashboard.putBoolean("NavXTest/Run", true);
  }

  public boolean isNavXValidationRunning() {
    return navXValidationState != NavXValidationState.IDLE
        && navXValidationState != NavXValidationState.DONE_PASS
        && navXValidationState != NavXValidationState.DONE_FAIL;
  }

  public String getNavXValidationStatus() {
    return navXValidationStatus;
  }

  public String getNavXValidationErrorCode() {
    return navXValidationErrorCode;
  }

  private void updateNavXDashboard() {
    SmartDashboard.putString("NavXTest/Status", navXValidationStatus);
    SmartDashboard.putString("NavXTest/ErrorCode", navXValidationErrorCode);
    SmartDashboard.putNumber("NavXTest/LastYawStartDeg", navXPhaseStartYawDeg);
    SmartDashboard.putNumber("NavXTest/LastYawEndDeg", navXPhaseEndYawDeg);
    SmartDashboard.putNumber("NavXTest/DeltaDeg", navXPhaseDeltaDeg);
  }

  private static double nowSec() {
    return System.currentTimeMillis() / 1000.0;
  }

  private static long nowMs() {
    return System.currentTimeMillis();
  }

  private static double wrapDeltaDegrees(double startDeg, double endDeg) {
    return MathUtil.inputModulus(endDeg - startDeg, -180.0, 180.0);
  }

  private double getCurrentYawDeg() {
    if (RobotBase.isReal() && navx != null) {
      return navx.getYaw();
    }
    return simulatedHeading;
  }

  private boolean isNavXValid() {
    if (RobotBase.isReal()) {
      return navx != null && navx.isConnected() && Double.isFinite(navx.getYaw());
    }
    return false;
  }

  private void startNavXValidation() {
    if (!DriverStation.isDisabled()) {
      failNavXValidation("NAVX_E006", "NavX test must be triggered while disabled");
      return;
    }
    if (!isNavXValid()) {
      failNavXValidation("NAVX_E001", "NavX not connected or invalid reading");
      return;
    }

    zeroHeading();
    navXPhaseStartYawDeg = getCurrentYawDeg();
    navXPhaseEndYawDeg = navXPhaseStartYawDeg;
    navXPhaseDeltaDeg = 0.0;
    navXCwDeltaDeg = 0.0;
    navXCcwDeltaDeg = 0.0;
    navXZeroSettleEndMs = nowMs() + NavXTestConstants.ZERO_SETTLE_MS;
    navXValidationStatus = "ZEROING";
    navXValidationErrorCode = "OK";
    navXValidationState = NavXValidationState.ZEROING;
    updateNavXDashboard();
  }

  private void startTurnPhase(NavXValidationState state) {
    navXPhaseStartYawDeg = getCurrentYawDeg();
    navXPhaseEndYawDeg = navXPhaseStartYawDeg;
    navXPhaseDeltaDeg = 0.0;
    navXPhaseStartTimeSec = nowSec();
    navXValidationState = state;
    navXValidationStatus = state == NavXValidationState.TURN_CW ? "TURN_CW" : "TURN_CCW";
    updateNavXDashboard();
  }

  private void passNavXValidation() {
    stopAllMotors();
    navXValidationState = NavXValidationState.DONE_PASS;
    navXValidationStatus = "PASS";
    navXValidationErrorCode = "OK";
    SmartDashboard.putBoolean("NavXTest/Run", false);
    updateNavXDashboard();
    DriverStation.reportWarning(
        String.format("NavX yaw validation PASS (deltaCW=%.2f, deltaCCW=%.2f)", navXCwDeltaDeg, navXCcwDeltaDeg),
        false);
  }

  private void failNavXValidation(String errorCode, String message) {
    stopAllMotors();
    navXValidationState = NavXValidationState.DONE_FAIL;
    navXValidationStatus = "FAIL";
    navXValidationErrorCode = errorCode;
    SmartDashboard.putBoolean("NavXTest/Run", false);
    updateNavXDashboard();
    DriverStation.reportError(String.format("NavX yaw validation FAIL [%s] %s", errorCode, message), false);
  }

  private void runNavXValidationStateMachine() {
    double yawNow = getCurrentYawDeg();

    switch (navXValidationState) {
      case IDLE:
      case DONE_PASS:
      case DONE_FAIL:
        return;
      case ZEROING:
        if (!isNavXValid()) {
          failNavXValidation("NAVX_E001", "NavX lost connection during zeroing");
          return;
        }
        if (nowMs() >= navXZeroSettleEndMs) {
          startTurnPhase(NavXValidationState.TURN_CW);
        }
        return;
      case TURN_CW:
      case TURN_CCW:
        if (!isNavXValid()) {
          failNavXValidation("NAVX_E001", "NavX lost connection during turn phase");
          return;
        }

        double rotationCmd = navXValidationState == NavXValidationState.TURN_CW
            ? -NavXTestConstants.TEST_TURN_OUTPUT
            : NavXTestConstants.TEST_TURN_OUTPUT;
        drive(0.0, 0.0, rotationCmd);

        navXPhaseEndYawDeg = yawNow;
        navXPhaseDeltaDeg = wrapDeltaDegrees(navXPhaseStartYawDeg, navXPhaseEndYawDeg);
        updateNavXDashboard();

        if (Math.abs(navXPhaseDeltaDeg) > NavXTestConstants.MAX_ABS_YAW_JUMP_DEG) {
          failNavXValidation("NAVX_E007", "Yaw jump/outlier detected");
          return;
        }
        if (nowSec() - navXPhaseStartTimeSec > NavXTestConstants.TURN_PHASE_TIMEOUT_SEC) {
          if (Math.abs(navXPhaseDeltaDeg) < NavXTestConstants.MIN_EXPECTED_DELTA_DEG) {
            failNavXValidation("NAVX_E004", "Yaw delta too small");
          } else {
            failNavXValidation("NAVX_E005", "Turn phase timeout");
          }
          return;
        }
        if (Math.abs(navXPhaseDeltaDeg) < NavXTestConstants.MIN_EXPECTED_DELTA_DEG) {
          return;
        }

        if (navXValidationState == NavXValidationState.TURN_CW) {
          if (navXPhaseDeltaDeg >= 0.0) {
            failNavXValidation("NAVX_E002", "CW sign mismatch");
            return;
          }
          navXCwDeltaDeg = navXPhaseDeltaDeg;
          stopAllMotors();
          startTurnPhase(NavXValidationState.TURN_CCW);
        } else {
          if (navXPhaseDeltaDeg <= 0.0) {
            failNavXValidation("NAVX_E003", "CCW sign mismatch");
            return;
          }
          navXCcwDeltaDeg = navXPhaseDeltaDeg;
          passNavXValidation();
        }
        return;
      default:
        return;
    }
  }

  private void handleNavXValidationTrigger() {
    boolean dashboardRun = SmartDashboard.getBoolean("NavXTest/Run", false);
    boolean runRequested = navXRunRequestedFromApi || (dashboardRun && !navXRunDashboardPrevious);
    navXRunDashboardPrevious = dashboardRun;
    navXRunRequestedFromApi = false;

    if (runRequested && !isNavXValidationRunning()) {
      startNavXValidation();
    }
  }

    @Override
    public void periodic() {
        handleNavXValidationTrigger();
        runNavXValidationStateMachine();

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
  if (DriverStation.isDisabled() && !isNavXValidationRunning()) {
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

  // Run individual motors forward for testing
  public void runFrontLeftMotor(double speed) {
    frontLeftMotor.set(speed);
  }

  public void runFrontRightMotor(double speed) {
    frontRightMotor.set(speed);
  }

  public void runRearLeftMotor(double speed) {
    rearLeftMotor.set(speed);
  }

  public void runRearRightMotor(double speed) {
    rearRightMotor.set(speed);
  }

}
