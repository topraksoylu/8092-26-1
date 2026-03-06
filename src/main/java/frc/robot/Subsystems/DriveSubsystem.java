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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.NavXTestConstants;
import frc.robot.FieldConstants;
import frc.robot.Simulation.DrivetrainSimulator;
import frc.robot.Simulation.NavXSimulator;

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
    private double simulatedHeading = 0.0; // For simulation (deprecated - use navxSim)

    // Simulation
    private NavXSimulator navxSim;  // Enhanced NavX simulation
    private DrivetrainSimulator drivetrainSimulator;
    private double simFrontLeftPosition = 0.0;
    private double simFrontRightPosition = 0.0;
    private double simRearLeftPosition = 0.0;
    private double simRearRightPosition = 0.0;
    private double simFrontLeftVelocity = 0.0;
    private double simFrontRightVelocity = 0.0;
    private double simRearLeftVelocity = 0.0;
    private double simRearRightVelocity = 0.0;

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

  // Vision subsystem for pose estimation
  private VisionSubsystem visionSubsystem;
  private double lastVisionUpdateTimestamp = 0;

  // Cached Pose2d objects for Field2D visualization (created once, reused)
  private Pose2d robotOutlinePose;
  private Pose2d[] robotOutlineCorners;

  // Dashboard update batching (reduce SmartTables overhead)
  private int dashboardUpdateCounter = 0;
  private static final int DASHBOARD_UPDATE_RATE = 10; // Update every 200ms

  // No-op: motor-test controls handled in periodic via SmartDashboard toggles


  public DriveSubsystem(int frontLeftMotorID, int frontRightMotorID, int rearLeftMotorID, int rearRightMotorID, Pose2d initialPose) {
      this(frontLeftMotorID, frontRightMotorID, rearLeftMotorID, rearRightMotorID, initialPose, null);
  }

  public DriveSubsystem(int frontLeftMotorID, int frontRightMotorID, int rearLeftMotorID, int rearRightMotorID, Pose2d initialPose, VisionSubsystem visionSubsystem) {
    rearLeftMotor = new SparkMax(rearLeftMotorID, MotorType.kBrushless);
    frontLeftMotor = new SparkMax(frontLeftMotorID, MotorType.kBrushless);
    rearRightMotor = new SparkMax(rearRightMotorID, MotorType.kBrushless);
    frontRightMotor = new SparkMax(frontRightMotorID, MotorType.kBrushless);

    // Configure motors with smart current limiting for NEO V1.1
    SparkMaxConfig reversedConfig = new SparkMaxConfig();
    reversedConfig.inverted(true);
    reversedConfig.smartCurrentLimit(
        MotorConstants.DRIVE_MOTOR_STALL_CURRENT_LIMIT,
        MotorConstants.DRIVE_MOTOR_FREE_CURRENT_LIMIT,
        MotorConstants.DRIVE_MOTOR_CURRENT_LIMIT_THRESHOLD
    );

    SparkMaxConfig nonReversedConfig = new SparkMaxConfig();
    nonReversedConfig.inverted(false);
    nonReversedConfig.smartCurrentLimit(
        MotorConstants.DRIVE_MOTOR_STALL_CURRENT_LIMIT,
        MotorConstants.DRIVE_MOTOR_FREE_CURRENT_LIMIT,
        MotorConstants.DRIVE_MOTOR_CURRENT_LIMIT_THRESHOLD
    );

    rearLeftMotor.configure(MotorConstants.REAR_LEFT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontLeftMotor.configure(MotorConstants.FRONT_LEFT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rearRightMotor.configure(MotorConstants.REAR_RIGHT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(MotorConstants.FRONT_RIGHT_MOTOR_INVERTED ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize gyro based on robot mode
    if (RobotBase.isReal()) {
      navx = new AHRS(NavXComType.kMXP_SPI);
    } else {
      // Simulation mode - use NavX simulator
      navx = null;
      navxSim = new NavXSimulator();
      // Initialize drivetrain simulator
      drivetrainSimulator = new DrivetrainSimulator();
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

    // Initialize cached Pose2d objects for Field2D (created once to avoid GC overhead)
    robotOutlineCorners = new Pose2d[5];
    robotOutlineCorners[0] = new Pose2d(-0.3, -0.3, new Rotation2d());
    robotOutlineCorners[1] = new Pose2d(0.3, -0.3, new Rotation2d());
    robotOutlineCorners[2] = new Pose2d(0.3, 0.3, new Rotation2d());
    robotOutlineCorners[3] = new Pose2d(-0.3, 0.3, new Rotation2d());
    robotOutlineCorners[4] = robotOutlineCorners[0]; // Close the square

    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    // Keep MotorSafety enabled, but allow more headroom for heavier periodic cycles.
    mecanumDrive.setExpiration(0.5);
    mecanumDrive.setSafetyEnabled(true);

    // Store vision subsystem reference
    this.visionSubsystem = visionSubsystem;

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

    // Feed MotorSafety watchdog since we're setting motors directly
    // This is important for PathPlanner which calls setSpeeds() directly
    if (mecanumDrive != null) {
      mecanumDrive.feed();
    }
  }

  // Test helpers: set individual motors directly for bench testing.
  public void testSetFrontLeft(double speed) {
    if (frontLeftMotor != null) frontLeftMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void testSetFrontRight(double speed) {
    if (frontRightMotor != null) frontRightMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void testSetRearLeft(double speed) {
    if (rearLeftMotor != null) rearLeftMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void testSetRearRight(double speed) {
    if (rearRightMotor != null) rearRightMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void stopAllMotors() {
    if (frontLeftMotor != null) frontLeftMotor.set(0);
    if (frontRightMotor != null) frontRightMotor.set(0);
    if (rearLeftMotor != null) rearLeftMotor.set(0);
    if (rearRightMotor != null) rearRightMotor.set(0);
    if (mecanumDrive != null) mecanumDrive.feed();
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

  static double wrapDeltaDegrees(double startDeg, double endDeg) {
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
    // Simulation - NavX simulator is always valid
    return navxSim != null;
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

  /**
   * Update odometry with vision measurements from AprilTags
   * This corrects odometry drift over time
   */
  private void updateVisionMeasurements() {
    if (visionSubsystem == null) {
      return;
    }

    VisionSubsystem.VisionResult result = visionSubsystem.getRobotPoseFromAprilTag();

    if (result.valid && Timer.getFPGATimestamp() - lastVisionUpdateTimestamp >=
        frc.robot.Constants.VisionConstants.POSE_UPDATE_INTERVAL_SEC) {

      // Reset odometry with vision pose to correct drift
      // This is a simple approach that works with MecanumDriveOdometry
      odometry.resetPosition(getHeading(), getWheelPositions(), result.robotPose);

      lastVisionUpdateTimestamp = Timer.getFPGATimestamp();

      SmartDashboard.putBoolean("Drive/VisionUpdate", true);
      SmartDashboard.putNumber("Drive/VisionTagID", result.tagId);
    } else {
      SmartDashboard.putBoolean("Drive/VisionUpdate", false);
    }
  }

  /**
   * Reset robot pose from vision (AprilTag detection)
   * Useful for re-localizing robot on the field
   */
  public void resetPoseFromVision() {
    if (visionSubsystem != null) {
      VisionSubsystem.VisionResult result = visionSubsystem.getRobotPoseFromAprilTag();
      if (result.valid) {
        resetPose(result.robotPose);
        DriverStation.reportWarning(
            String.format("Pose reset from AprilTag %d at (%.2f, %.2f)",
                result.tagId, result.robotPose.getX(), result.robotPose.getY()),
            false);
      }
    }
  }

    @Override
    public void periodic() {
        // Update simülasyon fiziklerini önce (simülasyon modundaysa)
        if (!RobotBase.isReal()) {
            updateSimulation(0.02);  // 50Hz periodic
        }

        // Feed early so MotorSafety doesn't trip if later periodic work runs long.
        if (mecanumDrive != null) {
          mecanumDrive.feed();
        }

        handleNavXValidationTrigger();
        runNavXValidationStateMachine();

        // Update odometry in all modes (autonomous and teleop)
        // Vision measurements will correct drift regardless of mode
        odometry.update(getHeading(), getWheelPositions());

        // Update odometry with vision measurements to correct drift
        updateVisionMeasurements();

        field.setRobotPose(odometry.getPoseMeters());

        // Update cached robot outline poses (avoid creating new objects every cycle)
        Pose2d currentPose = getPose();
        robotOutlineCorners[0] = new Pose2d(-0.3, -0.3, new Rotation2d()).relativeTo(currentPose);
        robotOutlineCorners[1] = new Pose2d(0.3, -0.3, new Rotation2d()).relativeTo(currentPose);
        robotOutlineCorners[2] = new Pose2d(0.3, 0.3, new Rotation2d()).relativeTo(currentPose);
        robotOutlineCorners[3] = new Pose2d(-0.3, 0.3, new Rotation2d()).relativeTo(currentPose);
        robotOutlineCorners[4] = robotOutlineCorners[0]; // Close the square

        field.getObject("Robot_Outline").setPoses(robotOutlineCorners);

        // Batched SmartDashboard updates (only every 10 cycles = 200ms)
        dashboardUpdateCounter++;
        if (dashboardUpdateCounter >= DASHBOARD_UPDATE_RATE) {
            dashboardUpdateCounter = 0;

            SmartDashboard.putNumber("Robot X", getPose().getX());
            SmartDashboard.putNumber("Robot Y", getPose().getY());
            SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
            SmartDashboard.putBoolean("Drive/FieldOrientedEnabled", true);

            // Motor outputs (debugging)
            SmartDashboard.putNumber("Drive/FrontLeftOutput", frontLeftMotor != null ? frontLeftMotor.get() : 0.0);
            SmartDashboard.putNumber("Drive/FrontRightOutput", frontRightMotor != null ? frontRightMotor.get() : 0.0);
            SmartDashboard.putNumber("Drive/RearLeftOutput", rearLeftMotor != null ? rearLeftMotor.get() : 0.0);
            SmartDashboard.putNumber("Drive/RearRightOutput", rearRightMotor != null ? rearRightMotor.get() : 0.0);
        }

  // Simülasyon telemetrisi
  if (!RobotBase.isReal()) {
    SmartDashboard.putNumber("Sim/BatteryVoltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Sim/FLPosition", simFrontLeftPosition);
    SmartDashboard.putNumber("Sim/FRPosition", simFrontRightPosition);
    SmartDashboard.putNumber("Sim/RLPosition", simRearLeftPosition);
    SmartDashboard.putNumber("Sim/RRPosition", simRearRightPosition);
  }

  // Read motor test toggles from SmartDashboard and schedule/cancel commands
  boolean flToggle = SmartDashboard.getBoolean("MotorTest/FrontLeft", false);
  boolean frToggle = SmartDashboard.getBoolean("MotorTest/FrontRight", false);
  boolean rlToggle = SmartDashboard.getBoolean("MotorTest/RearLeft", false);
  boolean rrToggle = SmartDashboard.getBoolean("MotorTest/RearRight", false);

  // Feed MotorSafety watchdog to prevent "Output not updated often enough" warnings
  // This is especially important when PathPlanner controls motors directly via setSpeeds()
  if (mecanumDrive != null) {
    mecanumDrive.feed();
  }

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

  /**
   * Simülasyonu günceller - fizik tabanlı sürüş dinamiklerini hesaplar.
   * Sadece simülasyon modunda çalışır.
   *
   * @param dtSeconds Zaman adımı (saniye)
   */
  private void updateSimulation(double dtSeconds) {
    if (drivetrainSimulator == null) {
      return;
    }

    // Motor çıkışlarından voltaj girişlerini hesapla
    double batteryVoltage = RobotController.getBatteryVoltage();
    double frontLeftVoltage = frontLeftMotor.get() * batteryVoltage;
    double frontRightVoltage = frontRightMotor.get() * batteryVoltage;
    double rearLeftVoltage = rearLeftMotor.get() * batteryVoltage;
    double rearRightVoltage = rearRightMotor.get() * batteryVoltage;

    // Sol ve sağ tarafları ortalama (mekanum yaklaşıklaması)
    double leftVoltage = (frontLeftVoltage + rearLeftVoltage) / 2.0;
    double rightVoltage = (frontRightVoltage + rearRightVoltage) / 2.0;

    // Fizik simülasyonunu güncelle
    drivetrainSimulator.setInputs(leftVoltage, rightVoltage);
    drivetrainSimulator.update(dtSeconds);

    // Simülasyon durumunu güncelle
    double leftPosition = drivetrainSimulator.getLeftPositionMeters();
    double rightPosition = drivetrainSimulator.getRightPositionMeters();
    double leftVelocity = drivetrainSimulator.getLeftVelocityMetersPerSecond();
    double rightVelocity = drivetrainSimulator.getRightVelocityMetersPerSecond();

    // Mekanum tekerlekleri için pozisyonları yaklaşık olarak hesapla
    // Öndeki ve arkadaki tekerlekler aynı tarafta aynı hareketi yapar
    simFrontLeftPosition = leftPosition;
    simRearLeftPosition = leftPosition;
    simFrontRightPosition = rightPosition;
    simRearRightPosition = rightPosition;

    simFrontLeftVelocity = leftVelocity;
    simRearLeftVelocity = leftVelocity;
    simFrontRightVelocity = rightVelocity;
    simRearRightVelocity = rightVelocity;

    // Simüle edilmiş başlığı güncelle (NavX simülatörünü kullan)
    if (navxSim != null) {
      // Dönüş hızını simüle edilmiş tahmininden al
      double currentHeading = drivetrainSimulator.getHeadingDegrees();
      double angularVelocity = (currentHeading - simulatedHeading) / dtSeconds;

      // NavX simülatörünü güncelle
      navxSim.update(dtSeconds, angularVelocity);
      simulatedHeading = navxSim.getYaw(); // Backward compatibility
    } else {
      simulatedHeading = drivetrainSimulator.getHeadingDegrees();
    }

    // Vision subsystem'i güncelle (simülasyon için robot pozisyonunu gönder)
    if (visionSubsystem != null) {
      Pose2d currentPose = drivetrainSimulator.getSim().getPose();
      visionSubsystem.setRobotPoseForSimulation(currentPose);
    }
  }

  public Rotation2d getHeading() {
    if (RobotBase.isReal() && navx != null) {
      return Rotation2d.fromDegrees(navx.getYaw());
    } else {
      // Simulation - use NavX simulator if available
      if (navxSim != null) {
        return Rotation2d.fromDegrees(navxSim.getYaw());
      }
      // Fallback to old simulatedHeading
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
    if (RobotBase.isReal()) {
      return DriveMath.encoderPositionToMeters(
          encoder.getPosition(),
          DriveConstants.GEARBOX_RATIO,
          DriveConstants.WHEEL_CIRCUMFERENCE);
    } else {
      // Simülasyon - simüle edilmiş encoder pozisyonlarını kullan
      if (encoder == frontLeftMotor.getEncoder()) return simFrontLeftPosition;
      if (encoder == frontRightMotor.getEncoder()) return simFrontRightPosition;
      if (encoder == rearLeftMotor.getEncoder()) return simRearLeftPosition;
      if (encoder == rearRightMotor.getEncoder()) return simRearRightPosition;
      return 0.0;
    }
  }

  public double getVelocity(RelativeEncoder encoder) {
    if (RobotBase.isReal()) {
      return DriveMath.encoderVelocityRpmToMetersPerSecond(
          encoder.getVelocity(),
          DriveConstants.GEARBOX_RATIO,
          DriveConstants.WHEEL_CIRCUMFERENCE);
    } else {
      // Simülasyon - simüle edilmiş encoder hızlarını kullan
      if (encoder == frontLeftMotor.getEncoder()) return simFrontLeftVelocity;
      if (encoder == frontRightMotor.getEncoder()) return simFrontRightVelocity;
      if (encoder == rearLeftMotor.getEncoder()) return simRearLeftVelocity;
      if (encoder == rearRightMotor.getEncoder()) return simRearRightVelocity;
      return 0.0;
    }
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
      // Simulation - zero NavX simulator
      if (navxSim != null) {
        navxSim.zeroYaw();
      }
      simulatedHeading = 0.0; // Backward compatibility
    }
  }

  // Run individual motors forward for testing
  public void runFrontLeftMotor(double speed) {
    frontLeftMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void runFrontRightMotor(double speed) {
    frontRightMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void runRearLeftMotor(double speed) {
    rearLeftMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void runRearRightMotor(double speed) {
    rearRightMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  // Run all motors forward for testing
  public void runAllMotors(double speed) {
    frontLeftMotor.set(speed/16);
    frontRightMotor.set(speed/16);
    rearLeftMotor.set(speed/16);
    rearRightMotor.set(speed/16);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

}
