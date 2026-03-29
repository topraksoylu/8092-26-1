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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.MotorSabitleri;
import frc.robot.Sabitler.SurusSabitleri;
import frc.robot.Sabitler.GorusSabitleri;
import frc.robot.Sabitler.NavXTestSabitleri;
import frc.robot.FieldConstants;

public class SurusAltSistemi extends SubsystemBase {
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

  private MecanumDrivePoseEstimator poseEstimator;

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
  private GorusAltSistemi GorusAltSistemi;
  private double lastVisionUpdateTimestamp = 0;

  // No-op: motor-test controls handled in periodic via SmartDashboard toggles


  public SurusAltSistemi(int frontLeftMotorID, int frontRightMotorID, int rearLeftMotorID, int rearRightMotorID, Pose2d initialPose) {
      this(frontLeftMotorID, frontRightMotorID, rearLeftMotorID, rearRightMotorID, initialPose, null);
  }

  public SurusAltSistemi(int frontLeftMotorID, int frontRightMotorID, int rearLeftMotorID, int rearRightMotorID, Pose2d initialPose, GorusAltSistemi GorusAltSistemi) {
    rearLeftMotor = new SparkMax(rearLeftMotorID, MotorType.kBrushless);
    frontLeftMotor = new SparkMax(frontLeftMotorID, MotorType.kBrushless);
    rearRightMotor = new SparkMax(rearRightMotorID, MotorType.kBrushless);
    frontRightMotor = new SparkMax(frontRightMotorID, MotorType.kBrushless);

    // Configure motors with smart current limiting for NEO V1.1
    SparkMaxConfig reversedConfig = new SparkMaxConfig();
    reversedConfig.inverted(true);
    reversedConfig.idleMode(IdleMode.kBrake); // Aninda durma — pozisyon kaybini onler
    reversedConfig.smartCurrentLimit(
        MotorSabitleri.SURUS_MOTORU_DURMA_AKIM_SINIRI,
        MotorSabitleri.SURUS_MOTORU_BOSTA_AKIM_SINIRI,
        MotorSabitleri.SURUS_MOTORU_AKIM_SINIR_ESIGI
    );

    SparkMaxConfig nonReversedConfig = new SparkMaxConfig();
    nonReversedConfig.inverted(false);
    nonReversedConfig.idleMode(IdleMode.kBrake); // Aninda durma — pozisyon kaybini onler
    nonReversedConfig.smartCurrentLimit(
        MotorSabitleri.SURUS_MOTORU_DURMA_AKIM_SINIRI,
        MotorSabitleri.SURUS_MOTORU_BOSTA_AKIM_SINIRI,
        MotorSabitleri.SURUS_MOTORU_AKIM_SINIR_ESIGI
    );

    rearLeftMotor.configure(MotorSabitleri.ARKA_SOL_MOTOR_TERS ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontLeftMotor.configure(MotorSabitleri.ON_SOL_MOTOR_TERS ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rearRightMotor.configure(MotorSabitleri.ARKA_SAG_MOTOR_TERS ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(MotorSabitleri.ON_SAG_MOTOR_TERS ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize gyro based on robot mode
    if (RobotBase.isReal()) {
      navx = new AHRS(NavXComType.kMXP_SPI);
    } else {
      // Simulation mode - no physical NavX
      navx = null;
    }
    
    kinematics = new MecanumDriveKinematics(
      SurusSabitleri.TEKER_POZISYONLARI[0],
      SurusSabitleri.TEKER_POZISYONLARI[1],
      SurusSabitleri.TEKER_POZISYONLARI[2],
      SurusSabitleri.TEKER_POZISYONLARI[3]
    );

    poseEstimator = new MecanumDrivePoseEstimator(kinematics, getHeading(), getWheelPositions(), initialPose);

    field = new Field2d();

    FieldConstants.setupField(field);

    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    // Keep MotorSafety enabled, but allow more headroom for heavier periodic cycles.
    mecanumDrive.setExpiration(0.5);
    mecanumDrive.setSafetyEnabled(true);

    // Store vision subsystem reference
    this.GorusAltSistemi = GorusAltSistemi;

    try {
      RobotConfig config = RobotConfig.fromGUISettings();
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
      SmartDashboard.putBoolean("Auto/PathPlannerConfigured", true);
      SmartDashboard.putString("Auto/PathPlannerStatus", "READY");
    } catch (Exception e) {
      SmartDashboard.putBoolean("Auto/PathPlannerConfigured", false);
      SmartDashboard.putString("Auto/PathPlannerStatus", "CONFIG_ERROR");
      DriverStation.reportError("PathPlanner konfigurasyonu basarisiz: " + e.getMessage(), e.getStackTrace());
    }

  SmartDashboard.putData(field);
    
  }

  public void drive(double xSpeed, double ySpeed, double zRotation) {
    // WPILib expects (xSpeed, ySpeed, zRotation).
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  public void drive(double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle) {
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, gyroAngle);
  }

  public void driveFieldOriented(double xSpeed, double ySpeed, double zRotation) {
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, getHeading());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(SurusSabitleri.MAKS_HIZ_METRE_SANIYE);
    setSpeeds(wheelSpeeds);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    double frontLeftOutput = wheelSpeeds.frontLeftMetersPerSecond / SurusSabitleri.MAKS_HIZ_METRE_SANIYE;
    double frontRightOutput = wheelSpeeds.frontRightMetersPerSecond / SurusSabitleri.MAKS_HIZ_METRE_SANIYE;
    double rearLeftOutput = wheelSpeeds.rearLeftMetersPerSecond / SurusSabitleri.MAKS_HIZ_METRE_SANIYE;
    double rearRightOutput = wheelSpeeds.rearRightMetersPerSecond / SurusSabitleri.MAKS_HIZ_METRE_SANIYE;

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

  // Test yardimcilari: tezgah testleri icin motorlari tek tek dogrudan ayarla.
  public void testAyarlaOnSol(double hiz) {
    if (frontLeftMotor != null) frontLeftMotor.set(hiz);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void testAyarlaOnSag(double hiz) {
    if (frontRightMotor != null) frontRightMotor.set(hiz);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void testAyarlaArkaSol(double hiz) {
    if (rearLeftMotor != null) rearLeftMotor.set(hiz);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void testAyarlaArkaSag(double hiz) {
    if (rearRightMotor != null) rearRightMotor.set(hiz);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void tumMotorlariDurdur() {
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

    yonuSifirla();
    navXPhaseStartYawDeg = getCurrentYawDeg();
    navXPhaseEndYawDeg = navXPhaseStartYawDeg;
    navXPhaseDeltaDeg = 0.0;
    navXCwDeltaDeg = 0.0;
    navXCcwDeltaDeg = 0.0;
    navXZeroSettleEndMs = nowMs() + NavXTestSabitleri.SIFIRLAMA_OTURMA_MS;
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
    tumMotorlariDurdur();
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
    tumMotorlariDurdur();
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
            ? -NavXTestSabitleri.TEST_DONUS_CIKTISI
            : NavXTestSabitleri.TEST_DONUS_CIKTISI;
        drive(0.0, 0.0, rotationCmd);

        navXPhaseEndYawDeg = yawNow;
        navXPhaseDeltaDeg = wrapDeltaDegrees(navXPhaseStartYawDeg, navXPhaseEndYawDeg);
        updateNavXDashboard();

        if (Math.abs(navXPhaseDeltaDeg) > NavXTestSabitleri.MAKS_MUTLAK_YAW_SICRAMA_DERECE) {
          failNavXValidation("NAVX_E007", "Yaw jump/outlier detected");
          return;
        }
        if (nowSec() - navXPhaseStartTimeSec > NavXTestSabitleri.DONUS_ASAMA_ZAMAN_ASIMI_SN) {
          if (Math.abs(navXPhaseDeltaDeg) < NavXTestSabitleri.BEKLENEN_MIN_DELTA_DERECE) {
            failNavXValidation("NAVX_E004", "Yaw delta too small");
          } else {
            failNavXValidation("NAVX_E005", "Turn phase timeout");
          }
          return;
        }
        if (Math.abs(navXPhaseDeltaDeg) < NavXTestSabitleri.BEKLENEN_MIN_DELTA_DERECE) {
          return;
        }

        if (navXValidationState == NavXValidationState.TURN_CW) {
          if (navXPhaseDeltaDeg >= 0.0) {
            failNavXValidation("NAVX_E002", "CW sign mismatch");
            return;
          }
          navXCwDeltaDeg = navXPhaseDeltaDeg;
          tumMotorlariDurdur();
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
    if (GorusAltSistemi == null) {
      return;
    }

    GorusAltSistemi.VisionResult result = GorusAltSistemi.aprilTagdanRobotPozuAl();

    if (result.valid && Timer.getFPGATimestamp() - lastVisionUpdateTimestamp >=
        frc.robot.Sabitler.GorusSabitleri.POZ_GUNCELLEME_ARALIGI_SN) {

      // Fuse vision into estimator to correct drift without pose jumps.
      poseEstimator.addVisionMeasurement(result.robotPose, result.timestamp);

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
  public void gorusIlePozuSifirla() {
    if (GorusAltSistemi != null) {
      GorusAltSistemi.VisionResult result = GorusAltSistemi.aprilTagdanRobotPozuAl();
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
        // Feed early so MotorSafety doesn't trip if later periodic work runs long.
        if (mecanumDrive != null) {
          mecanumDrive.feed();
        }

        handleNavXValidationTrigger();
        runNavXValidationStateMachine();

        // Update odometry in all modes (autonomous and teleop)
        // Vision measurements will correct drift regardless of mode
        if (RobotBase.isReal()) {
            poseEstimator.update(getHeading(), getWheelPositions());
        } else {
            // Simulation - update heading based on wheel movements
            // This is a simple approximation for simulation
            // Approximate rotation based on velocity difference (simple sim)
            simulatedHeading += (getVelocity(frontRightMotor.getEncoder()) - getVelocity(frontLeftMotor.getEncoder())) * 0.01;
            poseEstimator.update(getHeading(), getWheelPositions());
        }

        // Update odometry with vision measurements to correct drift
        updateVisionMeasurements();

        field.setRobotPose(poseEstimator.getEstimatedPosition());

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

  // Teker hizlari (m/s) — motor donus kontrolu icin
  MecanumDriveWheelSpeeds hizlar = getWheelSpeeds();
  SmartDashboard.putNumber("Drive/HizOnSol_ms", hizlar.frontLeftMetersPerSecond);
  SmartDashboard.putNumber("Drive/HizOnSag_ms", hizlar.frontRightMetersPerSecond);
  SmartDashboard.putNumber("Drive/HizArkaSol_ms", hizlar.rearLeftMetersPerSecond);
  SmartDashboard.putNumber("Drive/HizArkaSag_ms", hizlar.rearRightMetersPerSecond);

  // Batarya ve NavX saglik bilgisi
  SmartDashboard.putNumber("Robot/BataryaVolt", RobotController.getBatteryVoltage());
  SmartDashboard.putBoolean("Robot/NavXBagli", RobotBase.isReal() && navx != null && navx.isConnected());
  SmartDashboard.putBoolean("Robot/NavXKalibre", RobotBase.isReal() && navx != null && !navx.isCalibrating());
  SmartDashboard.putNumber("Robot/NavXYaw", RobotBase.isReal() && navx != null ? -navx.getYaw() : simulatedHeading);

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

  // Manuel motor testi yalnizca robot devre disi iken izin verilir (guvenli tezgah testi)
  if (DriverStation.isDisabled() && !isNavXValidationRunning()) {
    if (flToggle) {
      testAyarlaOnSol(0.2);
    } else {
      testAyarlaOnSol(0.0);
    }

    if (frToggle) {
      testAyarlaOnSag(0.2);
    } else {
      testAyarlaOnSag(0.0);
    }

    if (rlToggle) {
      testAyarlaArkaSol(0.2);
    } else {
      testAyarlaArkaSol(0.0);
    }

    if (rrToggle) {
      testAyarlaArkaSag(0.2);
    } else {
      testAyarlaArkaSag(0.0);
    }
  }
    }

  public Rotation2d getHeading() {
    if (RobotBase.isReal() && navx != null) {
      // NavX CW-pozitif, WPILib CCW-pozitif kullanir — negatif almak zorunlu
      return Rotation2d.fromDegrees(-navx.getYaw());
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
    return SurusMatematigi.encoderPositionToMeters(
        encoder.getPosition(),
        SurusSabitleri.DISLI_ORANI,
        SurusSabitleri.TEKER_CEVRESI);
  }

  public double getVelocity(RelativeEncoder encoder) {
    return SurusMatematigi.encoderVelocityRpmToMetersPerSecond(
        encoder.getVelocity(),
        SurusSabitleri.DISLI_ORANI,
        SurusSabitleri.TEKER_CEVRESI);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getHeading(), getWheelPositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void encoderlariSifirla() {
    frontLeftMotor.getEncoder().setPosition(0);
    frontRightMotor.getEncoder().setPosition(0);
    rearLeftMotor.getEncoder().setPosition(0);
    rearRightMotor.getEncoder().setPosition(0);
  }

  public void yonuSifirla() {
    if (RobotBase.isReal() && navx != null) {
      navx.reset();
    } else {
      simulatedHeading = 0.0;
    }
  }

  // Run individual motors forward for testing
  public void onSolMotoruCalistir(double speed) {
    frontLeftMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void onSagMotoruCalistir(double speed) {
    frontRightMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void arkaSolMotoruCalistir(double speed) {
    rearLeftMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  public void arkaSagMotoruCalistir(double speed) {
    rearRightMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

  // Run all motors forward for testing
  public void tumMotorlariCalistir(double speed) {
    frontLeftMotor.set(speed);
    frontRightMotor.set(speed);
    rearLeftMotor.set(speed);
    rearRightMotor.set(speed);
    if (mecanumDrive != null) mecanumDrive.feed();
  }

}
