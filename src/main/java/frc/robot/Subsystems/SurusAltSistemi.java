package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.MotorSabitleri;
import frc.robot.Sabitler.SurusSabitleri;

public class SurusAltSistemi extends SubsystemBase {

  private final MecanumDrive mecanumDrive;
  private final AHRS navX;
  private final MecanumDriveKinematics kinematics;
  private final MecanumDriveOdometry odometri;
  private Pose2d mevcutPoz = new Pose2d();

  private final SparkMax frontLeftMotor;
  private final SparkMax frontRightMotor;
  private final SparkMax rearLeftMotor;
  private final SparkMax rearRightMotor;

  public SurusAltSistemi(int frontLeftMotorID, int frontRightMotorID, int rearLeftMotorID, int rearRightMotorID) {
    frontLeftMotor  = new SparkMax(frontLeftMotorID,  MotorType.kBrushless);
    frontRightMotor = new SparkMax(frontRightMotorID, MotorType.kBrushless);
    rearLeftMotor   = new SparkMax(rearLeftMotorID,   MotorType.kBrushless);
    rearRightMotor  = new SparkMax(rearRightMotorID,  MotorType.kBrushless);

    SparkMaxConfig reversedConfig = new SparkMaxConfig();
    reversedConfig.inverted(true);
    reversedConfig.idleMode(IdleMode.kBrake);
    reversedConfig.voltageCompensation(12);
    reversedConfig.smartCurrentLimit(
        MotorSabitleri.SURUS_MOTORU_DURMA_AKIM_SINIRI,
        MotorSabitleri.SURUS_MOTORU_BOSTA_AKIM_SINIRI,
        MotorSabitleri.SURUS_MOTORU_AKIM_SINIR_ESIGI
    );

    SparkMaxConfig nonReversedConfig = new SparkMaxConfig();
    nonReversedConfig.inverted(false);
    nonReversedConfig.idleMode(IdleMode.kBrake);
    nonReversedConfig.voltageCompensation(12);
    nonReversedConfig.smartCurrentLimit(
        MotorSabitleri.SURUS_MOTORU_DURMA_AKIM_SINIRI,
        MotorSabitleri.SURUS_MOTORU_BOSTA_AKIM_SINIRI,
        MotorSabitleri.SURUS_MOTORU_AKIM_SINIR_ESIGI
    );

    frontLeftMotor.configure( MotorSabitleri.ON_SOL_MOTOR_TERS   ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(MotorSabitleri.ON_SAG_MOTOR_TERS   ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rearLeftMotor.configure(  MotorSabitleri.ARKA_SOL_MOTOR_TERS ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rearRightMotor.configure( MotorSabitleri.ARKA_SAG_MOTOR_TERS ? reversedConfig : nonReversedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    mecanumDrive.setExpiration(0.5);
    mecanumDrive.setSafetyEnabled(true);

    navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

    kinematics = new MecanumDriveKinematics(
        SurusSabitleri.TEKER_POZISYONLARI[0],
        SurusSabitleri.TEKER_POZISYONLARI[1],
        SurusSabitleri.TEKER_POZISYONLARI[2],
        SurusSabitleri.TEKER_POZISYONLARI[3]
    );
    odometri = new MecanumDriveOdometry(kinematics, navX.getRotation2d(), getWheelPositions());
  }

  public void drive(double xSpeed, double ySpeed, double zRotation) {
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  // ── NavX ─────────────────────────────────────────────────────────────────

  public void gyroSifirla() {
    navX.reset();
  }

  public double getAngleDegrees() {
    return navX.getAngle();
  }

  public Rotation2d getRotation2d() {
    return navX.getRotation2d();
  }

  public boolean isNavXConnected() {
    return navX.isConnected();
  }

  // ── Odometri ──────────────────────────────────────────────────────────────

  public Pose2d getPoz() {
    return mevcutPoz;
  }

  public void odometriSifirla(Pose2d poz) {
    odometri.resetPosition(navX.getRotation2d(), getWheelPositions(), poz);
    mevcutPoz = poz;
  }

  public void odometriSifirla() {
    odometriSifirla(new Pose2d());
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
        getPosition(frontLeftMotor.getEncoder()),
        getPosition(frontRightMotor.getEncoder()),
        getPosition(rearLeftMotor.getEncoder()),
        getPosition(rearRightMotor.getEncoder())
    );
  }

  private double getPosition(RelativeEncoder encoder) {
    return SurusMatematigi.encoderPositionToMeters(
        encoder.getPosition(),
        SurusSabitleri.DISLI_ORANI,
        SurusSabitleri.TEKER_CEVRESI
    );
  }

  // ── PathPlanner entegrasyonu ───────────────────────────────────────────────

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(SurusSabitleri.MAKS_HIZ_METRE_SANIYE);
    double maxSpeed = SurusSabitleri.MAKS_HIZ_METRE_SANIYE;
    frontLeftMotor.set(wheelSpeeds.frontLeftMetersPerSecond   / maxSpeed);
    frontRightMotor.set(wheelSpeeds.frontRightMetersPerSecond / maxSpeed);
    rearLeftMotor.set(wheelSpeeds.rearLeftMetersPerSecond     / maxSpeed);
    rearRightMotor.set(wheelSpeeds.rearRightMetersPerSecond   / maxSpeed);
    mecanumDrive.feed();
  }

  public void tumMotorlariDurdur() {
    frontLeftMotor.set(0);
    frontRightMotor.set(0);
    rearLeftMotor.set(0);
    rearRightMotor.set(0);
    mecanumDrive.feed();
  }

  // ── Tekil motor testi (robot devre dışıyken) ─────────────────────────────

  public void testAyarlaOnSol(double hiz)   { frontLeftMotor.set(hiz);  mecanumDrive.feed(); }
  public void testAyarlaOnSag(double hiz)   { frontRightMotor.set(hiz); mecanumDrive.feed(); }
  public void testAyarlaArkaSol(double hiz) { rearLeftMotor.set(hiz);   mecanumDrive.feed(); }
  public void testAyarlaArkaSag(double hiz) { rearRightMotor.set(hiz);  mecanumDrive.feed(); }

  public void onSolMotoruCalistir(double speed)   { frontLeftMotor.set(speed);  mecanumDrive.feed(); }
  public void onSagMotoruCalistir(double speed)   { frontRightMotor.set(speed); mecanumDrive.feed(); }
  public void arkaSolMotoruCalistir(double speed) { rearLeftMotor.set(speed);   mecanumDrive.feed(); }
  public void arkaSagMotoruCalistir(double speed) { rearRightMotor.set(speed);  mecanumDrive.feed(); }
  public void tumMotorlariCalistir(double speed) {
    frontLeftMotor.set(speed);
    frontRightMotor.set(speed);
    rearLeftMotor.set(speed);
    rearRightMotor.set(speed);
    mecanumDrive.feed();
  }

  // ── Teker hızları (dashboard için) ───────────────────────────────────────

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        getVelocity(frontLeftMotor.getEncoder()),
        getVelocity(frontRightMotor.getEncoder()),
        getVelocity(rearLeftMotor.getEncoder()),
        getVelocity(rearRightMotor.getEncoder())
    );
  }

  private double getVelocity(RelativeEncoder encoder) {
    return SurusMatematigi.encoderVelocityRpmToMetersPerSecond(
        encoder.getVelocity(),
        SurusSabitleri.DISLI_ORANI,
        SurusSabitleri.TEKER_CEVRESI);
  }

  @Override
  public void periodic() {
    mecanumDrive.feed();

    // Motor çıkışları
    SmartDashboard.putNumber("Drive/FrontLeftOutput",  frontLeftMotor.get());
    SmartDashboard.putNumber("Drive/FrontRightOutput", frontRightMotor.get());
    SmartDashboard.putNumber("Drive/RearLeftOutput",   rearLeftMotor.get());
    SmartDashboard.putNumber("Drive/RearRightOutput",  rearRightMotor.get());

    // Teker hızları (m/s)
    MecanumDriveWheelSpeeds hizlar = getWheelSpeeds();
    SmartDashboard.putNumber("Drive/HizOnSol_ms",   hizlar.frontLeftMetersPerSecond);
    SmartDashboard.putNumber("Drive/HizOnSag_ms",   hizlar.frontRightMetersPerSecond);
    SmartDashboard.putNumber("Drive/HizArkaSol_ms", hizlar.rearLeftMetersPerSecond);
    SmartDashboard.putNumber("Drive/HizArkaSag_ms", hizlar.rearRightMetersPerSecond);

    // Batarya
    SmartDashboard.putNumber("Robot/BataryaVolt", RobotController.getBatteryVoltage());

    // NavX + Odometri
    mevcutPoz = odometri.update(navX.getRotation2d(), getWheelPositions());
    SmartDashboard.putNumber("Drive/NavXAci",        navX.getAngle());
    SmartDashboard.putBoolean("Drive/NavXBagli",     navX.isConnected());
    SmartDashboard.putBoolean("Drive/NavXKalibre",   navX.isCalibrating());
    SmartDashboard.putNumber("Drive/PozX",           mevcutPoz.getX());
    SmartDashboard.putNumber("Drive/PozY",           mevcutPoz.getY());

    // Manuel motor testi — sadece robot devre dışıyken
    if (DriverStation.isDisabled()) {
      boolean flToggle = SmartDashboard.getBoolean("MotorTest/FrontLeft",  false);
      boolean frToggle = SmartDashboard.getBoolean("MotorTest/FrontRight", false);
      boolean rlToggle = SmartDashboard.getBoolean("MotorTest/RearLeft",   false);
      boolean rrToggle = SmartDashboard.getBoolean("MotorTest/RearRight",  false);
      testAyarlaOnSol(  flToggle ? 0.2 : 0.0);
      testAyarlaOnSag(  frToggle ? 0.2 : 0.0);
      testAyarlaArkaSol(rlToggle ? 0.2 : 0.0);
      testAyarlaArkaSag(rrToggle ? 0.2 : 0.0);
    }
  }
}
