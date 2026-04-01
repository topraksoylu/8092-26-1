package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
  }

  public void drive(double xSpeed, double ySpeed, double zRotation) {
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation);
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
