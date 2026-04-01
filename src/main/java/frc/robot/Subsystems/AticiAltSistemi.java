package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.ModulSabitleri;
import frc.robot.Sabitler.MotorSabitleri;
import frc.robot.util.AtisHesaplayici;

public class AticiAltSistemi extends SubsystemBase {
    private SparkMax aticiMotoru;
    private SparkClosedLoopController pidKontrolcu;
    private RelativeEncoder aticiEnkoderi;
    private double sonHedefRPM = 0.0;

    public AticiAltSistemi() {
        SmartDashboard.putNumber("Ayarlama/YakinAtisRPM",   ModulSabitleri.YAKIN_ATIS_RPM);
        SmartDashboard.putNumber("Ayarlama/OrtaAtisRPM",    ModulSabitleri.ORTA_ATIS_RPM);
        SmartDashboard.putNumber("Ayarlama/UzakAtisRPM",    ModulSabitleri.UZAK_ATIS_RPM);
        SmartDashboard.putNumber("Ayarlama/CokUzakAtisRPM", ModulSabitleri.COK_UZAK_ATIS_RPM);
        SmartDashboard.putNumber("Ayarlama/AticiKP",        ModulSabitleri.ATICI_KP);
        SmartDashboard.putNumber("Ayarlama/AticiKI",        0.0);
        SmartDashboard.putNumber("Ayarlama/AticiKD",        0.0);
        SmartDashboard.putNumber("Ayarlama/AticiKFF",       ModulSabitleri.ATICI_KFF);

        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            aticiMotoru = new SparkMax(MotorSabitleri.ATICI_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig yapilandirma = new SparkMaxConfig();
            yapilandirma.inverted(MotorSabitleri.ATICI_MOTOR_TERS);
            yapilandirma.idleMode(IdleMode.kCoast);
            yapilandirma.voltageCompensation(12);
            yapilandirma.smartCurrentLimit(40);
            yapilandirma.encoder
                .velocityConversionFactor(1.0)
                .positionConversionFactor(1.0);

            double kp  = SmartDashboard.getNumber("Ayarlama/AticiKP",  ModulSabitleri.ATICI_KP);
            double ki  = SmartDashboard.getNumber("Ayarlama/AticiKI",  0.0);
            double kd  = SmartDashboard.getNumber("Ayarlama/AticiKD",  0.0);
            double kff = SmartDashboard.getNumber("Ayarlama/AticiKFF", ModulSabitleri.ATICI_KFF);

            yapilandirma.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kp, ki, kd)
                .outputRange(0.0, ModulSabitleri.ATICI_MAKS_CIKIS);
            yapilandirma.closedLoop.feedForward.kV(kff);

            aticiMotoru.configure(yapilandirma,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

            pidKontrolcu = aticiMotoru.getClosedLoopController();
            aticiEnkoderi = aticiMotoru.getEncoder();
        } else {
            aticiMotoru = null;
            pidKontrolcu = null;
            aticiEnkoderi = null;
        }
    }

    /** Belirtilen RPM'e velocity PID ile hizlan */
    public void atRPM(double hedefRPM) {
        double sinirliHedefRpm = rpmSinirla(hedefRPM);
        sonHedefRPM = sinirliHedefRpm;
        if (pidKontrolcu != null) {
            pidKontrolcu.setSetpoint(sinirliHedefRpm, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }
    }

    /** Limelight mesafesine göre RPM hesapla ve at. mesafeMetre < 0 ise durur. */
    public void atMesafeyeGore(double mesafeMetre) {
        if (mesafeMetre < 0) return;
        atRPM(AtisHesaplayici.hesaplaHedefRpm(mesafeMetre));
    }

    /** D-Pad Yukarı — yakın atış */
    public void atYakin() {
        atRPM(SmartDashboard.getNumber("Ayarlama/YakinAtisRPM", ModulSabitleri.YAKIN_ATIS_RPM));
        SmartDashboard.putString("Atici/AktifPreset", "YAKIN");
    }

    /** D-Pad Sağ — orta atış */
    public void atOrta() {
        atRPM(SmartDashboard.getNumber("Ayarlama/OrtaAtisRPM", ModulSabitleri.ORTA_ATIS_RPM));
        SmartDashboard.putString("Atici/AktifPreset", "ORTA");
    }

    /** D-Pad Aşağı — uzak atış */
    public void atUzak() {
        atRPM(SmartDashboard.getNumber("Ayarlama/UzakAtisRPM", ModulSabitleri.UZAK_ATIS_RPM));
        SmartDashboard.putString("Atici/AktifPreset", "UZAK");
    }

    /** D-Pad Sol — çok uzak atış */
    public void atCokUzak() {
        atRPM(SmartDashboard.getNumber("Ayarlama/CokUzakAtisRPM", ModulSabitleri.COK_UZAK_ATIS_RPM));
        SmartDashboard.putString("Atici/AktifPreset", "COK_UZAK");
    }

    public void durdur() {
        sonHedefRPM = 0.0;
        if (aticiMotoru != null) aticiMotoru.set(0);
    }

    public double getAktuelRPM() {
        if (aticiEnkoderi != null) return aticiEnkoderi.getVelocity();
        return 0.0;
    }

    /** Hedef RPM'e ulasti mi? (±200 RPM tolerans) */
    public boolean isHizaUlasti() {
        return sonHedefRPM > 0 && Math.abs(getAktuelRPM() - sonHedefRPM) < 200.0;
    }

    private double rpmSinirla(double rpm) {
        return MathUtil.clamp(rpm, ModulSabitleri.ATICI_MIN_RPM, ModulSabitleri.ATICI_MAKS_RPM);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Atici/HedefRPM",    sonHedefRPM);
        SmartDashboard.putNumber("Atici/AktuelRPM",   getAktuelRPM());
        SmartDashboard.putBoolean("Atici/HizaUlasti", isHizaUlasti());
        SmartDashboard.putNumber("Atici/RPMHata",     getAktuelRPM() - sonHedefRPM);
        SmartDashboard.putNumber("Atici/MotorCikisi",
            aticiMotoru != null ? aticiMotoru.getAppliedOutput() * aticiMotoru.getBusVoltage() : 0.0);
    }

    /** CAN kaynagini serbest birakir — test ortaminda @AfterAll ile cagrilmali */
    public void close() {
        if (aticiMotoru != null) {
            try { aticiMotoru.close(); } catch (Exception ignored) {}
            aticiMotoru = null;
        }
    }
}
