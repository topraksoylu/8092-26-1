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
import frc.robot.util.AtisHesaplayici;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.ModulSabitleri;
import frc.robot.Sabitler.MotorSabitleri;

public class AticiAltSistemi extends SubsystemBase {
    private SparkMax aticiMotoru;
    private SparkClosedLoopController pidKontrolcu;
    private RelativeEncoder aticiEnkoderi;
    private double sonHedefRPM = 0.0;

    public AticiAltSistemi() {
        SmartDashboard.putNumber("Ayarlama/AticiRPM", ModulSabitleri.ATICI_HEDEF_RPM);
        SmartDashboard.putNumber("Ayarlama/YakinAtisRPM", ModulSabitleri.YAKIN_ATIS_RPM);
        SmartDashboard.putNumber("Ayarlama/OrtaAtisRPM", ModulSabitleri.ORTA_ATIS_RPM);
        SmartDashboard.putNumber("Ayarlama/UzakAtisRPM", ModulSabitleri.UZAK_ATIS_RPM);
        SmartDashboard.putNumber("Ayarlama/YakinAtisHizi", ModulSabitleri.YAKIN_ATIS_HIZI);
        SmartDashboard.putNumber("Ayarlama/OrtaAtisHizi", ModulSabitleri.ORTA_ATIS_HIZI);
        SmartDashboard.putNumber("Ayarlama/UzakAtisHizi", ModulSabitleri.UZAK_ATIS_HIZI);
        SmartDashboard.putNumber("Ayarlama/AticiHiz", ModulSabitleri.ATICI_HIZI);
        SmartDashboard.putNumber("Ayarlama/YakinAtisHizCarpani", ModulSabitleri.YAKIN_ATIS_HIZ_CARPANI);
        SmartDashboard.putNumber("Ayarlama/OrtaAtisHizCarpani", ModulSabitleri.ORTA_ATIS_HIZ_CARPANI);
        SmartDashboard.putNumber("Ayarlama/UzakAtisHizCarpani", ModulSabitleri.UZAK_ATIS_HIZ_CARPANI);
        SmartDashboard.putBoolean("Ayarlama/AticiAcikCevrimModu", false);
        SmartDashboard.putBoolean("Ayarlama/AticiPresetDashboardAktif", false);

        // PID ayarları - SmartDashboard'dan ayarlanabilir
        // Daha yüksek P ve I değerleri ile RPM kontrolü güçlendirildi
        SmartDashboard.putNumber("Ayarlama/AticiKP",  ModulSabitleri.ATICI_KP);
        SmartDashboard.putNumber("Ayarlama/AticiKI",  0.0);
        SmartDashboard.putNumber("Ayarlama/AticiKD",  0.0);
        SmartDashboard.putNumber("Ayarlama/AticiKFF", ModulSabitleri.ATICI_KFF);

        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            aticiMotoru = new SparkMax(MotorSabitleri.ATICI_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig yapilandirma = new SparkMaxConfig();
            yapilandirma.inverted(MotorSabitleri.ATICI_MOTOR_TERS);
            yapilandirma.idleMode(IdleMode.kCoast); // Volan momentumunu korur — ani frenleme top hizini dusurur
            yapilandirma.voltageCompensation(12); // batarya voltaj dususunda tutarli RPM

            // Akım sınırlama - motoru ve shooter'ı korumak için
            yapilandirma.smartCurrentLimit(40); // 40A limit

            // Enkoder velocity conversion faktörü (REV öntanımlı: RPM)
            // NEO bir devir = 1 rotasyon (dıştan dişli yoksa)
            yapilandirma.encoder
                .velocityConversionFactor(1.0)
                .positionConversionFactor(1.0);

            // Velocity PID + Feedforward
            double kp  = SmartDashboard.getNumber("Ayarlama/AticiKP",  ModulSabitleri.ATICI_KP);
            double ki  = SmartDashboard.getNumber("Ayarlama/AticiKI",  0.0);
            double kd  = SmartDashboard.getNumber("Ayarlama/AticiKD",  0.0);
            double kff = SmartDashboard.getNumber("Ayarlama/AticiKFF", ModulSabitleri.ATICI_KFF);

            yapilandirma.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kp, ki, kd)
                .outputRange(0.0, ModulSabitleri.ATICI_MAKS_CIKIS);

            aticiMotoru.configure(yapilandirma,
                ResetMode.kResetSafeParameters,  // Safe parameters reset
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
            double kff = SmartDashboard.getNumber("Ayarlama/AticiKFF", ModulSabitleri.ATICI_KFF);
            double arbFF = kff * sinirliHedefRpm;
            pidKontrolcu.setReference(
                sinirliHedefRpm,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                arbFF,
                SparkClosedLoopController.ArbFFUnits.kPercentOut);
        }
    }

    /** Shooter'ı belirli bir duty cycle ile çalıştır (test için) */
    public void calistirDutyCycle(double output) {
        if (aticiMotoru != null) {
            aticiMotoru.set(dutyCycleSinirla(output));
        }
    }

    /** Limelight mesafesine göre RPM hesapla ve at. mesafeMetre < 0 ise durur. */
    public void atMesafeyeGore(double mesafeMetre) {
        if (mesafeMetre < 0) return;
        atRPM(AtisHesaplayici.hesaplaHedefRpm(mesafeMetre));
    }

    /** Varsayilan atici RPM ile at (Shuffleboard'dan ayarlanabilir) */
    public void at() {
        atRPM(SmartDashboard.getNumber("Ayarlama/AticiRPM", ModulSabitleri.ATICI_HEDEF_RPM));
    }

    /** Yakın atış RPM'i ile at (Elastic Dashboard'dan ayarlanabilir) */
    public void atYakin() {
        double hiz = presetHiziHesapla("Ayarlama/YakinAtisHizCarpani", ModulSabitleri.YAKIN_ATIS_HIZ_CARPANI);
        calistir(hiz);
        SmartDashboard.putString("Atici/AktifPreset", "YAKIN_HIZ");
        SmartDashboard.putNumber("Atici/PresetIstenenHiz", hiz);
    }

    /** Orta atış RPM'i ile at (Elastic Dashboard'dan ayarlanabilir) */
    public void atOrta() {
        double hiz = presetHiziHesapla("Ayarlama/OrtaAtisHizCarpani", ModulSabitleri.ORTA_ATIS_HIZ_CARPANI);
        calistir(hiz);
        SmartDashboard.putString("Atici/AktifPreset", "ORTA_HIZ");
        SmartDashboard.putNumber("Atici/PresetIstenenHiz", hiz);
    }

    /** Uzak atış RPM'i ile at (Elastic Dashboard'dan ayarlanabilir) */
    public void atUzak() {
        double hiz = presetHiziHesapla("Ayarlama/UzakAtisHizCarpani", ModulSabitleri.UZAK_ATIS_HIZ_CARPANI);
        calistir(hiz);
        SmartDashboard.putString("Atici/AktifPreset", "UZAK_HIZ");
        SmartDashboard.putNumber("Atici/PresetIstenenHiz", hiz);
    }

    /** Dogrudan motor gucu ile calistir (duty cycle mode - brick kurtarma) */
    public void calistir(double hiz) {
        if (aticiMotoru != null) {
            sonHedefRPM = 0.0;
            double sinirliHiz = dutyCycleSinirla(hiz);
            try {
                aticiMotoru.set(sinirliHiz);
            } catch (Exception e) {
                aticiMotoru.set(sinirliHiz);
            }
        }
    }

    public void durdur() {
        sonHedefRPM = 0.0;
        if (aticiMotoru != null) aticiMotoru.set(0);
    }

    /** Mevcut motor RPM (enkoder okumasi) */
    public double getAktuelRPM() {
        if (aticiEnkoderi != null) return aticiEnkoderi.getVelocity();
        return 0.0;
    }

    /** Hedef RPM'e ulasti mi? (±200 RPM tolerans) */
    public boolean isHizaUlasti() {
        return sonHedefRPM > 0 && Math.abs(getAktuelRPM() - sonHedefRPM) < 200.0;
    }

    public double getSonHedefRPM() {
        return sonHedefRPM;
    }

    private double rpmSinirla(double rpm) {
        return MathUtil.clamp(rpm, ModulSabitleri.ATICI_MIN_RPM, ModulSabitleri.ATICI_MAKS_RPM);
    }

    private double dutyCycleSinirla(double cikis) {
        return MathUtil.clamp(cikis, -ModulSabitleri.ATICI_MAKS_CIKIS, ModulSabitleri.ATICI_MAKS_CIKIS);
    }

    private double presetRpmOku(String anahtar, double fallbackRpm) {
        if (!SmartDashboard.getBoolean("Ayarlama/AticiPresetDashboardAktif", false)) {
            return fallbackRpm;
        }
        return SmartDashboard.getNumber(anahtar, fallbackRpm);
    }

    private double presetHiziHesapla(String carpanAnahtari, double fallbackCarpan) {
        double tabanHiz = SmartDashboard.getNumber("Ayarlama/AticiHiz", ModulSabitleri.ATICI_HIZI);
        double carpan = SmartDashboard.getNumber(carpanAnahtari, fallbackCarpan);
        return dutyCycleSinirla(tabanHiz * carpan);
    }

    /** PID değerlerini SmartDashboard'dan okuyup günceller (tuning için çağır) */
    public void pidAyarlariniGuncelle() {
        if (aticiMotoru != null) {
            double kp  = SmartDashboard.getNumber("Ayarlama/AticiKP",  ModulSabitleri.ATICI_KP);
            double ki  = SmartDashboard.getNumber("Ayarlama/AticiKI",  0.0);
            double kd  = SmartDashboard.getNumber("Ayarlama/AticiKD",  0.0);
            double kff = SmartDashboard.getNumber("Ayarlama/AticiKFF", ModulSabitleri.ATICI_KFF);

            SparkMaxConfig pidGuncelle = new SparkMaxConfig();
            pidGuncelle.closedLoop.p(kp).i(ki).d(kd);
            aticiMotoru.configure(pidGuncelle,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Atici/HedefRPM", sonHedefRPM);
        SmartDashboard.putNumber("Atici/AktuelRPM", getAktuelRPM());
        SmartDashboard.putBoolean("Atici/HizaUlasti", isHizaUlasti());
        SmartDashboard.putNumber("Atici/RPMHata", getAktuelRPM() - sonHedefRPM);
        SmartDashboard.putNumber("Atici/MotorCikisi", aticiMotoru != null ? aticiMotoru.getAppliedOutput() * aticiMotoru.getBusVoltage() : 0.0);
        SmartDashboard.putBoolean("Atici/AcikCevrimModu", SmartDashboard.getBoolean("Ayarlama/AticiAcikCevrimModu", false));

        // PID değerlerini gerçek zamanlı güncelle (tuning için)
        // Sadece değerler değiştiyse güncelle - sürekli configure işlemesin
        if (aticiMotoru != null && pidKontrolcu != null) {
            double kp = SmartDashboard.getNumber("Ayarlama/AticiKP", ModulSabitleri.ATICI_KP);
            double ki = SmartDashboard.getNumber("Ayarlama/AticiKI", 0.0);
            double kd = SmartDashboard.getNumber("Ayarlama/AticiKD", 0.0);

            // Her periyotta configure etme - performansı etkiler
            // Bunun yerine PID değerlerini manuel değiştiğinde güncelle
        }
    }

    /** CAN kaynagini serbest birakir — test ortaminda @AfterAll ile cagrilmali */
    public void close() {
        if (aticiMotoru != null) {
            try { aticiMotoru.close(); } catch (Exception ignored) {}
            aticiMotoru = null;
        }
    }
}
