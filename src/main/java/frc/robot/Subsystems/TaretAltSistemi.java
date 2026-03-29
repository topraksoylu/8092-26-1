package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.MotorSabitleri;

public class TaretAltSistemi extends SubsystemBase {
    private SparkMax taretMotoru;
    private RelativeEncoder taretEnkoderi;
    private double sonKomutHizi = 0.0;

    // Normally closed: get() == true → normal, get() == false → switch tetiklendi
    private final DigitalInput limitSwitch;

    public TaretAltSistemi() {
        limitSwitch = new DigitalInput(MotorSabitleri.TARET_LIMIT_SWITCH_DIO);

        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            taretMotoru = new SparkMax(MotorSabitleri.TARET_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig yapilandirma = new SparkMaxConfig();
            yapilandirma.inverted(MotorSabitleri.TARET_MOTOR_TERS);
            yapilandirma.idleMode(IdleMode.kBrake); // Pozisyon tutar — taret hedefe kilitli kalir

            // SparkMax donanim soft limitleri — RoboRIO cokse bile motor korunur
            float ileriLimitRot = (float) aciToMotorRotasyonu(MotorSabitleri.TARET_MAKS_ACI);
            float geriLimitRot  = (float) aciToMotorRotasyonu(MotorSabitleri.TARET_MIN_ACI);
            yapilandirma.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(ileriLimitRot)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(geriLimitRot);

            taretMotoru.configure(yapilandirma,
                com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
                com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
            taretEnkoderi = taretMotoru.getEncoder();
        } else {
            taretMotoru = null;
            taretEnkoderi = null;
        }
    }

    /** Normally closed switch tetiklendi mi? (taret deydi mi?)
     *  NC + Signal/GND baglantisi: basili degil=false, basili=true */
    public boolean limitSwitchTetiklendi() {
        return limitSwitch.get();
    }

    /** Taret acisini (derece) motor rotasyonuna donusturur (disli orani dikkate alinir) */
    private static double aciToMotorRotasyonu(double aciDerece) {
        return aciDerece / (360.0 / MotorSabitleri.TARET_DISLI_ORANI);
    }

    /** Enkoderi limit switch pozisyonuna ayarla (-90°) */
    public void enkoderiSifirla() {
        if (taretEnkoderi != null) {
            // -90° = -90 / (360 / disli_orani) motor rotasyonu
            double motorRotasyonu = MotorSabitleri.TARET_MIN_ACI / (360.0 / MotorSabitleri.TARET_DISLI_ORANI);
            taretEnkoderi.setPosition(motorRotasyonu);
        }
    }

    public void dondur(double hiz) {
        double aci = getAci();
        boolean maksimumda = aci >= MotorSabitleri.TARET_MAKS_ACI && hiz > 0;
        // Limit switch yonune (negatif) hareket ediliyorsa ve switch tetiklendiyse dur
        boolean limitSwitchYonunde = hiz < 0 && limitSwitchTetiklendi();
        boolean minimumda = aci <= MotorSabitleri.TARET_MIN_ACI && hiz < 0;

        if (maksimumda || minimumda || limitSwitchYonunde) {
            hiz = 0;
        }
        sonKomutHizi = hiz;
        if (taretMotoru != null) {
            taretMotoru.set(hiz);
        }
    }

    public void durdur() {
        sonKomutHizi = 0.0;
        if (taretMotoru != null) {
            taretMotoru.set(0);
        }
    }

    public double getAci() {
        // Motor rotasyonu → taret açısı: 360° / disli orani
        if (taretEnkoderi != null) {
            return taretEnkoderi.getPosition() * (360.0 / MotorSabitleri.TARET_DISLI_ORANI);
        }
        return 0.0;
    }

    public void aciAyarla(double hedefAci) {
        double mevcutAci = getAci();
        double hata = hedefAci - mevcutAci;
        double hiz = hata * 0.01;
        dondur(hiz);
    }

    public double getSonKomutHizi() {
        return sonKomutHizi;
    }

    @Override
    public void periodic() {
        double aci = getAci();
        boolean switchTetik = limitSwitchTetiklendi();

        SmartDashboard.putNumber("Taret/Aci", aci);
        SmartDashboard.putBoolean("Taret/LimitSwitch", switchTetik);
        SmartDashboard.putNumber("Taret/Motor", sonKomutHizi);

        // Blok nedeni — L1 neden dönmüyor sorununu teşhis eder
        SmartDashboard.putBoolean("Taret/Blok_LimitSwitch", sonKomutHizi == 0 && switchTetik);
        SmartDashboard.putBoolean("Taret/Blok_MinLimit",    aci <= MotorSabitleri.TARET_MIN_ACI);
        SmartDashboard.putBoolean("Taret/Blok_MaksLimit",   aci >= MotorSabitleri.TARET_MAKS_ACI);
    }

    /** DIO ve CAN kaynaklarini serbest birakir — test ortaminda @AfterAll ile cagrilmali */
    public void close() {
        limitSwitch.close();
        if (taretMotoru != null) {
            try { taretMotoru.close(); } catch (Exception ignored) {}
            taretMotoru = null;
        }
    }
}
