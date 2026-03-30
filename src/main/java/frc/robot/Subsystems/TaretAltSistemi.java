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
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.MotorSabitleri;

public class TaretAltSistemi extends SubsystemBase {
    private SparkMax taretMotoru;
    private SparkClosedLoopController pidKontrolcu;
    private RelativeEncoder taretEnkoderi;
    private double sonKomutHizi = 0.0;
    private double hedefAci = 0.0;

    /**
     * Enkoder sıfırlanmadan otomatik taret çalışmaz.
     * TaretHomingKomutu tamamlandığında true olur.
     */
    private boolean homingTamamlandi = false;

    // NC switch + Signal/GND: basili degil=false (kapali devre/GND), basili=true (acik devre/pull-up)
    private final DigitalInput limitSwitch;

    public TaretAltSistemi() {
        limitSwitch = new DigitalInput(MotorSabitleri.TARET_LIMIT_SWITCH_DIO);

        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            taretMotoru = new SparkMax(MotorSabitleri.TARET_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig yapilandirma = new SparkMaxConfig();
            yapilandirma.inverted(MotorSabitleri.TARET_MOTOR_TERS);
            yapilandirma.idleMode(IdleMode.kBrake); // Pozisyon tutar — taret hedefe kilitli kalir
            yapilandirma.voltageCompensation(12); // batarya voltaj dususunda tutarli cikis

            // SparkMax donanim soft limitleri — RoboRIO cokse bile motor korunur
            float ileriLimitRot = (float) aciToMotorRotasyonu(MotorSabitleri.TARET_MAKS_ACI);
            float geriLimitRot  = (float) aciToMotorRotasyonu(MotorSabitleri.TARET_MIN_ACI);
            yapilandirma.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(ileriLimitRot)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(geriLimitRot);

            // MAXMotion trapezoidal profil (3.2) + kI surme hatasi giderici (3.4)
            yapilandirma.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(MotorSabitleri.TARET_POZ_KP)
                .i(MotorSabitleri.TARET_KI)
                .d(0)
                .outputRange(-1, 1);
            yapilandirma.closedLoop.maxMotion
                .cruiseVelocity(MotorSabitleri.TARET_MAXMOTION_CRUISE_RPM)
                .maxAcceleration(MotorSabitleri.TARET_MAXMOTION_ACCEL_RPM_S)
                .allowedProfileError(MotorSabitleri.TARET_MAXMOTION_HATA_TOLERANSI)
                .positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal);

            taretMotoru.configure(yapilandirma,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
            taretEnkoderi = taretMotoru.getEncoder();
            pidKontrolcu = taretMotoru.getClosedLoopController();
        } else {
            taretMotoru = null;
            taretEnkoderi = null;
            pidKontrolcu = null;
        }
    }

    /** Normally closed switch tetiklendi mi? (taret deydi mi?)
     *  NC + Signal/GND baglantisi: basili degil=false, basili=true */
    public boolean limitSwitchTetiklendi() {
        boolean hamDeger = limitSwitch.get();
        return MotorSabitleri.TARET_LIMIT_SWITCH_AKTIF_HIGH ? hamDeger : !hamDeger;
    }

    /** Taret acisini (derece) motor rotasyonuna donusturur (disli orani dikkate alinir) */
    private static double aciToMotorRotasyonu(double aciDerece) {
        return aciDerece / (360.0 / MotorSabitleri.TARET_DISLI_ORANI);
    }

    /** Enkoderi limit switch pozisyonuna ayarla (-90°) ve homing tamamlandı bayrağını set et. */
    public void enkoderiSifirla() {
        if (taretEnkoderi != null) {
            double motorRotasyonu = aciToMotorRotasyonu(MotorSabitleri.TARET_MIN_ACI);
            taretEnkoderi.setPosition(motorRotasyonu);
        }
        homingTamamlandi = true;
    }

    /** Otomatik taret için homing tamamlandı mı? */
    public boolean isHomingTamamlandi() {
        return homingTamamlandi;
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

    /**
     * MAXMotion trapezoidal profil ile hedefe konumlan (yumusak hareket).
     * Soft limitler ile ±90° disina cikamaz.
     */
    public void aciAyarla(double hedefAci) {
        this.hedefAci = hedefAci;
        if (pidKontrolcu != null) {
            double hedefRotasyon = aciToMotorRotasyonu(hedefAci);
            pidKontrolcu.setSetpoint(hedefRotasyon, SparkBase.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
            // Hiz bilgisini tahmini olarak guncelle (gercek hiz enkoder ile izlenir)
            double hata = hedefAci - getAci();
            sonKomutHizi = Math.signum(hata) * Math.min(Math.abs(hata) * MotorSabitleri.TARET_POZ_KP, 1.0);
        } else {
            // Motor yoksa eski P kontrolu (sim / test)
            double hata = hedefAci - getAci();
            double hiz = hata * MotorSabitleri.TARET_POZ_KP;
            dondur(hiz);
        }
    }

    public double getSonKomutHizi() {
        return sonKomutHizi;
    }

    @Override
    public void periodic() {
        double aci = getAci();
        boolean switchTetik = limitSwitchTetiklendi();

        // Pozisyon ve hedef
        SmartDashboard.putNumber("Taret/Aci", aci);
        SmartDashboard.putNumber("Taret/HedefAci", hedefAci);
        SmartDashboard.putNumber("Taret/AciHatasi", hedefAci - aci);
        if (taretEnkoderi != null) {
            SmartDashboard.putNumber("Taret/EnkoderRotasyonu", taretEnkoderi.getPosition());
        }

        // Homing durumu
        SmartDashboard.putBoolean("Taret/HomingTamamlandi", homingTamamlandi);

        // Limit switch ve blokaj
        SmartDashboard.putBoolean("Taret/LimitSwitchHam", limitSwitch.get());
        SmartDashboard.putBoolean("Taret/LimitSwitch", switchTetik);
        SmartDashboard.putBoolean("Taret/Blok_LimitSwitch", sonKomutHizi == 0 && switchTetik);
        SmartDashboard.putBoolean("Taret/Blok_MinLimit",    aci <= MotorSabitleri.TARET_MIN_ACI);
        SmartDashboard.putBoolean("Taret/Blok_MaksLimit",   aci >= MotorSabitleri.TARET_MAKS_ACI);

        // Motor cikisi
        SmartDashboard.putNumber("Taret/KomutHizi", sonKomutHizi);
        if (taretMotoru != null) {
            SmartDashboard.putNumber("Taret/MotorCikisi", taretMotoru.getAppliedOutput());
            SmartDashboard.putNumber("Taret/Akim_A", taretMotoru.getOutputCurrent());
            SmartDashboard.putNumber("Taret/Sicaklik_C", taretMotoru.getMotorTemperature());
            SmartDashboard.putNumber("Taret/Voltaj_V", taretMotoru.getBusVoltage());
        }
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
