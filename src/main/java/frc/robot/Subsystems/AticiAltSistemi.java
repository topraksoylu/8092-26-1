package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            aticiMotoru = new SparkMax(MotorSabitleri.ATICI_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig yapilandirma = new SparkMaxConfig();
            yapilandirma.inverted(MotorSabitleri.ATICI_MOTOR_TERS);
            yapilandirma.idleMode(IdleMode.kCoast); // Volan momentumunu korur — ani frenleme top hizini dusurur

            // Velocity PID + Feedforward (REV 2026 resmi ornegi)
            yapilandirma.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(ModulSabitleri.ATICI_KP)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                .velocityFF(ModulSabitleri.ATICI_KV); // kV = 12V / NEO max RPM

            aticiMotoru.configure(yapilandirma,
                com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
                com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

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
        sonHedefRPM = hedefRPM;
        if (pidKontrolcu != null) {
            pidKontrolcu.setReference(hedefRPM, SparkBase.ControlType.kVelocity);
        }
    }

    /** Varsayilan atici RPM ile at */
    public void at() {
        atRPM(ModulSabitleri.ATICI_HEDEF_RPM);
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Atici/HedefRPM", sonHedefRPM);
        SmartDashboard.putNumber("Atici/AktuelRPM", getAktuelRPM());
        SmartDashboard.putBoolean("Atici/HizaUlasti", isHizaUlasti());
    }

    /** CAN kaynagini serbest birakir — test ortaminda @AfterAll ile cagrilmali */
    public void close() {
        if (aticiMotoru != null) {
            try { aticiMotoru.close(); } catch (Exception ignored) {}
            aticiMotoru = null;
        }
    }
}
