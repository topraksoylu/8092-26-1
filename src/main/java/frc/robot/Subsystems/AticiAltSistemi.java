package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.ModulSabitleri;
import frc.robot.Sabitler.MotorSabitleri;

public class AticiAltSistemi extends SubsystemBase {
    private SparkMax aticiMotoru;
    private double sonKomutHizi = 0.0;

    public AticiAltSistemi() {
        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            aticiMotoru = new SparkMax(MotorSabitleri.ATICI_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig yapilandirma = new SparkMaxConfig();
            yapilandirma.inverted(MotorSabitleri.ATICI_MOTOR_TERS);
            aticiMotoru.configure(yapilandirma,
                com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
                com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        } else {
            aticiMotoru = null;
        }
    }

    public void at() {
        sonKomutHizi = ModulSabitleri.ATICI_HIZI;
        if (aticiMotoru != null) aticiMotoru.set(ModulSabitleri.ATICI_HIZI);
    }

    public void durdur() {
        sonKomutHizi = 0.0;
        if (aticiMotoru != null) aticiMotoru.set(0);
    }

    public double getSonKomutHizi() {
        return sonKomutHizi;
    }

    /** CAN kaynagini serbest birakir — test ortaminda @AfterAll ile cagrilmali */
    public void close() {
        if (aticiMotoru != null) {
            try { aticiMotoru.close(); } catch (Exception ignored) {}
            aticiMotoru = null;
        }
    }
}
