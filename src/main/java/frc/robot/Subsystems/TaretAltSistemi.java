package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.MotorSabitleri;

public class TaretAltSistemi extends SubsystemBase {
    private SparkMax taretMotoru;
    private RelativeEncoder taretEnkoderi;
    private double sonKomutHizi = 0.0;

    public TaretAltSistemi() {
        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            taretMotoru = new SparkMax(MotorSabitleri.TARET_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig yapilandirma = new SparkMaxConfig();
            yapilandirma.inverted(MotorSabitleri.TARET_MOTOR_TERS);
            taretMotoru.configure(yapilandirma,
                com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
                com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
            taretEnkoderi = taretMotoru.getEncoder();
        } else {
            taretMotoru = null;
            taretEnkoderi = null;
        }
    }

    public void dondur(double hiz) {
        double aci = getAci();
        boolean maksimumda = aci >= MotorSabitleri.TARET_MAKS_ACI && hiz > 0;
        boolean minimumda = aci <= MotorSabitleri.TARET_MIN_ACI && hiz < 0;
        if (maksimumda || minimumda) {
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
        // Enkoder konumunu rotasyondan dereceye cevir
        if (taretEnkoderi != null) {
            return taretEnkoderi.getPosition() * 360.0;
        }
        return 0.0;
    }

    public void aciAyarla(double hedefAci) {
        double mevcutAci = getAci();
        double hata = hedefAci - mevcutAci;
        double hiz = hata * 0.01; // Oransal kontrol - ayarlanacak
        dondur(hiz);
    }

    public double getSonKomutHizi() {
        return sonKomutHizi;
    }
}
