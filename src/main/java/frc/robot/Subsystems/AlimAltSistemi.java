package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.ModulSabitleri;
import frc.robot.Sabitler.MotorSabitleri;

public class AlimAltSistemi extends SubsystemBase {
    private PWMSparkMax alimCimMotoru;
    private PWMSparkMax yukariTasiyiciCimMotoru;
    private double sonAlimHizi = 0.0;
    private double sonTasiyiciHizi = 0.0;

    public AlimAltSistemi() {
        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            alimCimMotoru = new PWMSparkMax(MotorSabitleri.ALIM_CIM_PWM_KANALI);
            yukariTasiyiciCimMotoru = new PWMSparkMax(MotorSabitleri.DEPO_ATICI_YUKARI_TASIYICI_CIM_PWM_KANALI);
            alimCimMotoru.setInverted(MotorSabitleri.ALIM_MOTOR_TERS);
            yukariTasiyiciCimMotoru.setInverted(MotorSabitleri.DEPO_ATICI_YUKARI_TASIYICI_TERS);
        } else {
            alimCimMotoru = null;
            yukariTasiyiciCimMotoru = null;
        }
    }

    public void al() {
        sonAlimHizi = ModulSabitleri.ALIM_HIZI;
        if (alimCimMotoru != null) {
            alimCimMotoru.set(ModulSabitleri.ALIM_HIZI);
        }
    }

    public void geriAt() {
        sonAlimHizi = -ModulSabitleri.ALIM_HIZI;
        if (alimCimMotoru != null) {
            alimCimMotoru.set(-ModulSabitleri.ALIM_HIZI);
        }
    }

    public void depodanAticiyaYukariTasimaBaslat() {
        sonTasiyiciHizi = ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI;
        if (yukariTasiyiciCimMotoru != null) {
            yukariTasiyiciCimMotoru.set(ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI);
        }
    }

    public void depodanAticiyaYukariTasimaTersBaslat() {
        sonTasiyiciHizi = -ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI;
        if (yukariTasiyiciCimMotoru != null) {
            yukariTasiyiciCimMotoru.set(-ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI);
        }
    }

    public void depodanAticiyaYukariTasimaDurdur() {
        sonTasiyiciHizi = 0.0;
        if (yukariTasiyiciCimMotoru != null) {
            yukariTasiyiciCimMotoru.set(0.0);
        }
    }

    public void durdur() {
        sonAlimHizi = 0.0;
        sonTasiyiciHizi = 0.0;
        if (alimCimMotoru != null) {
            alimCimMotoru.set(0.0);
        }
        if (yukariTasiyiciCimMotoru != null) {
            yukariTasiyiciCimMotoru.set(0.0);
        }
    }

    public double getSonAlimHizi() {
        return sonAlimHizi;
    }

    public double getSonTasiyiciHizi() {
        return sonTasiyiciHizi;
    }

    /** PWM kanallarini serbest birakir — test ortaminda @AfterAll ile cagrilmali */
    public void close() {
        if (alimCimMotoru != null) {
            alimCimMotoru.close();
            alimCimMotoru = null;
        }
        if (yukariTasiyiciCimMotoru != null) {
            yukariTasiyiciCimMotoru.close();
            yukariTasiyiciCimMotoru = null;
        }
    }
}
