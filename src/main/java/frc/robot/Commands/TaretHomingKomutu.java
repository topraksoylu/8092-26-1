package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sabitler.MotorSabitleri;
import frc.robot.Subsystems.TaretAltSistemi;

/**
 * Tareti limit switch'e kadar sola döndürür, switch tetiklenince durur ve
 * enkoderi sıfırlar. Açı kontrolü yok - basit homing.
 */
public class TaretHomingKomutu extends Command {
    private final TaretAltSistemi taretAltSistemi;

    public TaretHomingKomutu(TaretAltSistemi taretAltSistemi) {
        this.taretAltSistemi = taretAltSistemi;
        addRequirements(taretAltSistemi);
    }

    @Override
    public void initialize() {
        // Zaten switch üzerindeyse hemen sıfırla ve bitir
        if (taretAltSistemi.limitSwitchTetiklendi()) {
            taretAltSistemi.enkoderiSifirla();
        }
    }

    @Override
    public void execute() {
        // Sola dön - limit switch'e değene kadar devam et
        // Açı kontrolü yok, sadece negatif hız
        if (!taretAltSistemi.limitSwitchTetiklendi()) {
            taretAltSistemi.dondurManuel(MotorSabitleri.TARET_HOMING_HIZI);
        }
    }

    @Override
    public boolean isFinished() {
        // Limit switch tetiklenince bitir
        return taretAltSistemi.limitSwitchTetiklendi();
    }

    @Override
    public void end(boolean interrupted) {
        taretAltSistemi.durdur();
        // dondurManuel() zaten limit switch'te -90 olarak ayarladı
        // Burada tekrar sıfırlamaya gerek yok
    }
}
