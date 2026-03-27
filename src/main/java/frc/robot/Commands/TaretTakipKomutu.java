package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.TaretAltSistemi;
import frc.robot.Subsystems.GorusAltSistemi;

public class TaretTakipKomutu extends Command {
    private TaretAltSistemi taretAltSistemi;
    private GorusAltSistemi gorusAltSistemi;

    public TaretTakipKomutu(TaretAltSistemi taretAltSistemi, GorusAltSistemi gorusAltSistemi) {
        this.taretAltSistemi = taretAltSistemi;
        this.gorusAltSistemi = gorusAltSistemi;
        addRequirements(taretAltSistemi);
    }

    @Override
    public void execute() {
        if (gorusAltSistemi.hasTarget()) {
            double yatayKayma = gorusAltSistemi.getHorizontalOffset();
            double hiz = yatayKayma * 0.01; // Oransal kontrol - ayarlanacak
            taretAltSistemi.dondur(hiz);
        } else {
            taretAltSistemi.durdur();
        }
    }

    @Override
    public void end(boolean interrupted) {
        taretAltSistemi.durdur();
    }

    @Override
    public boolean isFinished() {
        return false; // Surekli takip
    }
}
