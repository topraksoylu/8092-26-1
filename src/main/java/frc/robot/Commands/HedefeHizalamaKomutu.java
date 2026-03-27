package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.TaretAltSistemi;
import frc.robot.Subsystems.GorusAltSistemi;

public class HedefeHizalamaKomutu extends Command {
    private SurusAltSistemi surusAltSistemi;
    private TaretAltSistemi taretAltSistemi;
    private GorusAltSistemi gorusAltSistemi;
    private double tolerans = 2.0; // derece

    public HedefeHizalamaKomutu(SurusAltSistemi surusAltSistemi, TaretAltSistemi taretAltSistemi, GorusAltSistemi gorusAltSistemi) {
        this.surusAltSistemi = surusAltSistemi;
        this.taretAltSistemi = taretAltSistemi;
        this.gorusAltSistemi = gorusAltSistemi;
        addRequirements(surusAltSistemi, taretAltSistemi);
    }

    @Override
    public void execute() {
        if (gorusAltSistemi.hasTarget()) {
            double yatayKayma = gorusAltSistemi.getHorizontalOffset();

            // Tareti dondur
            double taretHizi = yatayKayma * 0.02; // Ayarlanacak
            taretAltSistemi.dondur(taretHizi);

            // Gerekirse robotu da dondur
            // double surusDonus = yatayKayma * 0.01;
            // surusAltSistemi.drive(0, 0, surusDonus);
        }
    }

    @Override
    public void end(boolean interrupted) {
        taretAltSistemi.durdur();
        surusAltSistemi.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return gorusAltSistemi.hasTarget() && Math.abs(gorusAltSistemi.getHorizontalOffset()) < tolerans;
    }
}
