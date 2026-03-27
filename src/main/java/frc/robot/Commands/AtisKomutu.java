package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlimAltSistemi;
import frc.robot.Subsystems.AticiAltSistemi;

public class AtisKomutu extends Command {
    private AticiAltSistemi aticiAltSistemi;
    private AlimAltSistemi alimAltSistemi;

    public AtisKomutu(AticiAltSistemi aticiAltSistemi, AlimAltSistemi alimAltSistemi, double sure) {
        this.aticiAltSistemi = aticiAltSistemi;
        this.alimAltSistemi = alimAltSistemi;
        addRequirements(aticiAltSistemi, alimAltSistemi);
    }

    public AtisKomutu(AticiAltSistemi aticiAltSistemi, double sure) {
        this.aticiAltSistemi = aticiAltSistemi;
        this.alimAltSistemi = null;
        addRequirements(aticiAltSistemi);
    }

    @Override
    public void initialize() {
        aticiAltSistemi.at();
        if (alimAltSistemi != null) {
            alimAltSistemi.depodanAticiyaYukariTasimaBaslat();
        }
    }

    @Override
    public void execute() {
        // Atisi surduR
    }

    @Override
    public void end(boolean interrupted) {
        aticiAltSistemi.durdur();
        if (alimAltSistemi != null) {
            alimAltSistemi.depodanAticiyaYukariTasimaDurdur();
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Gerekirse zamanlama ekle
    }
}
