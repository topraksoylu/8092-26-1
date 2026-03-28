package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.GorusAltSistemi;
import frc.robot.Subsystems.TaretAltSistemi;

/**
 * Limelight tx kullanarak tareti sürekli atış hedefine hizalar.
 * Yalnızca mevcut ittifakın scoring taglarını (kırmızı: 2-5,8-11 / mavi: 18-21,24-27) takip eder.
 */
public class TaretTakipKomutu extends Command {
    private static final double KP = 0.01;
    // tx bu kadar derece altındaysa hizalanmış say
    private static final double HIZALAMA_ESIGI_DERECE = 2.0;

    private final TaretAltSistemi taretAltSistemi;
    private final GorusAltSistemi gorusAltSistemi;

    public TaretTakipKomutu(TaretAltSistemi taretAltSistemi, GorusAltSistemi gorusAltSistemi) {
        this.taretAltSistemi = taretAltSistemi;
        this.gorusAltSistemi = gorusAltSistemi;
        addRequirements(taretAltSistemi);
    }

    @Override
    public void execute() {
        if (gorusAltSistemi.isHedefTagGorunuyor()) {
            double tx = gorusAltSistemi.getHorizontalOffset();
            double hiz = tx * KP;
            taretAltSistemi.dondur(hiz);
            SmartDashboard.putBoolean("Taret/Hizalandi", Math.abs(tx) < HIZALAMA_ESIGI_DERECE);
            SmartDashboard.putNumber("Taret/TX", tx);
        } else {
            taretAltSistemi.durdur();
            SmartDashboard.putBoolean("Taret/Hizalandi", false);
        }
        SmartDashboard.putBoolean("Taret/HedefGorunuyor", gorusAltSistemi.isHedefTagGorunuyor());
    }

    @Override
    public void end(boolean interrupted) {
        taretAltSistemi.durdur();
    }

    @Override
    public boolean isFinished() {
        return false; // Sürekli takip
    }
}
