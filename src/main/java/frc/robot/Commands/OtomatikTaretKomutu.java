package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Sabitler.MotorSabitleri;
import frc.robot.Subsystems.GorusAltSistemi;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.TaretAltSistemi;

/**
 * Tareti otomatik olarak hedefe hizalar. TaretAltSistemi'nin default command'ıdır.
 *
 * Öncelik sırası:
 *   1. Limelight ittifak hedef tag'ini görüyorsa → tx ile doğrudan düzelt
 *   2. Hedef görünmüyorsa → odometri + hub koordinatıyla hesapla (NavX fallback)
 *
 * L1 / R1 tuşlarına basılınca bu komut kesilir (interrupted), bırakılınca
 * scheduler otomatik olarak default command'ı yeniden başlatır.
 *
 * Homing tamamlanmadan çalışmaz — enkoder sıfırlanmamışsa taret durdurulur.
 */
public class OtomatikTaretKomutu extends Command {

    /** Vision modunda tx başına hedef açı katkısı (1.0 = doğrudan derece). */
    private static final double TX_KP = 1.0;

    private final TaretAltSistemi taretAltSistemi;
    private final GorusAltSistemi gorusAltSistemi;
    private final SurusAltSistemi surusAltSistemi;

    private enum Mod { VISION, ODOMETRI, HOMING_GEREKLI }
    private Mod aktifMod = Mod.HOMING_GEREKLI;

    public OtomatikTaretKomutu(
            TaretAltSistemi taretAltSistemi,
            GorusAltSistemi gorusAltSistemi,
            SurusAltSistemi surusAltSistemi) {
        this.taretAltSistemi = taretAltSistemi;
        this.gorusAltSistemi = gorusAltSistemi;
        this.surusAltSistemi = surusAltSistemi;
        addRequirements(taretAltSistemi);
    }

    @Override
    public void execute() {
        if (!taretAltSistemi.isHomingTamamlandi()) {
            aktifMod = Mod.HOMING_GEREKLI;
            taretAltSistemi.durdur();
            SmartDashboard.putString("Taret/OtoMod", "HOMING_GEREKLI");
            SmartDashboard.putBoolean("Taret/OtoAktif", false);
            return;
        }

        double hedefAci;

        if (gorusAltSistemi.isHedefTagGorunuyor()) {
            // Mod 1: Limelight vision — tx derece cinsinden açı hatası
            aktifMod = Mod.VISION;
            double tx = gorusAltSistemi.getHorizontalOffset();
            hedefAci = taretAltSistemi.getAci() + tx * TX_KP;
        } else {
            // Mod 2: Odometri fallback — robot pozu + hub koordinatı
            aktifMod = Mod.ODOMETRI;
            hedefAci = odometriHedefAcisiHesapla();
        }

        // ±90° yazılım sınırına kırp
        hedefAci = Math.max(MotorSabitleri.TARET_MIN_ACI,
                    Math.min(MotorSabitleri.TARET_MAKS_ACI, hedefAci));

        taretAltSistemi.aciAyarla(hedefAci);

        SmartDashboard.putString("Taret/OtoMod", aktifMod.name());
        SmartDashboard.putBoolean("Taret/OtoAktif", true);
    }

    @Override
    public void end(boolean interrupted) {
        taretAltSistemi.durdur();
        SmartDashboard.putBoolean("Taret/OtoAktif", false);
        SmartDashboard.putString("Taret/OtoMod", "MANUEL");
    }

    @Override
    public boolean isFinished() {
        return false; // Sürekli çalışır — L1/R1 interrupt eder
    }

    /** Robot pozu ve ittifak hub koordinatından taret hedef açısını hesaplar. */
    private double odometriHedefAcisiHesapla() {
        Pose2d robotPoz = surusAltSistemi.getPose();

        boolean kirmizi = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        Translation2d hub = kirmizi ? FieldConstants.HUB_KIRMIZI : FieldConstants.HUB_MAVI;

        double dx = hub.getX() - robotPoz.getX();
        double dy = hub.getY() - robotPoz.getY();
        double sahaAcisi = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = robotPoz.getRotation().getDegrees();

        // Taret öne bakıyor → offset = 0°
        double taretHedef = sahaAcisi - robotHeading - MotorSabitleri.TARET_ON_OFFSET_DERECE;

        // −180° ile +180° arasına normalize et
        while (taretHedef > 180.0)  taretHedef -= 360.0;
        while (taretHedef < -180.0) taretHedef += 360.0;

        return taretHedef;
    }
}
