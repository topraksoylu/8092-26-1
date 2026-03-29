package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Sabitler.MotorSabitleri;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.TaretAltSistemi;

/**
 * Robot pozundan hub'a açı hesaplayarak tareti sürekli hizalar.
 *
 * tx PID yerine poz tabanlı yaklaşım:
 *   1. surusAltSistemi.getPose() → fused odometri + Limelight pozu
 *   2. İttifaka göre hub Translation2d seç (FieldConstants)
 *   3. atan2(dy, dx) → sahaya göre açı
 *   4. Robot heading çıkar, taret ön offset ekle (0° — taret öne bakıyor)
 *   5. ±90° sınırına kırp, taretAltSistemi.dondur() ile komut ver
 *
 * Tag görünmese bile odometri devam ettiğinden takip kesilmez.
 *
 * NOT: Şu an otomatik kullanım devre dışı. Bu komut yalnızca test amaçlı bağlanabilir.
 */
public class PozTabanliTaretKomutu extends Command {
    private final TaretAltSistemi taretAltSistemi;
    private final SurusAltSistemi surusAltSistemi;

    public PozTabanliTaretKomutu(TaretAltSistemi taretAltSistemi, SurusAltSistemi surusAltSistemi) {
        this.taretAltSistemi = taretAltSistemi;
        this.surusAltSistemi = surusAltSistemi;
        addRequirements(taretAltSistemi);
    }

    @Override
    public void execute() {
        Pose2d robotPoz = surusAltSistemi.getPose();

        // İttifaka göre hedef hub seç
        boolean kirmizi = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        Translation2d hub = kirmizi ? FieldConstants.HUB_KIRMIZI : FieldConstants.HUB_MAVI;

        // Hub yönü — saha koordinatlarında açı (derece)
        double dx = hub.getX() - robotPoz.getX();
        double dy = hub.getY() - robotPoz.getY();
        double sahaAcisiDerece = Math.toDegrees(Math.atan2(dy, dx));

        // Robot heading'i çıkar → robot-relative açı
        double robotHeadingDerece = robotPoz.getRotation().getDegrees();

        // Taret öne bakıyor: 0° offset — 0° = robotun ön yönü
        double taretHedefAcisi = sahaAcisiDerece - robotHeadingDerece
            - MotorSabitleri.TARET_ON_OFFSET_DERECE;

        // Açıyı −180° ile +180° arasına normalize et
        taretHedefAcisi = normalizeAci(taretHedefAcisi);

        // ±90° yazılım sınırına kırp
        taretHedefAcisi = Math.max(MotorSabitleri.TARET_MIN_ACI,
                          Math.min(MotorSabitleri.TARET_MAKS_ACI, taretHedefAcisi));

        // Oransal kontrol: hata → motor hızı
        double mevcutAci = taretAltSistemi.getAci();
        double hata = taretHedefAcisi - mevcutAci;
        double hiz = hata * MotorSabitleri.TARET_POZ_KP;
        taretAltSistemi.dondur(hiz);

        boolean hizalandi = Math.abs(hata) < MotorSabitleri.TARET_HIZALAMA_ESIGI_DERECE;
        SmartDashboard.putBoolean("Taret/Hizalandi", hizalandi);
        SmartDashboard.putNumber("Taret/HedefAci", taretHedefAcisi);
        SmartDashboard.putNumber("Taret/MevcutAci", mevcutAci);
        SmartDashboard.putNumber("Taret/Hata", hata);
    }

    @Override
    public void end(boolean interrupted) {
        taretAltSistemi.durdur();
    }

    @Override
    public boolean isFinished() {
        return false; // Sürekli takip
    }

    /** Açıyı −180° ile +180° arasına normalize eder */
    private static double normalizeAci(double derece) {
        while (derece > 180.0)  derece -= 360.0;
        while (derece < -180.0) derece += 360.0;
        return derece;
    }
}
