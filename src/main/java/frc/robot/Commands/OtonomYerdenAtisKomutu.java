package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sabitler.MotorSabitleri;
import frc.robot.Subsystems.AlimAltSistemi;
import frc.robot.Subsystems.AticiAltSistemi;
import frc.robot.Subsystems.GorusAltSistemi;
import frc.robot.Subsystems.TaretAltSistemi;
import frc.robot.util.AtisHesaplayici;

/**
 * Otonom "Yerden Ateş" komutu.
 *
 * Robot başlangıç konumunda sabit durur. Önceden doldurulmuş yakıtın
 * tamamını hedef yapıdaki AprilTag'lere kilitliyerek atar.
 *
 * ── Ön koşullar ──────────────────────────────────────────────────────────
 *   • TaretHomingKomutu tamamlanmış olmalı (enkoder kalibre).
 *     Limit switch yoksa veya homing timeout'a uğradıysa açık döngüye
 *     geçer — Limelight tx ile yine de hedefleme çalışır.
 *   • Limelight ittifak hedef tag'larından en az birini görüyor olmalı.
 *     İki tag görünürse MegaTag2 pose tahmini daha hassas olur — kod
 *     bundan otomatik olarak yararlanır.
 *
 * ── Faz sırası ───────────────────────────────────────────────────────────
 *
 *   HIZLANMA  (max HIZLANMA_ZAMAN_ASIMI_SN)
 *     • Limelight tx → tareti MAXMotion ile hizala.
 *     • Limelight mesafesi → AtisHesaplayici ile RPM spin-up.
 *     • |tx| < TX_TOLERANS ve atıcı RPM ±200 içinde → ATIS'e geç.
 *     • Zaman aşımında en iyi konumdan yine de ateş et.
 *
 *   ATIS  (ATIS_SURESI_SN)
 *     • Taşıyıcıyı başlat, yakıt akar.
 *     • Atıcı ve tareti aynı RPM + açıda tut.
 *     • Süre dolunca BITTI'ye geç.
 *
 *   BITTI
 *     • Tüm motorları durdur. Teleop'u bekle.
 *
 * ── SmartDashboard anahtarları ────────────────────────────────────────────
 *   Otonom/Faz           — aktif faz adı
 *   Otonom/FazSuresi_s   — bu fazda geçen süre
 *   Otonom/HedefRPM      — hesaplanan atıcı RPM
 *   Otonom/TaretHizali   — |tx| < tolerans mı?
 *   Otonom/AticiHazir    — isHizaUlasti() sonucu
 *   Otonom/AtisKalanSure — taşıyıcı kalan çalışma süresi (s)
 *   Otonom/TagSayisi     — o andaki görünen tag sayısı
 */
public class OtonomYerdenAtisKomutu extends Command {

    // ── Ayarlanabilir sabitler ────────────────────────────────────────────

    /** Taret hizalama toleransı: |tx| bu değerin altındaysa "hizalı" sayılır (°). */
    private static final double TX_TOLERANS_DERECE       = 1.5;

    /**
     * Hizalanma + spin-up için en fazla beklenecek süre.
     * Bu süre dolunca en iyi konumdan atış başlar (hedef kaysa bile).
     */
    private static final double HIZLANMA_ZAMAN_ASIMI_SN  = 5.0;

    /**
     * Taşıyıcının çalışacağı süre (tüm yakıtın bitmesi için).
     * 10 top × ~0.4 s/top ≈ 4 s; güvenli marj ile 5 s.
     */
    private static final double ATIS_SURESI_SN            = 5.0;

    /** Limelight görmüyorsa fallback atış mesafesi (m). */
    private static final double FALLBACK_MESAFE_METRE     = 2.8;

    // ── Alt sistemler ─────────────────────────────────────────────────────

    private final TaretAltSistemi taretAltSistemi;
    private final AticiAltSistemi aticiAltSistemi;
    private final AlimAltSistemi  alimAltSistemi;
    private final GorusAltSistemi gorusAltSistemi;

    // ── Durum ─────────────────────────────────────────────────────────────

    private enum Faz { HIZLANMA, ATIS, BITTI }

    private Faz    aktifFaz  = Faz.HIZLANMA;
    private double hedefRpm  = 0.0;
    private final Timer fazTimer = new Timer();

    // ─────────────────────────────────────────────────────────────────────

    public OtonomYerdenAtisKomutu(
            TaretAltSistemi taretAltSistemi,
            AticiAltSistemi aticiAltSistemi,
            AlimAltSistemi  alimAltSistemi,
            GorusAltSistemi gorusAltSistemi) {
        this.taretAltSistemi = taretAltSistemi;
        this.aticiAltSistemi = aticiAltSistemi;
        this.alimAltSistemi  = alimAltSistemi;
        this.gorusAltSistemi = gorusAltSistemi;
        addRequirements(taretAltSistemi, aticiAltSistemi, alimAltSistemi);
    }

    // ── WPILib command lifecycle ──────────────────────────────────────────

    @Override
    public void initialize() {
        aktifFaz = Faz.HIZLANMA;
        hedefRpm = 0.0;
        fazTimer.reset();
        fazTimer.start();
        SmartDashboard.putString("Otonom/Faz", "HIZLANMA");
        SmartDashboard.putBoolean("Otonom/TaretHizali", false);
        SmartDashboard.putBoolean("Otonom/AticiHazir",  false);
    }

    @Override
    public void execute() {
        switch (aktifFaz) {
            case HIZLANMA -> executeHizlanma();
            case ATIS     -> executeAtis();
            case BITTI    -> { /* durdur, bekle */ }
        }

        SmartDashboard.putString("Otonom/Faz",         aktifFaz.name());
        SmartDashboard.putNumber("Otonom/FazSuresi_s", fazTimer.get());
        SmartDashboard.putNumber("Otonom/HedefRPM",    hedefRpm);
    }

    @Override
    public void end(boolean interrupted) {
        aticiAltSistemi.durdur();
        alimAltSistemi.depodanAticiyaYukariTasimaDurdur();
        taretAltSistemi.durdur();
        fazTimer.stop();
        SmartDashboard.putString("Otonom/Faz", interrupted ? "KESILDI" : "TAMAMLANDI");
    }

    @Override
    public boolean isFinished() {
        return aktifFaz == Faz.BITTI;
    }

    // ── Faz yöneticileri ──────────────────────────────────────────────────

    /**
     * Faz 1: Taret hizalama + atıcı spin-up.
     * Her döngüde:
     *   • Limelight mesafesiyle RPM güncelle → atıcıya gönder.
     *   • tx'e göre taret açısını düzelt (MAXMotion veya açık döngü).
     *   • Hazırlık koşulları sağlanınca veya timeout'ta ATIS'e geç.
     */
    private void executeHizlanma() {
        boolean limelightGoruyor = gorusAltSistemi.isHedefTagGorunuyor();

        // 1 — RPM hesapla (limelight mesafesi → tablo interpolasyonu)
        if (limelightGoruyor) {
            double mesafe = gorusAltSistemi.getDistanceToTarget();
            hedefRpm = AtisHesaplayici.hesaplaHedefRpm(mesafe);
        } else if (hedefRpm == 0.0) {
            hedefRpm = AtisHesaplayici.hesaplaHedefRpm(FALLBACK_MESAFE_METRE);
        }
        aticiAltSistemi.atRPM(hedefRpm);

        // 2 — Taret hizalama
        hedefeTaretAyarla(limelightGoruyor);

        // 3 — Hazırlık kontrolü
        double tx        = gorusAltSistemi.getHorizontalOffset();
        boolean hizali   = limelightGoruyor && Math.abs(tx) < TX_TOLERANS_DERECE;
        boolean rpmHazir = aticiAltSistemi.isHizaUlasti();
        boolean timeout  = fazTimer.hasElapsed(HIZLANMA_ZAMAN_ASIMI_SN);

        SmartDashboard.putBoolean("Otonom/TaretHizali", hizali);
        SmartDashboard.putBoolean("Otonom/AticiHazir",  rpmHazir);

        if ((hizali && rpmHazir) || timeout) {
            gecisYapAtis();
        }
    }

    /**
     * Faz 2: Taşıyıcı çalışıyor — yakıt akar.
     * Atıcı RPM ve taret konumu aynı değerlerde tutulur.
     */
    private void executeAtis() {
        // Atıcıyı ve tareti koru — RPM düşmesin, hedef kaymasın
        aticiAltSistemi.atRPM(hedefRpm);
        hedefeTaretAyarla(gorusAltSistemi.isHedefTagGorunuyor());

        double kalanSure = ATIS_SURESI_SN - fazTimer.get();
        SmartDashboard.putNumber("Otonom/AtisKalanSure_s", kalanSure);

        if (fazTimer.hasElapsed(ATIS_SURESI_SN)) {
            gecisYapBitti();
        }
    }

    // ── Yardımcılar ───────────────────────────────────────────────────────

    /**
     * Tareti Limelight tx'e göre hizalar.
     * Homing tamamlandıysa MAXMotion position control, değilse açık döngü.
     */
    private void hedefeTaretAyarla(boolean limelightGoruyor) {
        if (!limelightGoruyor) return; // görünmüyorsa mevcut konumda bekle

        double tx = gorusAltSistemi.getHorizontalOffset();

        if (taretAltSistemi.isHomingTamamlandi()) {
            // MAXMotion — yumuşak trapezoidal hareket
            double hedefAci = taretAltSistemi.getAci() + tx;
            hedefAci = Math.max(MotorSabitleri.TARET_MIN_ACI,
                         Math.min(MotorSabitleri.TARET_MAKS_ACI, hedefAci));
            taretAltSistemi.aciAyarla(hedefAci);
        } else {
            // Homing yapılmamış — orantılı açık döngü (enkodere bağımlı değil)
            double hiz = tx * 0.02; // kP = 0.02; tx derece → motor hız oranı
            taretAltSistemi.dondur(hiz);
        }
    }

    /** HIZLANMA → ATIS geçişi. */
    private void gecisYapAtis() {
        aktifFaz = Faz.ATIS;
        fazTimer.reset();
        fazTimer.start();
        alimAltSistemi.depodanAticiyaYukariTasimaBaslat();
        SmartDashboard.putString("Otonom/Faz", "ATIS");
    }

    /** ATIS → BITTI geçişi. */
    private void gecisYapBitti() {
        aktifFaz = Faz.BITTI;
        fazTimer.stop();
        alimAltSistemi.depodanAticiyaYukariTasimaDurdur();
        aticiAltSistemi.durdur();
        taretAltSistemi.durdur();
        SmartDashboard.putString("Otonom/Faz", "BITTI");
        SmartDashboard.putNumber("Otonom/AtisKalanSure_s", 0.0);
    }
}
