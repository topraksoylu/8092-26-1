package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Sürücü kontrolcüsünü soyutlar.
 * PS4 (tam/basit) ve Xbox için farklı alt sınıflar mevcuttur.
 * Elastic veya SmartDashboard'daki SendableChooser ile runtime'da seçilebilir.
 *
 * SurusKomutu'nun beklentisi:
 *   xHizi supplier → yanal()   (Axis 0 ham değer — SurusKomutu içinde negatif alınır)
 *   yHizi supplier → ileriGeri() (Axis 1 ham değer — SurusKomutu içinde negatif alınır)
 *   zDonusu supplier → donus()  (önceden negatiflenmiş dönüş ekseni)
 */
public abstract class KontrolcuProfili {

    protected final GenericHID hid;

    protected KontrolcuProfili(GenericHID hid) {
        this.hid = hid;
    }

    // ── Sürüş eksenleri ──────────────────────────────────────────────────────
    /** Sol analog X ekseni ham değeri (SurusKomutu içinde negatif alınır). */
    public abstract double yanal();

    /** Sol analog Y ekseni ham değeri (SurusKomutu içinde negatif alınır). */
    public abstract double ileriGeri();

    /**
     * Dönüş ekseni — profil içinde önceden negatiflenmiş döner.
     * Sağ analog X → saat yönü = negatif (WPILib CCW-pozitif ile uyumlu).
     */
    public abstract double donus();

    // ── Atıcı ────────────────────────────────────────────────────────────────
    /** Yakın atış tetikleyici (~1.2m → 2750 RPM). */
    public abstract boolean yakinAtisBasili();

    /** Orta atış tetikleyici (~2.8m → 3700 RPM). */
    public abstract boolean ortaAtisBasili();

    /** Uzak atış tetikleyici (~2.8m). */
    public abstract boolean uzakAtisBasili();

    /** Çok uzak atış tetikleyici (~4.4m). */
    public abstract boolean cokUzakAtisBasili();

    /** @deprecated Kullanma - 3 atış butonu kullan */
    @Deprecated
    public boolean aticiSpinupBasili() { return false; }

    /** @deprecated Kullanma - 3 atış butonu kullan */
    @Deprecated
    public boolean atesBasili() { return false; }

    /** Shooter motorunu dogrudan calistiran dijital tus (opsiyonel). */
    public boolean shooterDirektBasili() { return false; }

    // ── Alım / taşıyıcı ──────────────────────────────────────────────────────
    public abstract boolean alimBasili();
    public abstract boolean geriAtBasili();
    /** Taşıyıcı yukarı (manuel, kilitsiz). */
    public abstract boolean tasiyiciBasili();
    /** Taşıyıcı ters — unjam (0.5 s). */
    public abstract boolean tasiyiciTersBasili();

    // ── Sistem ───────────────────────────────────────────────────────────────
    public abstract boolean gyroSifirlaBasili();
    /** Opsiyonel: tek tusla gecikmeli atis sirasi (spin-up + gecikmeli tasiyici). */
    public boolean gecikmeliAtisBasili() { return false; }

    // ── Geri bildirim ─────────────────────────────────────────────────────────
    /** Kontrolcü titreşimi (0.0 = kapalı, 1.0 = tam). */
    public void titrestir(double yogunluk) {
        hid.setRumble(GenericHID.RumbleType.kBothRumble, yogunluk);
    }

    public GenericHID getHID() {
        return hid;
    }

    // ── Güvenli okuma yardımcıları ────────────────────────────────────────────
    protected double eksenGuvenliOku(int eksen) {
        int port = hid.getPort();
        if (!DriverStation.isJoystickConnected(port)) return 0.0;
        if (eksen < 0 || eksen >= DriverStation.getStickAxisCount(port)) return 0.0;
        return hid.getRawAxis(eksen);
    }

    protected boolean dugmeGuvenliOku(int dugme) {
        int port = hid.getPort();
        if (!DriverStation.isJoystickConnected(port)) return false;
        if (dugme <= 0 || dugme > 32) return false;
        return (DriverStation.getStickButtons(port) & (1 << (dugme - 1))) != 0;
    }
}


