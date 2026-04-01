package frc.robot.util;

import frc.robot.Sabitler.AticiLauncherSabitleri;
import frc.robot.Sabitler.ModulSabitleri;

/**
 * am-5780_CN "Launcher in a Box" için atış mesafesi ↔ motor RPM hesaplama sınıfı.
 *
 * ── Donanım (am-5780_CN) ──────────────────────────────────────────────────────
 *   Teker çapı  : 4" Stealth Wheel = 0.1016 m
 *   Kayış oranı : 24T motor / 45T mil = 1.875:1 azaltma
 *   Verimlilik  : η ≈ 0.55 (basınç + kayma; saha kalibrasyonuyla doğrulandı)
 *   Atış açısı  : 45° (yan plakaların 3. churro pozisyonu)
 *
 * ── Balistik model (hava direnci ihmal) ───────────────────────────────────────
 *
 *   Yüzey hızı (teorik):
 *     v_yuzey = π × d × (RPM / oran) / 60
 *
 *   Çıkış hızı (verimlilik dahil):
 *     v_cikis = η × v_yuzey
 *
 *   Yatay / dikey bileşenler (θ = 45°):
 *     vx = v_cikis × cos θ
 *     vy = v_cikis × sin θ
 *
 *   Uçuş süresi (launcher yüksekliği h0, hedef yüksekliği h_t):
 *     Δh = h_t - h0
 *     t  = (vy + √(vy² − 2g·Δh)) / g
 *     (gerçek kök alınabilmesi için vy² ≥ 2g·Δh gerekir)
 *
 *   Yatay menzil:
 *     R = vx × t
 *
 * ── Kullanım ──────────────────────────────────────────────────────────────────
 *
 *   // Mesafeden RPM (birincil yol — kalibre edilmiş tablo):
 *   double rpm = AtisHesaplayici.hesaplaHedefRpm(mesafeMetre);
 *
 *   // Fizik tabanlı hesap (tablo yoksa / çapraz kontrol):
 *   double rpm = AtisHesaplayici.menzildenRpm(mesafeMetre);
 *
 *   // Belirli RPM'de teorik menzil:
 *   double menzil = AtisHesaplayici.rpmToMenzil(rpm);
 *
 * ── Kalibrasyon rehberi ───────────────────────────────────────────────────────
 *   1. Sabit bir mesafeye (ör. 2.8 m) çizgi çek.
 *   2. AtisHesaplayici.menzildenRpm(2.8) → fizik tahmini RPM'ini gör.
 *   3. O RPM'i manuel olarak gir; topun düştüğü noktayı ölç.
 *   4. Fark varsa AticiLauncherSabitleri.VERIMLILIK sabitini ayarla.
 *   5. Birkaç mesafe noktası için tekrarla; ardından ATIS_RPM_TABLOSU'nu güncelle.
 */
public final class AtisHesaplayici {

    private AtisHesaplayici() {}

    // ── Türetilmiş sabitler (her çağrıda yeniden hesaplamaktan kaçın) ─────────

    private static final double ATIS_ACISI_RAD =
        Math.toRadians(AticiLauncherSabitleri.ATIS_ACISI_DERECE);
    private static final double COS_TETA = Math.cos(ATIS_ACISI_RAD);
    private static final double SIN_TETA = Math.sin(ATIS_ACISI_RAD);

    /**
     * Yüzey hızından çıkış hızına dönüşüm katsayısı.
     * v_yuzey = k_yuzey × RPM
     * k_yuzey = π × d / (oran × 60)
     */
    private static final double K_YUZEY =
        Math.PI * AticiLauncherSabitleri.TEKER_CAPI_METRE
        / (AticiLauncherSabitleri.DISLI_ORANI * 60.0);

    /** v_cikis = K_CIKIS × RPM */
    private static final double K_CIKIS = AticiLauncherSabitleri.VERIMLILIK * K_YUZEY;

    private static final double G   = 9.81;
    private static final double D_H =
        AticiLauncherSabitleri.HEDEF_YUKSEKLIK - AticiLauncherSabitleri.LAUNCHER_YUKSEKLIK;

    // ── Herkese açık fizik metodları ──────────────────────────────────────────

    /**
     * Motor RPM → yüzey hızı (teorik, m/s).
     * Gerçek çıkış hızı için {@link #rpmToCikisHizi(double)} kullan.
     */
    public static double rpmToYuzeyHizi(double rpm) {
        return K_YUZEY * rpm;
    }

    /**
     * Motor RPM → top çıkış hızı (verimlilik dahil, m/s).
     */
    public static double rpmToCikisHizi(double rpm) {
        return K_CIKIS * rpm;
    }

    /**
     * Motor RPM → yatay menzil (m).
     *
     * Hedef yüksekliğine ulaşmak için yeterli vy yoksa {@code Double.NaN} döner.
     * NaN sınırı: RPM < {@link #minAtisRpm()} ≈ 2675 RPM (η=0.55, θ=45°, Δh=0.443 m).
     */
    public static double rpmToMenzil(double rpm) {
        double vCikis = rpmToCikisHizi(rpm);
        double vx = vCikis * COS_TETA;
        double vy = vCikis * SIN_TETA;
        double disc = vy * vy - 2.0 * G * D_H;
        if (disc < 0.0) return Double.NaN;
        double t = (vy + Math.sqrt(disc)) / G;
        return vx * t;
    }

    /**
     * Hedef mesafe (m) → gereken motor RPM.
     *
     * İkili arama ile balistik formülün tersi çözülür.
     * Tablo yokken veya çapraz kontrol için kullanılır.
     *
     * @param mesafeMetre  Hedefe yatay uzaklık (m)
     * @return             Gereken motor RPM
     */
    public static double menzildenRpm(double mesafeMetre) {
        // Arama aralığı: min-RPM'den NEO serbest hızına
        double lo = minAtisRpm();
        double hi = 5676.0;
        // 64 iterasyon → ~1e-15 hassasiyet (pratikte ilk 30'da yakınsar)
        for (int i = 0; i < 64; i++) {
            double mid    = (lo + hi) * 0.5;
            double menzil = rpmToMenzil(mid);
            if (Double.isNaN(menzil) || menzil < mesafeMetre) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return (lo + hi) * 0.5;
    }

    /**
     * Verilen verimlilik ve açı için hedefe ulaşabilen minimum RPM.
     * vy_min = √(2g·Δh)
     */
    public static double minAtisRpm() {
        return minAtisRpm(D_H, K_CIKIS, SIN_TETA);
    }

    static double minAtisRpm(double deltaH, double cikisKatsayisi, double sinTeta) {
        if (deltaH <= 0.0) return 0.0; // hedef launcher'dan alçakta — her RPM'de ulaşır
        double vyMin = Math.sqrt(2.0 * G * deltaH);
        return vyMin / (cikisKatsayisi * sinTeta);
    }

    // ── Birincil metot: kalibre edilmiş tablo + fizik fallback ────────────────

    /**
     * Mesafeden hedef RPM hesaplar.
     *
     * <b>Öncelik sırası:</b>
     * <ol>
     *   <li>Kalibre edilmiş {@code ATIS_RPM_TABLOSU} lineer interpolasyon</li>
     *   <li>Tablo geçersizse/boşsa → {@link #menzildenRpm(double)} fizik hesabı</li>
     * </ol>
     *
     * @param mesafeMetre  Hedefe yatay uzaklık (m)
     * @return             Motor hedef RPM
     */
    public static double hesaplaHedefRpm(double mesafeMetre) {
        double[] mesafeler = ModulSabitleri.ATIS_MESAFE_TABLOSU_METRE;
        double[] rpmler    = ModulSabitleri.ATIS_RPM_TABLOSU;

        // Tablo geçerlilik kontrolü
        if (mesafeler == null || rpmler == null
                || mesafeler.length == 0 || mesafeler.length != rpmler.length) {
            return menzildenRpm(mesafeMetre);
        }

        // Tablo alt/üst sınır dışı: klamp
        if (mesafeMetre <= mesafeler[0])                    return rpmler[0];
        int son = mesafeler.length - 1;
        if (mesafeMetre >= mesafeler[son])                  return rpmler[son];

        // Lineer interpolasyon
        for (int i = 0; i < son; i++) {
            if (mesafeMetre >= mesafeler[i] && mesafeMetre <= mesafeler[i + 1]) {
                double oran = (mesafeMetre - mesafeler[i]) / (mesafeler[i + 1] - mesafeler[i]);
                return rpmler[i] + (rpmler[i + 1] - rpmler[i]) * oran;
            }
        }

        // Buraya normalde gelinmez — güvenli fallback
        return menzildenRpm(mesafeMetre);
    }
}
