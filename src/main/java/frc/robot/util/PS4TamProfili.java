package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * PS4 kontrolcüsü — Axis 4 (L2 analog) ve Axis 5 (R2 analog) mevcut.
 *
 * Eksen haritası:
 *   0 = Sol X (yanal)
 *   1 = Sol Y (ileri/geri)
 *   2 = Sağ X (dönüş)
 *   3 = Sağ Y
 *   4 = L2 analog (0→1)  → atıcı spin-up
 *   5 = R2 analog (0→1)  → ateş (RobotKapsayici'de kilitle birleşir)
 *
 * Buton haritası (WPILib PS4 numaralandırması):
 *   1 = ■ Kare    → alım
 *   2 = ✕ Çarpı   → geri at
 *   3 = ● Daire   → taşıyıcı ters unjam (0.5 s)
 *   4 = ▲ Üçgen   → taşıyıcı yukarı manuel
 *   5 = L1        → taret sola (fallback)
 *   6 = R1        → taret sağa (fallback)
 *   7 = L2 dijital → taret homing
 *   8 = R2 dijital → gyro sıfırla
 *   9 = Share     → yedek
 *  10 = Options   → yedek
 */
public class PS4TamProfili extends KontrolcuProfili {

    private static final int SOL_X_EKSEN       = 0;
    private static final int SOL_Y_EKSEN       = 1;
    private static final int SAG_X_EKSEN       = 2;
    private static final int L2_ANALOG_EKSEN   = 4;
    private static final int R2_ANALOG_EKSEN   = 5;
    private static final double ANALOG_ESIK    = 0.1;

    private static final int ALIM_BTN          = 1;  // ■
    private static final int GERI_AT_BTN       = 2;  // ✕
    private static final int UNJAM_BTN         = 3;  // ●
    private static final int TASIYICI_BTN      = 4;  // ▲
    private static final int TARET_SOL_BTN     = 5;  // L1
    private static final int TARET_SAG_BTN     = 6;  // R1
    private static final int HOMING_BTN        = 7;  // L2 dijital
    private static final int GYRO_RESET_BTN    = 8;  // R2 dijital

    public PS4TamProfili(GenericHID hid) {
        super(hid);
    }

    @Override public double yanal()     { return eksenGuvenliOku(SOL_X_EKSEN); }
    @Override public double ileriGeri() { return eksenGuvenliOku(SOL_Y_EKSEN); }
    @Override public double donus()     { return -eksenGuvenliOku(SAG_X_EKSEN); }

    @Override public boolean aticiSpinupBasili() { return eksenGuvenliOku(L2_ANALOG_EKSEN) > ANALOG_ESIK; }
    @Override public boolean atesBasili()        { return eksenGuvenliOku(R2_ANALOG_EKSEN) > ANALOG_ESIK; }

    @Override public boolean alimBasili()          { return dugmeGuvenliOku(ALIM_BTN); }
    @Override public boolean geriAtBasili()        { return dugmeGuvenliOku(GERI_AT_BTN); }
    @Override public boolean tasiyiciBasili()      { return dugmeGuvenliOku(TASIYICI_BTN); }
    @Override public boolean tasiyiciTersBasili()  { return dugmeGuvenliOku(UNJAM_BTN); }

    @Override public boolean taretSolaBasili()   { return dugmeGuvenliOku(TARET_SOL_BTN); }
    @Override public boolean taretSagaBasili()   { return dugmeGuvenliOku(TARET_SAG_BTN); }
    @Override public boolean taretHomingBasili() { return dugmeGuvenliOku(HOMING_BTN); }

    @Override public boolean gyroSifirlaBasili() { return dugmeGuvenliOku(GYRO_RESET_BTN); }
}
