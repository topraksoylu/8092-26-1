package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * Xbox kontrolcüsü (standart WPILib axis/buton numaralandırması).
 *
 * Eksen haritası:
 *   0 = Sol X (yanal)
 *   1 = Sol Y (ileri/geri)
 *   2 = Sol Tetik LT analog (0→1)  → atıcı spin-up
 *   3 = Sağ Tetik RT analog (0→1)  → ateş
 *   4 = Sağ X (dönüş)
 *   5 = Sağ Y
 *
 * Buton haritası:
 *   1 = A        → alım
 *   2 = B        → geri at
 *   3 = X        → taşıyıcı ters unjam (0.5 s)
 *   4 = Y        → taşıyıcı yukarı manuel
 *   5 = LB       → taret sola (fallback)
 *   6 = RB       → taret sağa (fallback)
 *   7 = Back     → taret homing
 *   8 = Start    → gyro sıfırla
 */
public class XboxProfili extends KontrolcuProfili {

    private static final int SOL_X_EKSEN    = 0;
    private static final int SOL_Y_EKSEN    = 1;
    private static final int LT_EKSEN       = 2;
    private static final int RT_EKSEN       = 3;
    private static final int SAG_X_EKSEN    = 4;
    private static final double ANALOG_ESIK = 0.1;

    private static final int ALIM_BTN       = 1;   // A
    private static final int GERI_AT_BTN    = 2;   // B
    private static final int UNJAM_BTN      = 3;   // X
    private static final int TASIYICI_BTN   = 4;   // Y
    private static final int TARET_SOL_BTN  = 5;   // LB
    private static final int TARET_SAG_BTN  = 6;   // RB
    private static final int HOMING_BTN     = 7;   // Back
    private static final int GYRO_RESET_BTN = 8;   // Start

    public XboxProfili(GenericHID hid) {
        super(hid);
    }

    @Override public double yanal()     { return eksenGuvenliOku(SOL_X_EKSEN); }
    @Override public double ileriGeri() { return eksenGuvenliOku(SOL_Y_EKSEN); }
    @Override public double donus()     { return -eksenGuvenliOku(SAG_X_EKSEN); }

    @Override public boolean aticiSpinupBasili() { return eksenGuvenliOku(LT_EKSEN) > ANALOG_ESIK; }
    @Override public boolean atesBasili()        { return eksenGuvenliOku(RT_EKSEN) > ANALOG_ESIK; }

    @Override public boolean alimBasili()         { return dugmeGuvenliOku(ALIM_BTN); }
    @Override public boolean geriAtBasili()       { return dugmeGuvenliOku(GERI_AT_BTN); }
    @Override public boolean tasiyiciBasili()     { return dugmeGuvenliOku(TASIYICI_BTN); }
    @Override public boolean tasiyiciTersBasili() { return dugmeGuvenliOku(UNJAM_BTN); }

    @Override public boolean taretSolaBasili()   { return dugmeGuvenliOku(TARET_SOL_BTN); }
    @Override public boolean taretSagaBasili()   { return dugmeGuvenliOku(TARET_SAG_BTN); }
    @Override public boolean taretHomingBasili() { return dugmeGuvenliOku(HOMING_BTN); }

    @Override public boolean gyroSifirlaBasili() { return dugmeGuvenliOku(GYRO_RESET_BTN); }
}
