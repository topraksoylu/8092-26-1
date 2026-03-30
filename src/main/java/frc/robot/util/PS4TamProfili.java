package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * PS4 controller profile where Axis 4 (L2 analog) and Axis 5 (R2 analog) exist.
 *
 * Axis mapping:
 *   0 = Left X (strafe)
 *   1 = Left Y (forward/back)
 *   2 = Right X (turn)
 *   3 = Right Y
 *   4 = L2 analog (0->1) -> shooter spin-up
 *   5 = R2 analog (0->1) -> fire
 *
 * Button mapping (WPILib PS4 numbering):
 *   1 = Square    -> intake
 *   2 = Cross     -> reverse intake
 *   3 = Circle    -> conveyor reverse unjam (0.5 s)
 *   4 = Triangle  -> manual conveyor up
 *   5 = L1        -> turret left (fallback)
 *   6 = R1        -> turret right (fallback)
 *   7 = L2 digital -> turret homing
 *   8 = R2 digital -> gyro reset
 *   9 = Share     -> spare
 *  10 = Options   -> gecikmeli atis (1 s spin-up + tasiyici)
 *  12 = Touchpad  -> shooter direkt
 */
public class PS4TamProfili extends KontrolcuProfili {

    private static final int SOL_X_EKSEN       = 0;
    private static final int SOL_Y_EKSEN       = 1;
    private static final int SAG_X_EKSEN       = 2;
    private static final int L2_ANALOG_EKSEN   = 4;
    private static final int R2_ANALOG_EKSEN   = 5;
    private static final double ANALOG_ESIK    = 0.1;

    private static final int ALIM_BTN          = 1;  // Square
    private static final int GERI_AT_BTN       = 2;  // Cross
    private static final int UNJAM_BTN         = 3;  // Circle
    private static final int TASIYICI_BTN      = 4;  // Triangle
    private static final int TARET_SOL_BTN     = 5;  // L1
    private static final int TARET_SAG_BTN     = 6;  // R1
    private static final int HOMING_BTN        = 7;  // L2 digital
    private static final int GYRO_RESET_BTN    = 8;  // R2 digital
    private static final int GECIKMELI_ATIS_BTN = 10; // Options
    private static final int SHOOTER_DIREKT_BTN = 12; // Touchpad

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
    @Override public boolean gecikmeliAtisBasili() { return dugmeGuvenliOku(GECIKMELI_ATIS_BTN); }
    @Override public boolean shooterDirektBasili() { return dugmeGuvenliOku(SHOOTER_DIREKT_BTN); }
}


