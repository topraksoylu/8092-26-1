package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * PS4 controller profile where Axis 4/5 are not available.
 * Spin-up and fire are mapped to digital L2/R2 buttons.
 *
 * Axis mapping:
 *   0 = Left X (strafe)
 *   1 = Left Y (forward/back)
 *   2 = Right X (turn)
 *
 * Button mapping:
 *   1 = Square    -> intake
 *   2 = Cross     -> reverse intake
 *   3 = Circle    -> conveyor reverse unjam (0.5 s)
 *   4 = Triangle  -> shooter spin-up
 *   5 = L1        -> turret left (manual)
 *   6 = R1        -> turret right (manual)
 *   7 = L2 digital -> turret homing
 *   8 = R2 digital -> fire
 *   9 = Share     -> manual conveyor up
 *  10 = Options   -> gecikmeli atis (1 s spin-up + tasiyici)
 *  11 = PS        -> gyro reset
 *  12 = Touchpad  -> otomatik taret (whileTrue)
 */
public class PS4BasitProfili extends KontrolcuProfili {

    private static final int SOL_X_EKSEN    = 0;
    private static final int SOL_Y_EKSEN    = 1;
    private static final int SAG_X_EKSEN    = 2;

    private static final int ALIM_BTN       = 1;   // Square
    private static final int GERI_AT_BTN    = 2;   // Cross
    private static final int UNJAM_BTN      = 3;   // Circle
    private static final int SPINUP_BTN     = 4;   // Triangle (önceden homing idi)
    private static final int TARET_SOL_BTN  = 5;   // L1
    private static final int TARET_SAG_BTN  = 6;   // R1
    private static final int HOMING_BTN     = 7;   // L2 digital (önceden spinup idi)
    private static final int ATES_BTN       = 8;   // R2 digital
    private static final int TASIYICI_BTN   = 9;   // Share
    private static final int GECIKMELI_ATIS_BTN = 10; // Options
    private static final int GYRO_RESET_BTN = 11;  // PS
    private static final int SHOOTER_DIREKT_BTN = 12; // Touchpad

    public PS4BasitProfili(GenericHID hid) {
        super(hid);
    }

    @Override public double yanal()     { return eksenGuvenliOku(SOL_X_EKSEN); }
    @Override public double ileriGeri() { return eksenGuvenliOku(SOL_Y_EKSEN); }
    @Override public double donus()     { return -eksenGuvenliOku(SAG_X_EKSEN); }

    @Override public boolean aticiSpinupBasili() { return dugmeGuvenliOku(SPINUP_BTN); }
    @Override public boolean atesBasili()        { return dugmeGuvenliOku(ATES_BTN); }

    @Override public boolean alimBasili()         { return dugmeGuvenliOku(ALIM_BTN); }
    @Override public boolean geriAtBasili()       { return dugmeGuvenliOku(GERI_AT_BTN); }
    @Override public boolean tasiyiciBasili()     { return dugmeGuvenliOku(TASIYICI_BTN); }
    @Override public boolean tasiyiciTersBasili() { return dugmeGuvenliOku(UNJAM_BTN); }

    @Override public boolean taretSolaBasili()   { return dugmeGuvenliOku(TARET_SOL_BTN); }
    @Override public boolean taretSagaBasili()   { return dugmeGuvenliOku(TARET_SAG_BTN); }
    @Override public boolean taretHomingBasili() { return dugmeGuvenliOku(HOMING_BTN); }

    @Override public boolean gyroSifirlaBasili() { return dugmeGuvenliOku(GYRO_RESET_BTN); }
    @Override public boolean gecikmeliAtisBasili() { return dugmeGuvenliOku(GECIKMELI_ATIS_BTN); }
    @Override public boolean shooterDirektBasili() { return dugmeGuvenliOku(SHOOTER_DIREKT_BTN); }
}


