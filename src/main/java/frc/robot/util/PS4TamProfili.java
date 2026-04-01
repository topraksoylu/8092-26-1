package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * PS4 controller profile where Axis 4 (L2 analog) and Axis 5 (R2 analog) exist.
 *
 * Axis mapping:
 *   0 = Left X (strafe)
 *   1 = Left Y (forward/back)
 *   2 = Right X (turn)
 *   3 = Right Y
 *   4 = L2 analog (0->1)
 *   5 = R2 analog (0->1)
 *
 * Button mapping (WPILib PS4 numbering):
 *   1 = Square    -> intake
 *   2 = Cross     -> reverse intake
 *   3 = Circle    -> conveyor reverse unjam (0.5 s)
 *   4 = Triangle  -> taşıcı (manuel)
 *   5 = L1        -> (boş)
 *   6 = R1        -> (boş)
 *   7 = L2 digital -> (boş)
 *   8 = R2 digital -> limelight hizalama + otonom atış
 *   9 = Share     -> gyro reset
 *   10 = Options   -> gecikmeli atış
 *   12 = Touchpad  -> shooter direkt
 *
 * D-Pad (POV) mapping:
 *   0° (Up)    -> Yakın atış (~1.2m → 2750 RPM)
 *   90° (Right)-> Orta atış (~2.8m → 3700 RPM)
 *   180° (Down)-> Uzak atış (~4.4m → 4490 RPM)
 */
public class PS4TamProfili extends KontrolcuProfili {

    private static final int SOL_X_EKSEN       = 0;
    private static final int SOL_Y_EKSEN       = 1;
    private static final int SAG_X_EKSEN       = 2;
    private static final int L2_ANALOG_EKSEN   = 4;
    private static final int R2_ANALOG_EKSEN   = 5;

    private static final int ALIM_BTN           = 1;  // Square
    private static final int GERI_AT_BTN        = 2;  // Cross
    private static final int UNJAM_BTN          = 3;  // Circle
    private static final int TASIYICI_BTN       = 4;  // Triangle
    private static final int GYRO_RESET_BTN     = 9;  // Share
    private static final int GECIKMELI_ATIS_BTN = 10; // Options
    private static final int SHOOTER_DIREKT_BTN = 12; // Touchpad

    // D-Pad POV açıları
    private static final int POV_UP    = 0;
    private static final int POV_RIGHT = 90;
    private static final int POV_DOWN  = 180;
    private static final int POV_LEFT  = 270;

    public PS4TamProfili(GenericHID hid) {
        super(hid);
    }

    @Override public double yanal()     { return eksenGuvenliOku(SOL_X_EKSEN); }
    @Override public double ileriGeri() { return eksenGuvenliOku(SOL_Y_EKSEN); }
    @Override public double donus()     { return eksenGuvenliOku(SAG_X_EKSEN); }

    @Override public boolean yakinAtisBasili()    { return povOku() == POV_UP; }
    @Override public boolean ortaAtisBasili()     { return povOku() == POV_RIGHT; }
    @Override public boolean uzakAtisBasili()     { return povOku() == POV_DOWN; }
    @Override public boolean cokUzakAtisBasili()  { return povOku() == POV_LEFT; }

    @Override public boolean alimBasili()          { return dugmeGuvenliOku(ALIM_BTN); }
    @Override public boolean geriAtBasili()        { return dugmeGuvenliOku(GERI_AT_BTN); }
    @Override public boolean tasiyiciBasili()      { return dugmeGuvenliOku(TASIYICI_BTN); }
    @Override public boolean tasiyiciTersBasili()  { return dugmeGuvenliOku(UNJAM_BTN); }

    @Override public boolean gyroSifirlaBasili() { return dugmeGuvenliOku(GYRO_RESET_BTN); }
    @Override public boolean gecikmeliAtisBasili() { return dugmeGuvenliOku(GECIKMELI_ATIS_BTN); }
    @Override public boolean shooterDirektBasili() { return dugmeGuvenliOku(SHOOTER_DIREKT_BTN); }

    /** D-Pad (POV) açısını güvenli şekilde okur. Merkezde değilse -1 döner. */
    private int povOku() {
        int port = hid.getPort();
        if (!DriverStation.isJoystickConnected(port)) return -1;
        return hid.getPOV();
    }
}
