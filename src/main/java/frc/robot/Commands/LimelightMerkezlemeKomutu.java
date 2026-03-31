package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.GorusAltSistemi;
import frc.robot.Subsystems.SurusAltSistemi;

/**
 * Robotu Limelight tx hatasina gore saga/sola dondurerek hedefi merkeze alir.
 * Sadece istenen merkez taglari (9,10,25,26) geldiginde aktif olur.
 */
public class LimelightMerkezlemeKomutu extends Command {
    private static final Set<Integer> MERKEZ_TAGLER = Set.of(9, 10, 25, 26);
    private static final double TX_TOLERANS_DEG = 2.5;
    // Yonu ters cevirildi ve hareket yumusatildi (daha yavas donus).
    private static final double DONUS_KP = 0.008;
    private static final double MAX_DONUS = 0.08;
    // Kararli hizalama icin arka arkaya kac dongu tolerans icinde kalmali.
    private static final int KARARLI_SAYAC_ESIGI = 5;

    private final SurusAltSistemi surusAltSistemi;
    private final GorusAltSistemi gorusAltSistemi;
    private int kararliSayac;

    public LimelightMerkezlemeKomutu(SurusAltSistemi surusAltSistemi, GorusAltSistemi gorusAltSistemi) {
        this.surusAltSistemi = surusAltSistemi;
        this.gorusAltSistemi = gorusAltSistemi;
        addRequirements(surusAltSistemi);
    }

    @Override
    public void initialize() {
        kararliSayac = 0;
    }

    @Override
    public void execute() {
        if (!gorusAltSistemi.hasTarget()) {
            surusAltSistemi.drive(0.0, 0.0, 0.0);
            kararliSayac = 0;
            SmartDashboard.putString("Vision/MerkezlemeDurum", "HEDEF_YOK");
            return;
        }

        int tagId = gorusAltSistemi.getTagId();
        if (!MERKEZ_TAGLER.contains(tagId)) {
            surusAltSistemi.drive(0.0, 0.0, 0.0);
            kararliSayac = 0;
            SmartDashboard.putString("Vision/MerkezlemeDurum", "GECERSIZ_TAG_" + tagId);
            return;
        }

        int gorunenTagSayisi = gorusAltSistemi.getGorunenTagSayisi(MERKEZ_TAGLER);
        double tx = gorusAltSistemi.getOrtalamaHorizontalOffset(MERKEZ_TAGLER);
        double donusKomutu = 0.0;
        if (Math.abs(tx) > TX_TOLERANS_DEG) {
            donusKomutu = MathUtil.clamp(tx * DONUS_KP, -MAX_DONUS, MAX_DONUS);
            kararliSayac = 0;
        } else {
            kararliSayac++;
        }

        surusAltSistemi.drive(0.0, 0.0, donusKomutu);
        SmartDashboard.putString("Vision/MerkezlemeDurum", isFinished()
            ? "HIZALANDI"
            : (Math.abs(tx) <= TX_TOLERANS_DEG ? "YAKLASIK" : "HIZALANIYOR"));
        SmartDashboard.putNumber("Vision/MerkezlemeTagID", tagId);
        SmartDashboard.putNumber("Vision/MerkezlemeTx", tx);
        SmartDashboard.putNumber("Vision/MerkezlemeDonus", donusKomutu);
        SmartDashboard.putNumber("Vision/KararliSayac", kararliSayac);
        SmartDashboard.putNumber("Vision/GorunenTagSayisi", gorunenTagSayisi);
    }

    @Override
    public void end(boolean interrupted) {
        surusAltSistemi.drive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return kararliSayac >= KARARLI_SAYAC_ESIGI;
    }
}
