package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SurusAltSistemi;

/**
 * Aktif oldugu surece tek bir surus motorunu sabit hizda calistirir.
 * Motor kablo baglantisini ve yonunu guvenli sekilde test etmek icin
 * SmartDashboard uzerinden kullanilir.
 */
public class MotorTestKomutu extends Command {
    public static final int ON_SOL = 1;
    public static final int ON_SAG = 2;
    public static final int ARKA_SOL = 3;
    public static final int ARKA_SAG = 4;

    private final SurusAltSistemi surus;
    private final int motorIndeksi;
    private final double hiz;

    public MotorTestKomutu(SurusAltSistemi surus, int motorIndeksi, double hiz) {
        this.surus = surus;
        this.motorIndeksi = motorIndeksi;
        this.hiz = hiz;
        addRequirements(surus);
    }

    @Override
    public void initialize() {
        // bos
    }

    @Override
    public void execute() {
        switch (motorIndeksi) {
            case ON_SOL:
                surus.testAyarlaOnSol(hiz);
                break;
            case ON_SAG:
                surus.testAyarlaOnSag(hiz);
                break;
            case ARKA_SOL:
                surus.testAyarlaArkaSol(hiz);
                break;
            case ARKA_SAG:
                surus.testAyarlaArkaSag(hiz);
                break;
            default:
                // bilinmeyen indeks
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        surus.tumMotorlariDurdur();
    }

    @Override
    public boolean isFinished() {
        return false; // aktif oldugu surece calistir
    }
}
