package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.AlimAltSistemi;
import frc.robot.Subsystems.AticiAltSistemi;
import frc.robot.Subsystems.TaretAltSistemi;
import frc.robot.Subsystems.GorusAltSistemi;

public class OtonomAtisKomutu extends SequentialCommandGroup {
    public OtonomAtisKomutu(SurusAltSistemi surusAltSistemi, AlimAltSistemi alimAltSistemi,
                             AticiAltSistemi aticiAltSistemi, TaretAltSistemi taretAltSistemi,
                             GorusAltSistemi gorusAltSistemi) {
        addCommands(
            // Tareti hedefe hizala
            new HedefeHizalamaKomutu(surusAltSistemi, taretAltSistemi, gorusAltSistemi),
            // Atis + depodan aticiya yukari tasima
            new AtisKomutu(aticiAltSistemi, alimAltSistemi, 3.0)
        );
    }
}
