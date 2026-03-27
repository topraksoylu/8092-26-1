package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlimAltSistemi;

public class AlimKomutu extends Command {
    private AlimAltSistemi alimAltSistemi;

    public AlimKomutu(AlimAltSistemi alimAltSistemi) {
        this.alimAltSistemi = alimAltSistemi;
        addRequirements(alimAltSistemi);
    }

    @Override
    public void execute() {
        alimAltSistemi.al();
    }

    @Override
    public void end(boolean interrupted) {
        alimAltSistemi.durdur();
    }

    @Override
    public boolean isFinished() {
        return false; // Kesilene kadar calistir
    }
}
