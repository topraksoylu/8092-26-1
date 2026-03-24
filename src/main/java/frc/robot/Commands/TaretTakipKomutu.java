package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.GorusAltSistemi;

public class TaretTakipKomutu extends Command {
    private TurretSubsystem turretSubsystem;
    private GorusAltSistemi GorusAltSistemi;

    public TaretTakipKomutu(TurretSubsystem turretSubsystem, GorusAltSistemi GorusAltSistemi) {
        this.turretSubsystem = turretSubsystem;
        this.GorusAltSistemi = GorusAltSistemi;
        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        if (GorusAltSistemi.hasTarget()) {
            double horizontalOffset = GorusAltSistemi.getHorizontalOffset();
            double speed = horizontalOffset * 0.01; // Proportional control, tune this
            turretSubsystem.rotate(speed);
        } else {
            turretSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Continuous tracking
    }
}