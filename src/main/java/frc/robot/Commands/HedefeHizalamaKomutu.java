package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.GorusAltSistemi;

public class HedefeHizalamaKomutu extends Command {
    private SurusAltSistemi SurusAltSistemi;
    private TurretSubsystem turretSubsystem;
    private GorusAltSistemi GorusAltSistemi;
    private double tolerance = 2.0; // degrees

    public HedefeHizalamaKomutu(SurusAltSistemi SurusAltSistemi, TurretSubsystem turretSubsystem, GorusAltSistemi GorusAltSistemi) {
        this.SurusAltSistemi = SurusAltSistemi;
        this.turretSubsystem = turretSubsystem;
        this.GorusAltSistemi = GorusAltSistemi;
        addRequirements(SurusAltSistemi, turretSubsystem);
    }

    @Override
    public void execute() {
        if (GorusAltSistemi.hasTarget()) {
            double horizontalOffset = GorusAltSistemi.getHorizontalOffset();
            double verticalOffset = GorusAltSistemi.getVerticalOffset();

            // Rotate turret
            double turretSpeed = horizontalOffset * 0.02; // Tune
            turretSubsystem.rotate(turretSpeed);

            // Optionally rotate robot if needed
            // double driveRotation = horizontalOffset * 0.01;
            // SurusAltSistemi.drive(0, 0, driveRotation);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stop();
        SurusAltSistemi.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return GorusAltSistemi.hasTarget() && Math.abs(GorusAltSistemi.getHorizontalOffset()) < tolerance;
    }
}