package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.GorusAltSistemi;

public class OtonomAtisKomutu extends SequentialCommandGroup {
    public OtonomAtisKomutu(SurusAltSistemi SurusAltSistemi, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,
                                  TurretSubsystem turretSubsystem, GorusAltSistemi GorusAltSistemi) {
        addCommands(
            // Align turret to target
            new HedefeHizalamaKomutu(SurusAltSistemi, turretSubsystem, GorusAltSistemi),
            // Shoot
            new ShootCommand(shooterSubsystem, 3.0)
        );
    }
}