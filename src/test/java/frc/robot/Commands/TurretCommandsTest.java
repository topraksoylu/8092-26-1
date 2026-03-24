package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.GorusAltSistemi;

class TurretCommandsTest {
    @Test
    @Tag("fast")
    void turretTrackRotatesWhenTargetVisible() {
        TurretSubsystem turret = new TurretSubsystem();
        GorusAltSistemi vision = mock(GorusAltSistemi.class);
        when(vision.hasTarget()).thenReturn(true);
        when(vision.getHorizontalOffset()).thenReturn(10.0);

        TaretTakipKomutu command = new TaretTakipKomutu(turret, vision);
        command.execute();

        assertEquals(0.1, turret.getLastCommandedSpeed(), 1e-9);
    }

    @Test
    @Tag("fast")
    void alignToTargetStopsWhenEnded() {
        TurretSubsystem turret = new TurretSubsystem();
        GorusAltSistemi vision = mock(GorusAltSistemi.class);
        SurusAltSistemi drive = mock(SurusAltSistemi.class);

        HedefeHizalamaKomutu command = new HedefeHizalamaKomutu(drive, turret, vision);
        command.end(false);

        assertEquals(0.0, turret.getLastCommandedSpeed(), 1e-9);
    }
}
