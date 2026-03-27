package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.TaretAltSistemi;
import frc.robot.Subsystems.GorusAltSistemi;

class TaretKomutlariTesti {
    @Test
    @Tag("fast")
    void taretTakipHedefGorunurkenDondurur() {
        TaretAltSistemi taret = new TaretAltSistemi();
        GorusAltSistemi gorus = mock(GorusAltSistemi.class);
        when(gorus.hasTarget()).thenReturn(true);
        when(gorus.getHorizontalOffset()).thenReturn(10.0);

        TaretTakipKomutu komut = new TaretTakipKomutu(taret, gorus);
        komut.execute();

        assertEquals(0.1, taret.getSonKomutHizi(), 1e-9);
    }

    @Test
    @Tag("fast")
    void hedefeHizalamaBitinceAltSistemiDurdurur() {
        TaretAltSistemi taret = new TaretAltSistemi();
        GorusAltSistemi gorus = mock(GorusAltSistemi.class);
        SurusAltSistemi surus = mock(SurusAltSistemi.class);

        HedefeHizalamaKomutu komut = new HedefeHizalamaKomutu(surus, taret, gorus);
        komut.end(false);

        assertEquals(0.0, taret.getSonKomutHizi(), 1e-9);
    }
}
