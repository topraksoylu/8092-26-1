package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;

import edu.wpi.first.hal.HAL;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.TaretAltSistemi;
import frc.robot.Subsystems.GorusAltSistemi;

@TestInstance(Lifecycle.PER_CLASS)
class TaretKomutlariTesti {
    private TaretAltSistemi taret;

    @BeforeAll
    void initAll() {
        HAL.initialize(500, 0);
        taret = new TaretAltSistemi();
    }

    @BeforeEach
    void resetState() {
        taret.durdur();
    }

    @AfterAll
    void teardownAll() {
        taret.close();
    }

    @Test
    @Tag("fast")
    void taretTakipHedefGorunurkenDondurur() {
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
        GorusAltSistemi gorus = mock(GorusAltSistemi.class);
        SurusAltSistemi surus = mock(SurusAltSistemi.class);

        HedefeHizalamaKomutu komut = new HedefeHizalamaKomutu(surus, taret, gorus);
        komut.end(false);

        assertEquals(0.0, taret.getSonKomutHizi(), 1e-9);
    }
}
