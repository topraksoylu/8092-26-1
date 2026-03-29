package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;

import edu.wpi.first.hal.HAL;
import frc.robot.Sabitler.ModulSabitleri;

@TestInstance(Lifecycle.PER_CLASS)
class MekanizmaAltSistemiTesti {
    private AlimAltSistemi alim;
    private AticiAltSistemi atici;
    private TaretAltSistemi taret;

    @BeforeAll
    void initAll() {
        HAL.initialize(500, 0);
        alim = new AlimAltSistemi();
        atici = new AticiAltSistemi();
        taret = new TaretAltSistemi();
    }

    @BeforeEach
    void resetState() {
        alim.durdur();
        atici.durdur();
        taret.durdur();
    }

    @AfterAll
    void teardownAll() {
        alim.close();
        atici.close();
        taret.close();
    }

    @Test
    @Tag("fast")
    void alimSonKomutTakipEder() {
        alim.al();
        assertEquals(ModulSabitleri.ALIM_HIZI, alim.getSonAlimHizi(), 1e-9);

        alim.geriAt();
        assertEquals(-ModulSabitleri.ALIM_HIZI, alim.getSonAlimHizi(), 1e-9);

        alim.depodanAticiyaYukariTasimaBaslat();
        assertEquals(ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI, alim.getSonTasiyiciHizi(), 1e-9);
        alim.depodanAticiyaYukariTasimaDurdur();
        assertEquals(0.0, alim.getSonTasiyiciHizi(), 1e-9);
    }

    @Test
    @Tag("fast")
    void aticiSonKomutTakipEder() {
        atici.at();
        assertEquals(ModulSabitleri.ATICI_HIZI, atici.getSonKomutHizi(), 1e-9);

        atici.durdur();
        assertEquals(0.0, atici.getSonKomutHizi(), 1e-9);
    }

    @Test
    @Tag("fast")
    void taretAciAyarlamaOranliKomutVerir() {
        taret.aciAyarla(100.0);
        assertEquals(1.0, taret.getSonKomutHizi(), 1e-9);
        taret.durdur();
        assertEquals(0.0, taret.getSonKomutHizi(), 1e-9);
    }
}
