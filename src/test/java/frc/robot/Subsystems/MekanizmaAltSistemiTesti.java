package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import frc.robot.Sabitler.ModulSabitleri;

class MekanizmaAltSistemiTesti {
    @Test
    @Tag("fast")
    void alimSonKomutTakipEder() {
        AlimAltSistemi alim = new AlimAltSistemi();
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
        AticiAltSistemi atici = new AticiAltSistemi();
        atici.at();
        assertEquals(ModulSabitleri.ATICI_HIZI, atici.getSonKomutHizi(), 1e-9);

        atici.durdur();
        assertEquals(0.0, atici.getSonKomutHizi(), 1e-9);
    }

    @Test
    @Tag("fast")
    void taretAciAyarlamaOranliKomutVerir() {
        TaretAltSistemi taret = new TaretAltSistemi();
        taret.aciAyarla(100.0);
        assertEquals(1.0, taret.getSonKomutHizi(), 1e-9);
        taret.durdur();
        assertEquals(0.0, taret.getSonKomutHizi(), 1e-9);
    }
}
