package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import frc.robot.Sabitler.ModulSabitleri;
import frc.robot.Subsystems.AlimAltSistemi;
import frc.robot.Subsystems.AticiAltSistemi;

class AlimVeAtisKomutuTesti {
    @Test
    @Tag("fast")
    void alimKomutuMotorCiktisiniAyarlarVeDurdurur() {
        AlimAltSistemi altSistem = new AlimAltSistemi();
        AlimKomutu komut = new AlimKomutu(altSistem);

        komut.execute();
        assertEquals(ModulSabitleri.ALIM_HIZI, altSistem.getSonAlimHizi(), 1e-9);

        komut.end(false);
        assertEquals(0.0, altSistem.getSonAlimHizi(), 1e-9);
    }

    @Test
    @Tag("fast")
    void atisKomutuAticiVeTasiyiciyiBaslatirVeDurdurur() {
        AticiAltSistemi aticiAltSistemi = new AticiAltSistemi();
        AlimAltSistemi alimAltSistemi = new AlimAltSistemi();
        AtisKomutu komut = new AtisKomutu(aticiAltSistemi, alimAltSistemi, 3.0);

        komut.initialize();
        assertEquals(ModulSabitleri.ATICI_HIZI, aticiAltSistemi.getSonKomutHizi(), 1e-9);
        assertEquals(ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI, alimAltSistemi.getSonTasiyiciHizi(), 1e-9);

        komut.end(false);
        assertEquals(0.0, aticiAltSistemi.getSonKomutHizi(), 1e-9);
        assertEquals(0.0, alimAltSistemi.getSonTasiyiciHizi(), 1e-9);
    }
}
