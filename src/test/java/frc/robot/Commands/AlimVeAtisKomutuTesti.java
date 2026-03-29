package frc.robot.Commands;

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
import frc.robot.Subsystems.AlimAltSistemi;
import frc.robot.Subsystems.AticiAltSistemi;

@TestInstance(Lifecycle.PER_CLASS)
class AlimVeAtisKomutuTesti {
    private AlimAltSistemi alimAltSistemi;
    private AticiAltSistemi aticiAltSistemi;

    @BeforeAll
    void initAll() {
        HAL.initialize(500, 0);
        alimAltSistemi = new AlimAltSistemi();
        aticiAltSistemi = new AticiAltSistemi();
    }

    @BeforeEach
    void resetState() {
        alimAltSistemi.durdur();
        aticiAltSistemi.durdur();
    }

    @AfterAll
    void teardownAll() {
        alimAltSistemi.close();
        aticiAltSistemi.close();
    }

    @Test
    @Tag("fast")
    void alimKomutuMotorCiktisiniAyarlarVeDurdurur() {
        AlimKomutu komut = new AlimKomutu(alimAltSistemi);

        komut.execute();
        assertEquals(ModulSabitleri.ALIM_HIZI, alimAltSistemi.getSonAlimHizi(), 1e-9);

        komut.end(false);
        assertEquals(0.0, alimAltSistemi.getSonAlimHizi(), 1e-9);
    }

    @Test
    @Tag("fast")
    void atisKomutuAticiVeTasiyiciyiBaslatirVeDurdurur() {
        AtisKomutu komut = new AtisKomutu(aticiAltSistemi, alimAltSistemi, 3.0);

        komut.initialize();
        assertEquals(ModulSabitleri.ATICI_HEDEF_RPM, aticiAltSistemi.getSonHedefRPM(), 1e-9);
        assertEquals(ModulSabitleri.DEPO_ATICI_YUKARI_TASIYICI_HIZI, alimAltSistemi.getSonTasiyiciHizi(), 1e-9);

        komut.end(false);
        assertEquals(0.0, aticiAltSistemi.getSonHedefRPM(), 1e-9);
        assertEquals(0.0, alimAltSistemi.getSonTasiyiciHizi(), 1e-9);
    }
}
