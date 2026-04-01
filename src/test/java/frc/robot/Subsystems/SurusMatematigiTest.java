package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import frc.robot.Sabitler.SurusSabitleri;

class SurusMatematigiTest {
    @Test
    @Tag("fast")
    void encoderConversionsMatchExpectedUnits() {
        double motorRotationsForTenWheelRotations = 10.0 * SurusSabitleri.DISLI_ORANI;
        double meters = SurusMatematigi.encoderPositionToMeters(
            motorRotationsForTenWheelRotations,
            SurusSabitleri.DISLI_ORANI,
            SurusSabitleri.TEKER_CEVRESI
        );
        assertEquals(10.0 * SurusSabitleri.TEKER_CEVRESI, meters, 1e-9);

        double mps = SurusMatematigi.encoderVelocityRpmToMetersPerSecond(
            motorRotationsForTenWheelRotations,
            SurusSabitleri.DISLI_ORANI,
            SurusSabitleri.TEKER_CEVRESI
        );
        assertEquals((10.0 * SurusSabitleri.TEKER_CEVRESI) / 60.0, mps, 1e-9);
    }

    @Test
    @Tag("fast")
    void wrapDeltaDegreesHandlesCrossingBoundary() {
        assertEquals(20.0, SurusMatematigi.wrapDeltaDegrees(170.0, -170.0), 1e-9);
        assertEquals(-20.0, SurusMatematigi.wrapDeltaDegrees(-170.0, 170.0), 1e-9);
    }
}
