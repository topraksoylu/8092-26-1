package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

class AtisHesaplayiciTest {
    @Test
    @Tag("fast")
    void belowTableUsesFirstValue() {
        assertEquals(3200.0, AtisHesaplayici.hesaplaHedefRpm(0.8), 1e-9);
    }

    @Test
    @Tag("fast")
    void aboveTableUsesLastValue() {
        assertEquals(4200.0, AtisHesaplayici.hesaplaHedefRpm(6.5), 1e-9);
    }

    @Test
    @Tag("fast")
    void interpolatesBetweenPoints() {
        assertEquals(3772.0, AtisHesaplayici.hesaplaHedefRpm(2.4), 1e-9);
    }

    @Test
    @Tag("fast")
    void minAtisRpmReturnsPositiveValueForCurrentRobotGeometry() {
        assertTrue(AtisHesaplayici.minAtisRpm() > 0.0);
    }

    @Test
    @Tag("fast")
    void minAtisRpmReturnsZeroWhenTargetIsNotAboveLauncher() {
        assertEquals(0.0, AtisHesaplayici.minAtisRpm(0.0, 1.0, 1.0), 1e-9);
        assertEquals(0.0, AtisHesaplayici.minAtisRpm(-0.2, 1.0, 1.0), 1e-9);
    }
}
