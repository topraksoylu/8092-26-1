package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

class AtisHesaplayiciTest {
    @Test
    @Tag("fast")
    void belowTableUsesFirstValue() {
        assertEquals(2900.0, AtisHesaplayici.hesaplaHedefRpm(0.8), 1e-9);
    }

    @Test
    @Tag("fast")
    void aboveTableUsesLastValue() {
        assertEquals(4800.0, AtisHesaplayici.hesaplaHedefRpm(6.5), 1e-9);
    }

    @Test
    @Tag("fast")
    void interpolatesBetweenPoints() {
        assertEquals(3500.0, AtisHesaplayici.hesaplaHedefRpm(2.4), 1e-9);
    }
}
