package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

class SurusKomutuTest {
    @Test
    @Tag("fast")
    void shapeAxisAppliesDeadbandAndSquareCurve() {
        assertEquals(0.0, SurusKomutu.eksenSekillendir(0.01, 0.5), 1e-9);

        double shaped = SurusKomutu.eksenSekillendir(0.5, 0.5);
        assertTrue(shaped > 0.0);
        assertEquals(Math.pow(0.5 - 0.08, 2) / Math.pow(1.0 - 0.08, 2) * 0.5, shaped, 1e-6);

        double negative = SurusKomutu.eksenSekillendir(-0.5, 0.5);
        assertEquals(-shaped, negative, 1e-6);
    }

    @Test
    @Tag("fast")
    void executeSendsShapedCommandsToDriveOutput() {
        AtomicReference<double[]> outputs = new AtomicReference<>(new double[] {0, 0, 0});
        SubsystemBase req = new SubsystemBase() {};
        SurusKomutu command = new SurusKomutu(
            () -> 0.7,
            () -> 0.4,
            () -> -0.2,
            (y, x, z) -> outputs.set(new double[] {y, x, z}),
            req
        );

        command.execute();
        double[] out = outputs.get();

        assertTrue(out[0] < 0.0);
        assertTrue(out[1] > 0.0);
        assertTrue(out[2] < 0.0);
    }
}
