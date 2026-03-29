package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashMap;
import java.util.Map;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

class GorusAltSistemiTest {
    private static class FakeVisionIO implements GorusAltSistemi.VisionIO {
        private final Map<String, Double> doubles = new HashMap<>();
        private final Map<String, Long> integers = new HashMap<>();
        private final Map<String, String> strings = new HashMap<>();
        private final Map<String, double[]> arrays = new HashMap<>();

        @Override
        public double getDouble(String key, double defaultValue) {
            return doubles.getOrDefault(key, defaultValue);
        }

        @Override
        public long getInteger(String key, long defaultValue) {
            return integers.getOrDefault(key, defaultValue);
        }

        @Override
        public String getString(String key, String defaultValue) {
            return strings.getOrDefault(key, defaultValue);
        }

        @Override
        public double[] getDoubleArray(String key, double[] defaultValue) {
            return arrays.getOrDefault(key, defaultValue);
        }

        @Override
        public void setNumber(String key, double value) {
            doubles.put(key, value);
        }

        @Override
        public void setDoubleArray(String key, double[] value) {
            arrays.put(key, value);
        }
    }

    private static class FakeRuntimeIO implements GorusAltSistemi.RuntimeIO {
        private boolean isReal = true;
        private boolean isRedAlliance = false;
        private double nowSec = 100.0;

        @Override
        public boolean isReal() {
            return isReal;
        }

        @Override
        public double nowSec() {
            return nowSec;
        }

        @Override
        public boolean isRedAlliance() {
            return isRedAlliance;
        }
    }

    @Test
    @Tag("fast")
    void limelightConfigStatusRequiresPipelineModeAndFmap() {
        FakeVisionIO io = new FakeVisionIO();
        FakeRuntimeIO runtime = new FakeRuntimeIO();
        GorusAltSistemi vision = new GorusAltSistemi(io, runtime);

        io.doubles.put("pipeline", 0.0);
        io.doubles.put("camMode", 0.0);
        io.doubles.put("ledMode", 0.0);
        io.integers.put("fmap/tagCount", 0L);

        assertFalse(vision.isLimelightConfigOk());
        assertTrue(vision.getLimelightConfigStatus().contains("FMAP"));

        io.integers.put("fmap/tagCount", 32L);
        assertTrue(vision.isLimelightConfigOk());
        assertEquals("OK", vision.getLimelightConfigStatus());
    }

    @Test
    @Tag("extended")
    void periodicCachesPoseAtConfiguredUpdateRate() {
        FakeVisionIO io = new FakeVisionIO();
        FakeRuntimeIO runtime = new FakeRuntimeIO();
        GorusAltSistemi vision = new GorusAltSistemi(io, runtime);

        io.doubles.put("tv", 1.0);
        io.integers.put("tid", 4L);
        io.arrays.put("botpose_orb_wpiblue", new double[] {2.0, 3.0, 0, 0, 0, 90.0, 20.0});
        io.arrays.put("targetpose_cameraspace", new double[] {0, 0, 0, 0, 0, 0, 0.05});
        io.integers.put("fmap/tagCount", 32L);

        for (int i = 0; i < 4; i++) {
            vision.periodic();
            assertFalse(vision.aprilTagdanRobotPozuAl().valid);
        }

        vision.periodic();
        GorusAltSistemi.VisionResult result = vision.aprilTagdanRobotPozuAl();
        assertTrue(result.valid);
        assertEquals(2.0, result.robotPose.getX(), 1e-9);
        assertEquals(3.0, result.robotPose.getY(), 1e-9);
        assertEquals(4, result.tagId);
        assertEquals(99.98, result.timestamp, 1e-6);
    }

    @Test
    @Tag("extended")
    void allianceSelectionSwitchesPoseKey() {
        FakeVisionIO io = new FakeVisionIO();
        FakeRuntimeIO runtime = new FakeRuntimeIO();
        GorusAltSistemi vision = new GorusAltSistemi(io, runtime);

        io.doubles.put("tv", 1.0);
        io.integers.put("tid", 5L);
        io.arrays.put("botpose_orb_wpiblue", new double[] {1.0, 2.0, 0, 0, 0, 0.0, 0.0});
        io.arrays.put("botpose_orb_wpired", new double[] {8.0, 9.0, 0, 0, 0, 180.0, 0.0});
        io.arrays.put("targetpose_cameraspace", new double[] {0, 0, 0, 0, 0, 0, 0.05});
        io.integers.put("fmap/tagCount", 32L);

        for (int i = 0; i < 5; i++) {
            vision.periodic();
        }
        assertEquals(1.0, vision.aprilTagdanRobotPozuAl().robotPose.getX(), 1e-9);

        runtime.isRedAlliance = true;
        for (int i = 0; i < 5; i++) {
            vision.periodic();
        }
        assertEquals(8.0, vision.aprilTagdanRobotPozuAl().robotPose.getX(), 1e-9);
    }
}
