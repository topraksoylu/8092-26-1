package frc.robot.Subsystems;

import frc.robot.Limelight.LimelightHelpers;
import frc.robot.Limelight.LimelightHelpers.LimelightResults;
import frc.robot.Limelight.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.Limelight.LimelightHelpers.PoseEstimate;

/**
 * VisionIO implementation using the official LimelightHelpers library.
 * This provides a clean, tested interface to Limelight NetworkTables data.
 *
 * This class delegates all NetworkTables access to LimelightHelpers, which is:
 * - Officially maintained by Limelight
 * - Optimized for performance with cached DoubleArrayEntry
 * - Regularly updated with new features
 */
public class LimelightHelpersIO implements VisionSubsystem.VisionIO {
    private final String limelightName;

    /**
     * Create a new LimelightHelpersIO for a specific Limelight.
     * @param limelightName The NetworkTables name of the Limelight (e.g., "limelight")
     */
    public LimelightHelpersIO(String limelightName) {
        this.limelightName = limelightName;
    }

    @Override
    public double getDouble(String key, double defaultValue) {
        return LimelightHelpers.getLimelightNTDouble(limelightName, key);
    }

    @Override
    public long getInteger(String key, long defaultValue) {
        double val = LimelightHelpers.getLimelightNTDouble(limelightName, key);
        return (long) val;
    }

    @Override
    public String getString(String key, String defaultValue) {
        String val = LimelightHelpers.getLimelightNTString(limelightName, key);
        return (val != null && !val.isEmpty()) ? val : defaultValue;
    }

    @Override
    public double[] getDoubleArray(String key, double[] defaultValue) {
        double[] result = LimelightHelpers.getLimelightNTDoubleArray(limelightName, key);
        return (result != null && result.length > 0) ? result : defaultValue;
    }

    @Override
    public void setNumber(String key, double value) {
        LimelightHelpers.setLimelightNTDouble(limelightName, key, value);
    }

    @Override
    public double getTV() {
        // getTV() returns boolean, convert to double (0.0 or 1.0)
        return LimelightHelpers.getTV(limelightName) ? 1.0 : 0.0;
    }

    @Override
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    @Override
    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    @Override
    public double getTA() {
        return LimelightHelpers.getTA(limelightName);
    }

    @Override
    public double getTS() {
        // ts (target skew) - use generic NT method
        return LimelightHelpers.getLimelightNTDouble(limelightName, "ts");
    }

    @Override
    public long getTID() {
        // getFiducialID() returns double, cast to long
        return (long) LimelightHelpers.getFiducialID(limelightName);
    }

    @Override
    public double getPipeline() {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName);
    }

    @Override
    public double getCamMode() {
        // camMode - use generic NT method
        return LimelightHelpers.getLimelightNTDouble(limelightName, "camMode");
    }

    @Override
    public double getLedMode() {
        // ledMode - use generic NT method
        return LimelightHelpers.getLimelightNTDouble(limelightName, "ledMode");
    }

    @Override
    public double getStream() {
        // stream - use generic NT method
        return LimelightHelpers.getLimelightNTDouble(limelightName, "stream");
    }

    @Override
    public long getFmapTagCount() {
        // fmap/tagCount - use generic NT method, convert to long
        return (long) LimelightHelpers.getLimelightNTDouble(limelightName, "fmap/tagCount");
    }

    @Override
    public String getFmapSize() {
        // fmap/size - use generic NT method
        String val = LimelightHelpers.getLimelightNTString(limelightName, "fmap/size");
        return (val != null && !val.isEmpty()) ? val : "Unknown";
    }

    /**
     * Get the latest Limelight results using the official API.
     * @return LimelightResults containing all target and pose data
     */
    public LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults(limelightName);
    }

    /**
     * Get the latest botpose estimate using the official API.
     * @param useMegaTag Whether to use MegaTag2 estimation
     * @return PoseEstimate with botpose and ambiguity data
     */
    public PoseEstimate getBotPoseEstimate(boolean useMegaTag) {
        if (useMegaTag) {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        } else {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }
    }

    /**
     * Check if Limelight is connected and producing valid data.
     * @return true if connected and producing data
     */
    public boolean isConnected() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Get the latency from the Limelight (pipeline + capture).
     * @return Total latency in seconds
     */
    public double getLatency() {
        return LimelightHelpers.getLatency_Capture(limelightName) / 1000.0
               + LimelightHelpers.getLatency_Pipeline(limelightName) / 1000.0;
    }
}
