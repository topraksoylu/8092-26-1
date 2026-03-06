package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Simulation.VisionSimulator;

/**
 * Optimized VisionSubsystem for Limelight AprilTag detection.
 *
 * NetworkTables Optimization Best Practices Applied:
 * 1. Cached NetworkTableEntry objects (no repeated getEntry() calls)
 * 2. Batched SmartDashboard updates (reduce overhead)
 * 3. Throttled vision reads (only when needed)
 * 4. Removed redundant NT calls in hot paths
 */
public class VisionSubsystem extends SubsystemBase {
    interface VisionIO {
        double getDouble(String key, double defaultValue);
        long getInteger(String key, long defaultValue);
        String getString(String key, String defaultValue);
        double[] getDoubleArray(String key, double[] defaultValue);
        void setNumber(String key, double value);

        // Fast access methods for frequently accessed values
        default double getTV() { return getDouble("tv", 0); }
        default double getTX() { return getDouble("tx", 0); }
        default double getTY() { return getDouble("ty", 0); }
        default double getTA() { return getDouble("ta", 0); }
        default double getTS() { return getDouble("ts", 0); }
        default long getTID() { return getInteger("tid", 0); }
        default double getPipeline() { return getDouble("pipeline", -1); }
        default double getCamMode() { return getDouble("camMode", -1); }
        default double getLedMode() { return getDouble("ledMode", -1); }
        default double getStream() { return getDouble("stream", -1); }
        default long getFmapTagCount() { return getInteger("fmap/tagCount", 0); }
        default String getFmapSize() { return getString("fmap/size", "Unknown"); }
    }

    interface RuntimeIO {
        boolean isReal();
        double nowSec();
        boolean isRedAlliance();
    }

    /**
     * Optimized NetworkTables implementation with cached entries.
     * Reduces network overhead by storing entry references.
     */
    static class NetworkTableVisionIO implements VisionIO {
        private final NetworkTable table;

        // Cached entries for frequently accessed values
        private NetworkTableEntry tvEntry;
        private NetworkTableEntry txEntry;
        private NetworkTableEntry tyEntry;
        private NetworkTableEntry taEntry;
        private NetworkTableEntry tsEntry;
        private NetworkTableEntry tidEntry;
        private NetworkTableEntry pipelineEntry;
        private NetworkTableEntry camModeEntry;
        private NetworkTableEntry ledModeEntry;
        private NetworkTableEntry streamEntry;
        private NetworkTableEntry botposeBlueEntry;
        private NetworkTableEntry botposeRedEntry;
        private NetworkTableEntry targetPoseEntry;
        private NetworkTableEntry fmapTagCountEntry;
        private NetworkTableEntry fmapSizeEntry;

        NetworkTableVisionIO(String tableName) {
            table = NetworkTableInstance.getDefault().getTable(tableName);
            cacheEntries();
        }

        /**
         * Cache all frequently used NetworkTable entries.
         * This prevents repeated getEntry() calls which add overhead.
         */
        private void cacheEntries() {
            tvEntry = table.getEntry("tv");
            txEntry = table.getEntry("tx");
            tyEntry = table.getEntry("ty");
            taEntry = table.getEntry("ta");
            tsEntry = table.getEntry("ts");
            tidEntry = table.getEntry("tid");
            pipelineEntry = table.getEntry("pipeline");
            camModeEntry = table.getEntry("camMode");
            ledModeEntry = table.getEntry("ledMode");
            streamEntry = table.getEntry("stream");
            botposeBlueEntry = table.getEntry("botpose_wpiblue");
            botposeRedEntry = table.getEntry("botpose_wpired");
            targetPoseEntry = table.getEntry("targetpose_cameraspace");
            fmapTagCountEntry = table.getEntry("fmap/tagCount");
            fmapSizeEntry = table.getEntry("fmap/size");
        }

        @Override
        public double getDouble(String key, double defaultValue) {
            // For uncached entries, fall back to dynamic lookup
            return table.getEntry(key).getDouble(defaultValue);
        }

        @Override
        public long getInteger(String key, long defaultValue) {
            // For uncached entries, fall back to dynamic lookup
            return table.getEntry(key).getInteger(defaultValue);
        }

        @Override
        public String getString(String key, String defaultValue) {
            // For uncached entries, fall back to dynamic lookup
            return table.getEntry(key).getString(defaultValue);
        }

        @Override
        public double[] getDoubleArray(String key, double[] defaultValue) {
            // Use cached entries when possible
            if ("botpose_wpiblue".equals(key)) {
                return botposeBlueEntry.getDoubleArray(defaultValue);
            }
            if ("botpose_wpired".equals(key)) {
                return botposeRedEntry.getDoubleArray(defaultValue);
            }
            if ("targetpose_cameraspace".equals(key)) {
                return targetPoseEntry.getDoubleArray(defaultValue);
            }
            return table.getEntry(key).getDoubleArray(defaultValue);
        }

        @Override
        public void setNumber(String key, double value) {
            table.getEntry(key).setNumber(value);
        }

        // Fast access methods using cached entries
        public double getTV() { return tvEntry.getDouble(0); }
        public double getTX() { return txEntry.getDouble(0); }
        public double getTY() { return tyEntry.getDouble(0); }
        public double getTA() { return taEntry.getDouble(0); }
        public double getTS() { return tsEntry.getDouble(0); }
        public long getTID() { return tidEntry.getInteger(0); }
        public double getPipeline() { return pipelineEntry.getDouble(-1); }
        public double getCamMode() { return camModeEntry.getDouble(-1); }
        public double getLedMode() { return ledModeEntry.getDouble(-1); }
        public double getStream() { return streamEntry.getDouble(-1); }
        public long getFmapTagCount() { return fmapTagCountEntry.getInteger(0); }
        public String getFmapSize() { return fmapSizeEntry.getString("Unknown"); }
    }

    static class WpiRuntimeIO implements RuntimeIO {
        @Override
        public boolean isReal() {
            return RobotBase.isReal();
        }

        @Override
        public double nowSec() {
            return Timer.getFPGATimestamp();
        }

        @Override
        public boolean isRedAlliance() {
            return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        }
    }

    private final VisionIO io;
    private final RuntimeIO runtime;

    private boolean cachedHasTarget = false;
    private VisionResult cachedVisionResult = new VisionResult();
    private int visionUpdateCounter = 0;
    private static final int VISION_UPDATE_RATE = 5;
    private double lastConfigApplyTimestampSec = -1.0;
    private boolean tuningInitialized = false;

    // Batching: Cache values to update SmartDashboard less frequently
    private int dashboardUpdateCounter = 0;
    private static final int DASHBOARD_UPDATE_RATE = 10; // Update every 200ms

    // NetworkTables diagnostic info
    private String ntConnectionStatus = "Checking...";
    private double lastTvValue = -999;

    // Simulation support
    private VisionSimulator visionSim;
    private VisionResult cachedSimResult;

    public static class VisionResult {
        public Pose2d robotPose;
        public int tagId;
        public double ambiguity;
        public double timestamp;
        public boolean valid;
    }

    public VisionSubsystem() {
        this(new LimelightHelpersIO(VisionConstants.LIMELIGHT_NAME), new WpiRuntimeIO());
    }

    VisionSubsystem(VisionIO io, RuntimeIO runtime) {
        this.io = io;
        this.runtime = runtime;
        initializeTuningDashboard();
        applyDesiredLimelightConfig();

        // Initialize vision simulator for simulation mode
        if (!runtime.isReal()) {
            visionSim = new VisionSimulator();
            cachedSimResult = new VisionResult();
        }
    }

    public boolean hasTarget() {
        return cachedHasTarget;
    }

    public double getHorizontalOffset() {
        return io.getTX();
    }

    public double getVerticalOffset() {
        return io.getTY();
    }

    public double getTargetArea() {
        return io.getTA();
    }

    public double getTargetSkew() {
        return io.getTS();
    }

    public double getDistanceToTarget() {
        double targetOffsetAngleVertical = getVerticalOffset();
        double angleToGoalDegrees = VisionConstants.LIMELIGHT_ANGLE + targetOffsetAngleVertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        double distanceInches =
            (VisionConstants.TARGET_HEIGHT - VisionConstants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);
        return distanceInches * 0.0254;
    }

    public int getTagId() {
        if (!hasTarget()) {
            return 0;
        }
        return (int) io.getTID();
    }

    public int getFmapTagCount() {
        return (int) io.getFmapTagCount();
    }

    public String getFmapStatus() {
        int tagCount = getFmapTagCount();
        String fmapSize = io.getFmapSize();
        return String.format("FMAP: %d tags, Size: %s", tagCount, fmapSize);
    }

    public int getCurrentPipeline() {
        return (int) io.getPipeline();
    }

    public int getCurrentCamMode() {
        return (int) io.getCamMode();
    }

    public int getCurrentLedMode() {
        return (int) io.getLedMode();
    }

    public int getCurrentStreamMode() {
        return (int) io.getStream();
    }

    private void initializeTuningDashboard() {
        SmartDashboard.putBoolean("Vision/Tune/Enable", false);
        SmartDashboard.putNumber("Vision/Tune/Pipeline", VisionConstants.DESIRED_PIPELINE);
        SmartDashboard.putNumber("Vision/Tune/SourceImageCamMode", VisionConstants.DESIRED_CAM_MODE);
        SmartDashboard.putNumber("Vision/Tune/LEDMode", VisionConstants.DESIRED_LED_MODE);
        SmartDashboard.putNumber("Vision/Tune/StreamMode", VisionConstants.DESIRED_STREAM_MODE);
        SmartDashboard.putString("Vision/Tune/Unsupported", "Resolution, Stream Orientation, Exposure, LED Power -> Limelight UI");
        tuningInitialized = true;
    }

    private int getDesiredPipeline() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/Pipeline", VisionConstants.DESIRED_PIPELINE);
        }
        return VisionConstants.DESIRED_PIPELINE;
    }

    private int getDesiredCamMode() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/SourceImageCamMode", VisionConstants.DESIRED_CAM_MODE);
        }
        return VisionConstants.DESIRED_CAM_MODE;
    }

    private int getDesiredLedMode() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/LEDMode", VisionConstants.DESIRED_LED_MODE);
        }
        return VisionConstants.DESIRED_LED_MODE;
    }

    private int getDesiredStreamMode() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/StreamMode", VisionConstants.DESIRED_STREAM_MODE);
        }
        return VisionConstants.DESIRED_STREAM_MODE;
    }

    public void applyDesiredLimelightConfig() {
        io.setNumber("pipeline", getDesiredPipeline());
        io.setNumber("camMode", getDesiredCamMode());
        io.setNumber("ledMode", getDesiredLedMode());
        io.setNumber("stream", getDesiredStreamMode());
        lastConfigApplyTimestampSec = runtime.nowSec();
    }

    public boolean isLimelightConfigOk() {
        boolean pipelineOk = getCurrentPipeline() == getDesiredPipeline();
        boolean camModeOk = getCurrentCamMode() == getDesiredCamMode();
        boolean ledModeOk = getCurrentLedMode() == getDesiredLedMode();
        boolean streamOk = getCurrentStreamMode() == getDesiredStreamMode();
        boolean fmapLoaded = getFmapTagCount() > 0;
        return pipelineOk && camModeOk && ledModeOk && streamOk && fmapLoaded;
    }

    public String getLimelightConfigStatus() {
        int desiredPipeline = getDesiredPipeline();
        int desiredCamMode = getDesiredCamMode();
        int desiredLedMode = getDesiredLedMode();
        int desiredStreamMode = getDesiredStreamMode();

        boolean pipelineOk = getCurrentPipeline() == desiredPipeline;
        boolean camModeOk = getCurrentCamMode() == desiredCamMode;
        boolean ledModeOk = getCurrentLedMode() == desiredLedMode;
        boolean streamOk = getCurrentStreamMode() == desiredStreamMode;
        int fmapTagCount = getFmapTagCount();

        if (!pipelineOk) {
            return String.format("Pipeline mismatch (%d != %d)", getCurrentPipeline(), desiredPipeline);
        }
        if (!camModeOk) {
            return String.format("CamMode mismatch (%d != %d)", getCurrentCamMode(), desiredCamMode);
        }
        if (!ledModeOk) {
            return String.format("LedMode mismatch (%d != %d)", getCurrentLedMode(), desiredLedMode);
        }
        if (!streamOk) {
            return String.format("Stream mismatch (%d != %d)", getCurrentStreamMode(), desiredStreamMode);
        }
        if (fmapTagCount <= 0) {
            return "FMAP not loaded (tagCount=0)";
        }
        if (fmapTagCount < VisionConstants.TOTAL_APRILTAGS) {
            return String.format("FMAP partial (%d/%d tags)", fmapTagCount, VisionConstants.TOTAL_APRILTAGS);
        }
        return "OK";
    }

    /**
     * Simülasyon modunda robot pozisyonunu günceller.
     * DriveSubsystem tarafından her döngüde çağrılır.
     *
     * @param pose Robotun mevcut pozisyonu
     */
    public void setRobotPoseForSimulation(Pose2d pose) {
        if (visionSim != null) {
            visionSim.setRobotPose(pose);
        }
    }

    private void updateCachedVisionData() {
        // Use cached entry for fast read
        lastTvValue = io.getTV();
        cachedHasTarget = lastTvValue == 1;
        visionUpdateCounter++;
        if (visionUpdateCounter >= VISION_UPDATE_RATE) {
            visionUpdateCounter = 0;
            if (cachedHasTarget) {
                cachedVisionResult = getRobotPoseFromAprilTagInternal();
            } else {
                cachedVisionResult.valid = false;
            }
        }
    }

    private VisionResult getRobotPoseFromAprilTagInternal() {
        VisionResult result = new VisionResult();
        result.valid = false;

        // Use cached entry for fast read
        double tv = lastTvValue;

        // Determine connection status
        if (tv == -999) {
            ntConnectionStatus = "NOT CONNECTED";
        } else if (tv == 0) {
            ntConnectionStatus = "Connected - No Tag";
        } else {
            ntConnectionStatus = "Connected - Tag Detected";
        }

        // Debug: Less frequent SmartDashboard updates
        if (visionUpdateCounter == 0) {
            SmartDashboard.putNumber("Vision/Debug/RawTV_Value", tv);
            SmartDashboard.putBoolean("Vision/Debug/RawTV", tv == 1);
            SmartDashboard.putString("Vision/Debug/NT_Status", ntConnectionStatus);
        }

        // Check if NetworkTables is connected at all
        boolean ntConnected = (tv != -999);

        if (visionUpdateCounter == 0) {
            SmartDashboard.putBoolean("Vision/Debug/NT_Connected", ntConnected);
        }

        if (!ntConnected) {
            if (visionUpdateCounter == 0) {
                SmartDashboard.putString("Vision/Debug/Status", "NT NOT CONNECTED");
            }
            // Only print error occasionally to reduce spam
            if (dashboardUpdateCounter == 0) {
                System.out.println("============================================");
                System.out.println("VISION: NetworkTables NOT CONNECTED");
                System.out.println("Limelight IP: 10.80.92.200:5801");
                System.out.println("Troubleshooting:");
                System.out.println("  1) Check Limelight has power (LEDs on?)");
                System.out.println("  2) Check ethernet cable to radio");
                System.out.println("  3) Open http://10.80.92.200:5801 in browser");
                System.out.println("  4) Verify team number is 8092 in Limelight settings");
                System.out.println("============================================");
            }
            return result;
        }

        // BUG FIX: Check raw tv value, not cached value!
        if (tv != 1) {
            if (visionUpdateCounter == 0) {
                SmartDashboard.putString("Vision/Debug/Status", "No target (tv=" + tv + ")");
            }
            return result;
        }

        // Log target detection - DISABLED to prevent console flooding
        // if (visionUpdateCounter == 0 || dashboardUpdateCounter == 0) {
        //     System.out.println("OK VISION: Target DETECTED (tv=1) - Reading pose data...");
        // }

        String poseEntry = runtime.isRedAlliance() ? "botpose_wpired" : "botpose_wpiblue";

        if (visionUpdateCounter == 0) {
            SmartDashboard.putString("Vision/Debug/PoseEntry", poseEntry);
        }

        double[] botPoseArray = io.getDoubleArray(poseEntry, new double[0]);

        if (visionUpdateCounter == 0) {
            SmartDashboard.putNumber("Vision/Debug/PoseArrayLength", botPoseArray.length);
        }

        if (botPoseArray.length < 7) {
            if (visionUpdateCounter == 0) {
                SmartDashboard.putString("Vision/Debug/Status", "Array too short: " + botPoseArray.length);
            }
            return result;
        }

        double x = botPoseArray[0];
        double y = botPoseArray[1];
        double yaw = botPoseArray[5];
        double latency = botPoseArray[6] / 1000.0;
        double timestamp = runtime.nowSec() - latency;

        result.robotPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
        result.timestamp = timestamp;
        result.tagId = (int) io.getTID();

        // Set ambiguity to 0.0 (valid) - we don't validate it anymore
        result.ambiguity = 0.0;

        // Validate result: only check if we have a valid tag ID
        // If Limelight says tv=1 (target detected), we trust it
        result.valid = (result.tagId > 0);

        // Logging - DISABLED to prevent console flooding
        // if (visionUpdateCounter == 0 || dashboardUpdateCounter == 0) {
        //     if (result.valid) {
        //         System.out.println("OK VISION: TagID=" + result.tagId + " Pose=(" + String.format("%.2f", x) + ", " + String.format("%.2f", y) + ")");
        //     } else {
        //         System.out.println("WARN VISION: No valid tag (TagID=" + result.tagId + ")");
        //     }
        // }

        // Batched debug updates (only every 5 cycles)
        if (visionUpdateCounter == 0) {
            SmartDashboard.putNumber("Vision/Debug/TagID", result.tagId);
            SmartDashboard.putNumber("Vision/Debug/RobotX", x);
            SmartDashboard.putNumber("Vision/Debug/RobotY", y);
            SmartDashboard.putBoolean("Vision/Debug/Valid", result.valid);
            SmartDashboard.putString("Vision/Debug/Status", result.valid ? "OK" : "No Tag");
        }

        return result;
    }

    /**
     * Batched SmartDashboard update helper.
     * Reduces NetworkTables overhead by grouping updates.
     */
    private void updateDashboard() {
        dashboardUpdateCounter++;
        if (dashboardUpdateCounter < DASHBOARD_UPDATE_RATE) {
            return;
        }
        dashboardUpdateCounter = 0;

        // Update all vision data at once
        SmartDashboard.putBoolean("Vision/HasTarget", cachedHasTarget);
        SmartDashboard.putString("Vision/NT_Status", ntConnectionStatus);
        SmartDashboard.putNumber("Vision/TV_Value", lastTvValue);

        if (cachedHasTarget) {
            SmartDashboard.putNumber("Vision/HorizontalOffset", io.getTX());
            SmartDashboard.putNumber("Vision/VerticalOffset", io.getTY());
            SmartDashboard.putNumber("Vision/TargetArea", io.getTA());
            SmartDashboard.putNumber("Vision/DistanceToTarget", getDistanceToTarget());
        }

        SmartDashboard.putNumber("Vision/FmapTagCount", getFmapTagCount());
        SmartDashboard.putString("Vision/FmapStatus", getFmapStatus());
        SmartDashboard.putNumber("Vision/Pipeline", getCurrentPipeline());
        SmartDashboard.putNumber("Vision/CamMode", getCurrentCamMode());
        SmartDashboard.putNumber("Vision/LEDMode", getCurrentLedMode());
        SmartDashboard.putNumber("Vision/StreamMode", getCurrentStreamMode());
        SmartDashboard.putBoolean("Vision/ConfigOk", isLimelightConfigOk());
        SmartDashboard.putString("Vision/ConfigStatus", getLimelightConfigStatus());

        SmartDashboard.putBoolean("Vision/PoseValid", cachedVisionResult.valid);
        if (cachedVisionResult.valid) {
            SmartDashboard.putNumber("Vision/RobotX", cachedVisionResult.robotPose.getX());
            SmartDashboard.putNumber("Vision/RobotY", cachedVisionResult.robotPose.getY());
            SmartDashboard.putNumber("Vision/RobotYaw", cachedVisionResult.robotPose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/TagID", cachedVisionResult.tagId);
            SmartDashboard.putNumber("Vision/Ambiguity", cachedVisionResult.ambiguity);
        }
    }

    @Override
    public void periodic() {
        if (runtime.isReal()) {
            if (!tuningInitialized) {
                initializeTuningDashboard();
            }

            double nowSec = runtime.nowSec();
            if (lastConfigApplyTimestampSec < 0
                || nowSec - lastConfigApplyTimestampSec >= VisionConstants.CONFIG_REAPPLY_INTERVAL_SEC) {
                applyDesiredLimelightConfig();
            }

            updateCachedVisionData();
            updateDashboard(); // Batched SmartDashboard updates

        } else {
            // Simulation mode - use VisionSimulator
            if (visionSim != null) {
                VisionSimulator.VisionResult simResult = visionSim.getVisionResult();
                cachedSimResult = new VisionSubsystem.VisionResult();
                cachedSimResult.robotPose = simResult.robotPose;
                cachedSimResult.tagId = simResult.tagId;
                cachedSimResult.valid = simResult.valid;
                cachedSimResult.ambiguity = simResult.ambiguity;
                cachedSimResult.timestamp = runtime.nowSec();
                cachedHasTarget = cachedSimResult.valid;
            }

            // Update SmartDashboard with simulated data
            SmartDashboard.putBoolean("Vision/HasTarget", cachedHasTarget);
            if (cachedHasTarget && visionSim != null) {
                SmartDashboard.putNumber("Vision/HorizontalOffset", visionSim.getHorizontalOffset());
                SmartDashboard.putNumber("Vision/DistanceToTarget", visionSim.getDistanceToTarget());
                SmartDashboard.putNumber("Vision/TagID", visionSim.getTagId());
                SmartDashboard.putBoolean("Vision/PoseValid", cachedSimResult.valid);
                SmartDashboard.putNumber("Vision/RobotX", cachedSimResult.robotPose.getX());
                SmartDashboard.putNumber("Vision/RobotY", cachedSimResult.robotPose.getY());
                SmartDashboard.putNumber("Vision/RobotYaw", cachedSimResult.robotPose.getRotation().getDegrees());
                SmartDashboard.putNumber("Vision/Ambiguity", cachedSimResult.ambiguity);
            }
            SmartDashboard.putString("Vision/Mode", "Simulation");
        }
    }

    public VisionResult getRobotPoseFromAprilTag() {
        if (!runtime.isReal() && visionSim != null) {
            // Simülasyon modunda - VisionSimulator'dan veri döndür
            return cachedSimResult;
        }
        // Gerçek mod - NetworkTables'tan veri oku
        return getRobotPoseFromAprilTagInternal();
    }
}
