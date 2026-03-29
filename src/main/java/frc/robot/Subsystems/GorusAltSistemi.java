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
import frc.robot.Sabitler.GorusSabitleri;

public class GorusAltSistemi extends SubsystemBase {
    interface VisionIO {
        double getDouble(String key, double defaultValue);
        long getInteger(String key, long defaultValue);
        String getString(String key, String defaultValue);
        double[] getDoubleArray(String key, double[] defaultValue);
        void setNumber(String key, double value);
        void setDoubleArray(String key, double[] value);
    }

    interface RuntimeIO {
        boolean isReal();
        double nowSec();
        boolean isRedAlliance();
    }

    static class NetworkTableVisionIO implements VisionIO {
        private final NetworkTable table;

        NetworkTableVisionIO(String tableName) {
            table = NetworkTableInstance.getDefault().getTable(tableName);
        }

        private NetworkTableEntry entry(String key) {
            return table.getEntry(key);
        }

        @Override
        public double getDouble(String key, double defaultValue) {
            return entry(key).getDouble(defaultValue);
        }

        @Override
        public long getInteger(String key, long defaultValue) {
            return entry(key).getInteger(defaultValue);
        }

        @Override
        public String getString(String key, String defaultValue) {
            return entry(key).getString(defaultValue);
        }

        @Override
        public double[] getDoubleArray(String key, double[] defaultValue) {
            return entry(key).getDoubleArray(defaultValue);
        }

        @Override
        public void setNumber(String key, double value) {
            entry(key).setNumber(value);
        }

        @Override
        public void setDoubleArray(String key, double[] value) {
            entry(key).setDoubleArray(value);
        }
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

    public static class VisionResult {
        public Pose2d robotPose;
        public int tagId;
        public double ambiguity;
        public double timestamp;
        public boolean valid;
        public int tagCount;      // MegaTag2: kac tag kullanildi
        public double avgTagDist; // MegaTag2: ortalama tag mesafesi (m), std dev olcekleme icin
    }

    public GorusAltSistemi() {
        this(new NetworkTableVisionIO(GorusSabitleri.LIMELIGHT_ADI), new WpiRuntimeIO());
    }

    GorusAltSistemi(VisionIO io, RuntimeIO runtime) {
        this.io = io;
        this.runtime = runtime;
        initializeTuningDashboard();
        applyDesiredLimelightConfig();
    }

    /**
     * MegaTag2 icin jiro yonunu Limelight'a bildir.
     * SurusAltSistemi.periodic() tarafindan her dongude cagrilmali.
     * @param yawDegrees Robot yonu (WPILib CCW+, derece)
     */
    public void setRobotOrientation(double yawDegrees) {
        // Limelight format: [yaw, yawRate, pitch, pitchRate, roll, rollRate]
        io.setDoubleArray("robot_orientation_set", new double[]{yawDegrees, 0, 0, 0, 0, 0});
    }

    public boolean hasTarget() {
        return cachedHasTarget;
    }

    public double getHorizontalOffset() {
        return io.getDouble("tx", 0);
    }

    public double getVerticalOffset() {
        return io.getDouble("ty", 0);
    }

    public double getTargetArea() {
        return io.getDouble("ta", 0);
    }

    public double getTargetSkew() {
        return io.getDouble("ts", 0);
    }

    public double getDistanceToTarget() {
        double targetOffsetAngleVertical = getVerticalOffset();
        double angleToGoalDegrees = GorusSabitleri.LIMELIGHT_ACISI + targetOffsetAngleVertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        double distanceInches =
            (GorusSabitleri.HEDEF_YUKSEKLIGI - GorusSabitleri.LIMELIGHT_YUKSEKLIGI) / Math.tan(angleToGoalRadians);
        return distanceInches * 0.0254;
    }

    public int getTagId() {
        if (!hasTarget()) {
            return 0;
        }
        return (int) io.getInteger("tid", 0);
    }

    /** Görünen tag, mevcut ittifakın atış hedefi mi? */
    public boolean isHedefTagGorunuyor() {
        if (!cachedHasTarget) return false;
        int tagId = getTagId();
        int[] hedefler = runtime.isRedAlliance()
            ? GorusSabitleri.KIRMIZI_HEDEF_TAGLERI
            : GorusSabitleri.MAVI_HEDEF_TAGLERI;
        for (int id : hedefler) {
            if (id == tagId) return true;
        }
        return false;
    }

    public int getFmapTagCount() {
        return (int) io.getInteger("fmap/tagCount", 0);
    }

    public String getFmapStatus() {
        int tagCount = getFmapTagCount();
        String fmapSize = io.getString("fmap/size", "Unknown");
        return String.format("FMAP: %d tags, Size: %s", tagCount, fmapSize);
    }

    public int getCurrentPipeline() {
        return (int) io.getDouble("pipeline", -1);
    }

    public int getCurrentCamMode() {
        return (int) io.getDouble("camMode", -1);
    }

    public int getCurrentLedMode() {
        return (int) io.getDouble("ledMode", -1);
    }

    public int getCurrentStreamMode() {
        return (int) io.getDouble("stream", -1);
    }

    private void initializeTuningDashboard() {
        SmartDashboard.putBoolean("Vision/Tune/Enable", false);
        SmartDashboard.putNumber("Vision/Tune/Pipeline", GorusSabitleri.ISTENEN_HAT);
        SmartDashboard.putNumber("Vision/Tune/SourceImageCamMode", GorusSabitleri.ISTENEN_KAMERA_MODU);
        SmartDashboard.putNumber("Vision/Tune/LEDMode", GorusSabitleri.ISTENEN_LED_MODU);
        SmartDashboard.putNumber("Vision/Tune/StreamMode", GorusSabitleri.ISTENEN_YAYIN_MODU);
        SmartDashboard.putString("Vision/Tune/Unsupported", "Resolution, Stream Orientation, Exposure, LED Power -> Limelight UI");
        tuningInitialized = true;
    }

    private int getDesiredPipeline() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/Pipeline", GorusSabitleri.ISTENEN_HAT);
        }
        return GorusSabitleri.ISTENEN_HAT;
    }

    private int getDesiredCamMode() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/SourceImageCamMode", GorusSabitleri.ISTENEN_KAMERA_MODU);
        }
        return GorusSabitleri.ISTENEN_KAMERA_MODU;
    }

    private int getDesiredLedMode() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/LEDMode", GorusSabitleri.ISTENEN_LED_MODU);
        }
        return GorusSabitleri.ISTENEN_LED_MODU;
    }

    private int getDesiredStreamMode() {
        if (SmartDashboard.getBoolean("Vision/Tune/Enable", false)) {
            return (int) SmartDashboard.getNumber("Vision/Tune/StreamMode", GorusSabitleri.ISTENEN_YAYIN_MODU);
        }
        return GorusSabitleri.ISTENEN_YAYIN_MODU;
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
        if (fmapTagCount < GorusSabitleri.TOPLAM_APRILTAG) {
            return String.format("FMAP partial (%d/%d tags)", fmapTagCount, GorusSabitleri.TOPLAM_APRILTAG);
        }
        return "OK";
    }

    private void updateCachedVisionData() {
        cachedHasTarget = io.getDouble("tv", 0) == 1;
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

        // Debug: Check raw tv value
        double tv = io.getDouble("tv", 0);
        SmartDashboard.putBoolean("Vision/Debug/RawTV", tv == 1);

        // BUG FIX: Check raw tv value, not cached value!
        if (tv != 1) {
            SmartDashboard.putString("Vision/Debug/Status", "No target (tv=" + tv + ")");
            return result;
        }

        // MegaTag2: botpose_orb_* — jiro destekli, cok daha kararli
        String poseEntry = runtime.isRedAlliance() ? "botpose_orb_wpired" : "botpose_orb_wpiblue";
        SmartDashboard.putString("Vision/Debug/PoseEntry", poseEntry);

        double[] botPoseArray = io.getDoubleArray(poseEntry, new double[0]);
        SmartDashboard.putNumber("Vision/Debug/PoseArrayLength", botPoseArray.length);

        // MegaTag2 array: [x, y, z, rx, ry, rz, latency_ms, tagCount, tagSpan, avgTagDist, avgTagArea]
        if (botPoseArray.length < 7) {
            SmartDashboard.putString("Vision/Debug/Status", "Array too short: " + botPoseArray.length);
            return result;
        }

        double x = botPoseArray[0];
        double y = botPoseArray[1];
        double yaw = botPoseArray[5];
        double latency = botPoseArray[6] / 1000.0;
        double timestamp = runtime.nowSec() - latency;

        result.robotPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
        result.timestamp = timestamp;
        result.tagId = (int) io.getInteger("tid", 0);
        result.tagCount = botPoseArray.length > 7 ? (int) botPoseArray[7] : 1;
        result.avgTagDist = botPoseArray.length > 9 ? botPoseArray[9] : 1.0;

        double[] targetPoseArray = io.getDoubleArray("targetpose_cameraspace", new double[0]);
        result.ambiguity = (targetPoseArray.length >= 7) ? targetPoseArray[6] : 1.0;

        // Debug values
        SmartDashboard.putNumber("Vision/Debug/TagID", result.tagId);
        SmartDashboard.putNumber("Vision/Debug/RobotX", x);
        SmartDashboard.putNumber("Vision/Debug/RobotY", y);
        SmartDashboard.putNumber("Vision/Debug/Ambiguity", result.ambiguity);

        // Validate result
        result.valid = (result.ambiguity < GorusSabitleri.BELIRSIZLIK_ESIGI) && (result.tagId > 0);

        SmartDashboard.putBoolean("Vision/Debug/Valid", result.valid);
        SmartDashboard.putString("Vision/Debug/Status", result.valid ? "OK" : "Invalid");

        return result;
    }

    @Override
    public void periodic() {
        if (runtime.isReal()) {
            if (!tuningInitialized) {
                initializeTuningDashboard();
            }

            double nowSec = runtime.nowSec();
            if (lastConfigApplyTimestampSec < 0
                || nowSec - lastConfigApplyTimestampSec >= GorusSabitleri.YAPILANDIRMA_YENIDEN_UYGULAMA_ARALIGI_SN) {
                applyDesiredLimelightConfig();
            }

            updateCachedVisionData();

            // NetworkTables connectivity test
            SmartDashboard.putBoolean("Vision/NT/Connected", io.getDouble("tv", -999) != -999);

            SmartDashboard.putBoolean("Vision/HasTarget", cachedHasTarget);
            if (cachedHasTarget) {
                SmartDashboard.putNumber("Vision/HorizontalOffset", io.getDouble("tx", 0));
                SmartDashboard.putNumber("Vision/VerticalOffset", io.getDouble("ty", 0));
                SmartDashboard.putNumber("Vision/TargetArea", io.getDouble("ta", 0));
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
        } else {
            SmartDashboard.putBoolean("Vision/HasTarget", true);
            SmartDashboard.putNumber("Vision/HorizontalOffset", 5.0);
            SmartDashboard.putNumber("Vision/DistanceToTarget", 3.0);
            SmartDashboard.putBoolean("Vision/PoseValid", true);
            SmartDashboard.putNumber("Vision/RobotX", 2.0);
            SmartDashboard.putNumber("Vision/RobotY", 3.0);
            SmartDashboard.putNumber("Vision/RobotYaw", 0.0);
            SmartDashboard.putNumber("Vision/TagID", 1);
            SmartDashboard.putString("Vision/Mode", "Simulation");
        }
    }

    public VisionResult aprilTagdanRobotPozuAl() {
        return cachedVisionResult;
    }
}
