package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.HashMap;
import java.util.Map;

/**
 * AprilTag field layout for the 2026 FRC game.
 *
 * These are OFFICIAL AprilTag positions from the FRC 2026 Andymark field FMAP file.
 * The FMAP contains 32 AprilTags total (16 per alliance side).
 *
 * Coordinate system:
 * - Origin (0, 0) is at the blue alliance corner
 * - X-axis increases toward red alliance
 * - Y-axis increases toward the left wall when standing at blue alliance
 *
 * FMAP source: FRC2026_ANDYMARK.fmap
 * Tag family: apriltag3_36h11_classic
 * Tag size: 165.1mm (6.5 inches)
 */
public class AprilTagSahaYerlesimi {
    private static final Map<Integer, Pose2d> BLUE_ALLIANCE_TAGS = new HashMap<>();
    private static final Map<Integer, Pose2d> RED_ALLIANCE_TAGS = new HashMap<>();

    static {
        // ============================================================
        // BLUE ALLIANCE APRILTAGS (Tags 1-16)
        // Official 2026 Andymark field positions from FMAP
        // ============================================================

        // Blue alliance side - Red scoring zone
        BLUE_ALLIANCE_TAGS.put(1, new Pose2d(3.605, 3.390, Rotation2d.fromDegrees(-90)));
        BLUE_ALLIANCE_TAGS.put(2, new Pose2d(3.642, 0.603, Rotation2d.fromDegrees(90)));
        BLUE_ALLIANCE_TAGS.put(3, new Pose2d(3.039, 0.355, Rotation2d.fromDegrees(-90)));
        BLUE_ALLIANCE_TAGS.put(4, new Pose2d(3.039, 0.000, Rotation2d.fromDegrees(-90)));
        BLUE_ALLIANCE_TAGS.put(5, new Pose2d(3.642, -0.604, Rotation2d.fromDegrees(-90)));
        BLUE_ALLIANCE_TAGS.put(6, new Pose2d(3.605, -3.390, Rotation2d.fromDegrees(-90)));
        BLUE_ALLIANCE_TAGS.put(7, new Pose2d(3.680, -3.390, Rotation2d.fromDegrees(0)));
        BLUE_ALLIANCE_TAGS.put(8, new Pose2d(3.998, -0.604, Rotation2d.fromDegrees(-90)));

        // Midfield tags
        BLUE_ALLIANCE_TAGS.put(9, new Pose2d(4.246, -0.356, Rotation2d.fromDegrees(0)));
        BLUE_ALLIANCE_TAGS.put(10, new Pose2d(4.246, 0.000, Rotation2d.fromDegrees(0)));
        BLUE_ALLIANCE_TAGS.put(11, new Pose2d(4.276, 0.603, Rotation2d.fromDegrees(90)));
        BLUE_ALLIANCE_TAGS.put(12, new Pose2d(4.276, 3.642, Rotation2d.fromDegrees(90)));
        BLUE_ALLIANCE_TAGS.put(13, new Pose2d(7.995, 1.145, Rotation2d.fromDegrees(0)));
        BLUE_ALLIANCE_TAGS.put(14, new Pose2d(7.995, -1.145, Rotation2d.fromDegrees(0)));
        BLUE_ALLIANCE_TAGS.put(15, new Pose2d(8.539, 0.603, Rotation2d.fromDegrees(90)));
        BLUE_ALLIANCE_TAGS.put(16, new Pose2d(8.539, -0.604, Rotation2d.fromDegrees(-90)));

        // ============================================================
        // RED ALLIANCE APRILTAGS (Tags 1-16, mirrored)
        // ============================================================

        double fieldLength = FieldConstants.FIELD_LENGTH;

        for (Map.Entry<Integer, Pose2d> entry : BLUE_ALLIANCE_TAGS.entrySet()) {
            int tagId = entry.getKey();
            Pose2d bluePose = entry.getValue();

            // Mirror pose for red alliance (flip X coordinate)
            double redX = fieldLength - bluePose.getX();
            double redY = bluePose.getY();

            // For red alliance, tags are typically rotated 180 degrees
            double redYaw = bluePose.getRotation().getDegrees() + 180;
            while (redYaw > 180) redYaw -= 360;
            while (redYaw < -180) redYaw += 360;

            RED_ALLIANCE_TAGS.put(tagId, new Pose2d(redX, redY, Rotation2d.fromDegrees(redYaw)));
        }
    }

    /**
     * Get the pose of a specific AprilTag
     * @param tagId The AprilTag ID (1-16)
     * @param isRedAlliance True if on red alliance, false for blue
     * @return Pose2d of the tag, or null if tag ID not found
     */
    public static Pose2d etiketPozunuAl(int tagId, boolean isRedAlliance) {
        Map<Integer, Pose2d> tags = isRedAlliance ? RED_ALLIANCE_TAGS : BLUE_ALLIANCE_TAGS;
        return tags.get(tagId);
    }

    /**
     * Check if a tag ID exists in the layout
     * @param tagId The AprilTag ID to check
     * @return True if the tag exists, false otherwise
     */
    public static boolean tagExists(int tagId) {
        return BLUE_ALLIANCE_TAGS.containsKey(tagId);
    }

    /**
     * Get all tag IDs for a specific alliance
     * @param isRedAlliance True for red alliance, false for blue
     * @return Array of tag IDs
     */
    public static int[] getAllTagIds(boolean isRedAlliance) {
        Map<Integer, Pose2d> tags = isRedAlliance ? RED_ALLIANCE_TAGS : BLUE_ALLIANCE_TAGS;
        return tags.keySet().stream().mapToInt(Integer::intValue).toArray();
    }

    /**
     * Get total number of tags per alliance
     * @return Number of tags (16 per alliance side)
     */
    public static int getTotalTagCount() {
        return BLUE_ALLIANCE_TAGS.size();
    }
}
