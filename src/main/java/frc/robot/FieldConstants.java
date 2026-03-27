package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    // 2026 FRC Field Dimensions (meters)
    public static final double FIELD_LENGTH = 16.54; // 54 feet 3 inches
    public static final double FIELD_WIDTH = 8.21;   // 26 feet 11 inches

    // April Tag locations for 2026 (simplified positions)
    public static final Pose2d[] APRIL_TAG_POSES = {
        new Pose2d(3.0, 0.5, Rotation2d.fromDegrees(0)),     // Speaker center
        new Pose2d(3.0, 7.71, Rotation2d.fromDegrees(0)),    // Speaker opposite
        new Pose2d(14.7, 2.5, Rotation2d.fromDegrees(180)),  // Amp
        new Pose2d(14.7, 5.71, Rotation2d.fromDegrees(180)), // Amp opposite
        new Pose2d(0.5, 4.1, Rotation2d.fromDegrees(90)),    // Source
        new Pose2d(15.9, 4.1, Rotation2d.fromDegrees(-90)),  // Source opposite
        new Pose2d(10.0, 2.0, Rotation2d.fromDegrees(120)),  // Stage left
        new Pose2d(10.0, 6.21, Rotation2d.fromDegrees(-120)) // Stage right
    };

    public static void setupField(Field2d field) {
        // Add April Tag positions to field visualization
        for (int i = 0; i < APRIL_TAG_POSES.length; i++) {
            field.getObject("AprilTag_" + (i + 1)).setPose(APRIL_TAG_POSES[i]);
        }

        // Add field boundaries
        field.getObject("FieldOutline").setPoses(
            new Pose2d(0, 0, new Rotation2d()),
            new Pose2d(FIELD_LENGTH, 0, new Rotation2d()),
            new Pose2d(FIELD_LENGTH, FIELD_WIDTH, new Rotation2d()),
            new Pose2d(0, FIELD_WIDTH, new Rotation2d()),
            new Pose2d(0, 0, new Rotation2d())
        );

        // Add key field elements
        // Speaker (blue alliance side)
        field.getObject("Speaker").setPose(new Pose2d(0, FIELD_WIDTH/2, Rotation2d.fromDegrees(180)));

        // Amp zone
        field.getObject("Amp").setPose(new Pose2d(FIELD_LENGTH, 5.71, Rotation2d.fromDegrees(180)));

        // Source zones
        field.getObject("Source_Blue").setPose(new Pose2d(0.5, 4.1, Rotation2d.fromDegrees(90)));
        field.getObject("Source_Red").setPose(new Pose2d(15.9, 4.1, Rotation2d.fromDegrees(-90)));

        SmartDashboard.putData("2026 FRC Field", field);
    }
}