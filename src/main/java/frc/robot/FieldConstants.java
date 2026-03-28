package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldConstants {
    // 2026 FRC Field Dimensions (meters)
    public static final double FIELD_LENGTH = 16.54; // 54 feet 3 inches
    public static final double FIELD_WIDTH = 8.21;   // 26 feet 11 inches

    /**
     * 2026 Hub (scoring yapısı) merkez koordinatları — WPILib mavi-alliance orijini.
     * Kaynak: Hammerheads 5000 fmap ölçümleri (181.56 in = 4.61 m)
     *
     * Taret arka tarafta, Limelight arka bakıyor.
     * Tareti hedefe döndürmek için bu koordinatlar kullanılır.
     */
    public static final Translation2d HUB_MAVI = new Translation2d(4.61, FIELD_WIDTH / 2.0);
    public static final Translation2d HUB_KIRMIZI = new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2.0);

    public static void setupField(Field2d field) {
        field.getObject("Hub_Mavi").setPose(new Pose2d(HUB_MAVI, new Rotation2d()));
        field.getObject("Hub_Kirmizi").setPose(new Pose2d(HUB_KIRMIZI, new Rotation2d()));

        // Saha siniri
        field.getObject("FieldOutline").setPoses(
            new Pose2d(0, 0, new Rotation2d()),
            new Pose2d(FIELD_LENGTH, 0, new Rotation2d()),
            new Pose2d(FIELD_LENGTH, FIELD_WIDTH, new Rotation2d()),
            new Pose2d(0, FIELD_WIDTH, new Rotation2d()),
            new Pose2d(0, 0, new Rotation2d())
        );

        SmartDashboard.putData("2026 FRC Field", field);
    }
}
