// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;
import frc.robot.Constants.VisionConstants;

/**
 * AprilTag tespit simülasyonu.
 * Robotun sahada olduğu konuma göre en yakın görünen AprilTag'leri hesaplar.
 * Vision tabanlı komutları simülasyon modunda test etmek için kullanılır.
 */
public class VisionSimulator {
    private Pose2d robotPose;
    private static final double MAX_DETECTION_RANGE_METERS = 3.0;
    private static final double CAMERA_FOV_RAD = Math.toRadians(60.0); // 60 degree FOV

    /**
     * Yeni bir vision simülatörü oluşturur.
     */
    public VisionSimulator() {
        this.robotPose = new Pose2d();
    }

    /**
     * Robotun pozisyonunu günceller.
     * Simülasyon modunda DriveSubsystem tarafından çağrılır.
     *
     * @param pose Robotun mevcut pozisyonu
     */
    public void setRobotPose(Pose2d pose) {
        this.robotPose = pose;
    }

    /**
     * Herhangi bir AprilTag'in görünüp görünmediğini kontrol eder.
     *
     * @return true varsa en az bir AprilTag algılanabilir
     */
    public boolean hasTarget() {
        return getNearestTagId() != -1;
    }

    /**
     * En yakın AprilTag'in ID'sini döndürür.
     *
     * @return En yakın AprilTag ID'si, veya -1 hiçbiri algılanamazsa
     */
    public int getNearestTagId() {
        double minDistance = MAX_DETECTION_RANGE_METERS;
        int nearestId = -1;

        for (int i = 0; i < FieldConstants.APRIL_TAG_POSES.length; i++) {
            Pose2d tagPose = FieldConstants.APRIL_TAG_POSES[i];
            double distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                nearestId = i + 1; // IDs are 1-indexed
            }
        }
        return nearestId;
    }

    /**
     * En yakın AprilTag'in mesafesini döndürür (metre).
     *
     * @return Mesafe (metre), veya varsayılan 3.0m
     */
    public double getDistanceToTarget() {
        int nearestId = getNearestTagId();
        if (nearestId != -1) {
            Pose2d tagPose = FieldConstants.APRIL_TAG_POSES[nearestId - 1];
            return robotPose.getTranslation().getDistance(tagPose.getTranslation());
        }
        return 3.0; // Varsayılan simülasyon mesafesi
    }

    /**
     * Hedefe olan yatay ofseti hesaplar (derece).
     * Kamera görüş alanı (FOV) içinde mi kontrolü dahildir.
     *
     * @return Yatay ofset (derece), veya 0 hedef yoksa
     */
    public double getHorizontalOffset() {
        int nearestId = getNearestTagId();
        if (nearestId != -1) {
            Pose2d tagPose = FieldConstants.APRIL_TAG_POSES[nearestId - 1];
            double dx = tagPose.getX() - robotPose.getX();
            double dy = tagPose.getY() - robotPose.getY();

            // Hedef açısını hesapla
            double angleToTag = Math.atan2(dx, dy);

            // Robotun mevcut yönü ile hedef açısı arasındaki farkı hesapla
            double robotHeading = robotPose.getRotation().getRadians();
            double angleDifference = angleToTag - robotHeading;

            // Açıyı [-π, π] aralığına normalize et
            while (angleDifference > Math.PI) angleDifference -= 2 * Math.PI;
            while (angleDifference < -Math.PI) angleDifference += 2 * Math.PI;

            // FOV kontrolü
            if (Math.abs(angleDifference) < CAMERA_FOV_RAD / 2.0) {
                return Math.toDegrees(angleDifference);
            }
        }
        return 0.0;
    }

    /**
     * Hedef AprilTag'in ID'sini döndürür.
     *
     * @return Tag ID (1-8), veya 1 hedef yoksa
     */
    public int getTagId() {
        int nearestId = getNearestTagId();
        return nearestId != -1 ? nearestId : 1;
    }

    /**
     * Robotun sahadaki konumuna göre vision sonucunu döndürür.
     * VisionSubsystem ile entegrasyon için kullanılır.
     *
     * @return VisionSonucu (pozisyon, tag ID, geçerlilik durumu)
     */
    public VisionResult getVisionResult() {
        VisionResult result = new VisionResult();
        result.valid = hasTarget();
        result.tagId = getTagId();

        if (result.valid) {
            // Robot pozisyonunu hesapla (en yakın tag'e göre)
            int nearestId = getNearestTagId();
            Pose2d tagPose = FieldConstants.APRIL_TAG_POSES[nearestId - 1];

            // Mesafeye dayalı gerçekçi ambigüite hesapla
            double distance = getDistanceToTarget();

            // Ambiguity modeli (gerçekçi Limelight verilerine dayalı):
            // - 0.5m: ~0.05 (çok iyi)
            // - 1.0m: ~0.10 (iyi)
            // - 1.5m: ~0.14 (iyi)
            // - 2.0m: ~0.18 (borderline)
            // - 2.5m: ~0.22 (kötü)
            // - 3.0m: ~0.26 (çok kötü)
            // Formül: ambiguity = 0.02 + distance * 0.08 (daha gerçekçi)
            result.ambiguity = 0.02 + distance * 0.08;

            // FOV kontrolü - hedef görüş alanında mı?
            double horizontalOffset = getHorizontalOffset();
            double offsetDegrees = Math.abs(horizontalOffset);

            // FOV kenarlarına yakın olma cezası ekle
            // Merkezde (0°): ek ceza yok
            // FOV/2'de (30°): +0.15 ambiguity
            double fovPenalty = (offsetDegrees / (CAMERA_FOV_RAD / 2.0)) * 0.15;
            result.ambiguity += fovPenalty;

            // Ambiguity'yi [0, 1] aralığına sınırla
            result.ambiguity = Math.max(0.0, Math.min(1.0, result.ambiguity));

            // Robot pozisyonunu kullan (simülasyon)
            result.robotPose = robotPose;
        } else {
            // Hedef yok - varsayılan değerler
            result.robotPose = robotPose;
            result.ambiguity = 1.0; // Yüksek belirsizlik
        }

        return result;
    }

    /**
     * Vision sonucu için veri sınıfı.
     */
    public static class VisionResult {
        public Pose2d robotPose;
        public int tagId;
        public boolean valid;
        public double ambiguity;
        public double timestamp;

        public VisionResult() {
            this.robotPose = new Pose2d();
            this.tagId = 1;
            this.valid = false;
            this.ambiguity = 1.0;
            this.timestamp = 0.0;
        }
    }
}
