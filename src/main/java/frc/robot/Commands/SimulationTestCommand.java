// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

/**
 * Sürücü treni simülasyon doğrulama komutu.
 * Robotu 1 metrelik kare patterns içinde sürerek odometri takibini doğrular.
 * Sadece simülasyon modunda test için kullanılır.
 */
public class SimulationTestCommand extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;

    private double startTime;
    private int phase = 0;
    private Pose2d initialPose;
    private static final double TEST_SPEED = 0.3;
    private static final double TEST_DURATION_PER_PHASE = 2.0; // seconds

    /**
     * Yeni bir simülasyon test komutu oluşturur.
     *
     * @param drive Sürücü alt sistemi
     */
    public SimulationTestCommand(DriveSubsystem drive) {
        this(drive, null);
    }

    /**
     * Yeni bir simülasyon test komutu oluşturur (vision ile).
     *
     * @param drive Sürücü alt sistemi
     * @param vision Vision alt sistemi (opsiyonel)
     */
    public SimulationTestCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis() / 1000.0;
        phase = 0;
        initialPose = drive.getPose();
        drive.resetEncoders();
        drive.zeroHeading();
        System.out.println("========================================");
        System.out.println("SIMULATION TEST: Starting");
        System.out.println("Initial Pose: (" + initialPose.getX() + ", " + initialPose.getY() + ")");
        System.out.println("========================================");
    }

    @Override
    public void execute() {
        double elapsed = (System.currentTimeMillis() / 1000.0) - startTime;
        int currentPhase = (int) (elapsed / TEST_DURATION_PER_PHASE);

        if (currentPhase > phase) {
            // Faz değişti - durum raporu
            Pose2d currentPose = drive.getPose();
            System.out.println("SIMULATION TEST: Phase " + phase + " complete");
            System.out.println("  Current Pose: (" + currentPose.getX() + ", " + currentPose.getY() + ")");
            phase = currentPhase;
        }

        // 1m kare pattern drive et
        switch (phase) {
            case 0:
                // İleri (2 saniye)
                drive.driveFieldOriented(TEST_SPEED, 0, 0);
                break;
            case 1:
                // Sağ (2 saniye)
                drive.driveFieldOriented(0, TEST_SPEED, 0);
                break;
            case 2:
                // Geri (2 saniye)
                drive.driveFieldOriented(-TEST_SPEED, 0, 0);
                break;
            case 3:
                // Sol (2 saniye)
                drive.driveFieldOriented(0, -TEST_SPEED, 0);
                break;
            default:
                // Test tamamlandı - dur
                drive.stopAllMotors();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopAllMotors();
        Pose2d finalPose = drive.getPose();
        double distanceError = finalPose.getTranslation().getDistance(initialPose.getTranslation());

        System.out.println("========================================");
        System.out.println("SIMULATION TEST: " + (interrupted ? "INTERRUPTED" : "COMPLETED"));
        System.out.println("Initial Pose: (" + initialPose.getX() + ", " + initialPose.getY() + ")");
        System.out.println("Final Pose: (" + finalPose.getX() + ", " + finalPose.getY() + ")");
        System.out.println("Distance Error: " + distanceError + " meters");
        System.out.println("Heading Error: " + Math.abs(finalPose.getRotation().getDegrees() - initialPose.getRotation().getDegrees()) + " degrees");

        if (distanceError < 0.2) {
            System.out.println("RESULT: PASS - Odometry tracking is accurate!");
        } else {
            System.out.println("RESULT: FAIL - Odometry error is too large!");
        }
        System.out.println("========================================");
    }

    @Override
    public boolean isFinished() {
        double elapsed = (System.currentTimeMillis() / 1000.0) - startTime;
        return elapsed > (TEST_DURATION_PER_PHASE * 4.5); // 4 faz + 0.5s buffer
    }
}
