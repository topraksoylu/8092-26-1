package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.AprilTagFieldLayout;

/**
 * Aligns the robot to face and drive toward a specific AprilTag.
 * Uses vision feedback to position the robot in front of the tag.
 */
public class AlignToAprilTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    // Target configuration
    private final int targetTagId;
    private final double targetDistanceMeters;  // How far in front of tag to stop
    private final double toleranceMeters;        // Position tolerance
    private final double toleranceDegrees;      // Heading tolerance

    // PID controllers for holonomic alignment
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    // Alliance configuration
    private boolean isRedAlliance;

    /**
     * Creates a new AlignToAprilTagCommand.
     *
     * @param driveSubsystem The drive subsystem to use
     * @param visionSubsystem The vision subsystem to use
     * @param targetTagId The AprilTag ID to align to (1-16)
     * @param targetDistanceMeters How far in front of the tag to stop (default: 1.0m)
     */
    public AlignToAprilTagCommand(
            DriveSubsystem driveSubsystem,
            VisionSubsystem visionSubsystem,
            int targetTagId,
            double targetDistanceMeters) {

        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetTagId = targetTagId;
        this.targetDistanceMeters = targetDistanceMeters;
        this.toleranceMeters = 0.15;  // 15cm tolerance
        this.toleranceDegrees = 5.0;    // 5 degree heading tolerance

        // Create PID controllers
        // Very reduced gains for extremely smooth, slow tracking
        xController = new PIDController(0.5, 0, 0.02);    // Forward/back (very slow)
        yController = new PIDController(0.5, 0, 0.02);    // Strafe (very slow)
        thetaController = new PIDController(0.1, 0, 0.005); // Rotation (MUCH slower to prevent oscillation)

        // Enable continuous input for theta (handles wraparound at -180/180)
        thetaController.enableContinuousInput(-180, 180);

        // Set tolerances
        xController.setTolerance(toleranceMeters);
        yController.setTolerance(toleranceMeters);
        thetaController.setTolerance(toleranceDegrees);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("===========================================");
        System.out.println("ALIGN TO APRILTAG COMMAND INITIALIZED");
        System.out.println("Mode: TRACKING - Will follow moving target");
        System.out.println("Target Tag ID: " + targetTagId);
        System.out.println("Target Distance: " + targetDistanceMeters + " meters");
        System.out.println("Hold Button 6 to track, release to stop");
        System.out.println("===========================================");

        // Determine alliance at command start
        isRedAlliance = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        // Initialize all SmartDashboard values
        SmartDashboard.putString("AlignToAprilTag/Status", "Initialized");
        SmartDashboard.putNumber("AlignToAprilTag/TargetTag", targetTagId);
        SmartDashboard.putBoolean("AlignToAprilTag/IsRed", isRedAlliance);
        SmartDashboard.putBoolean("AlignToAprilTag/ResultValid", false);
        SmartDashboard.putNumber("AlignToAprilTag/ResultTagID", 0);
        SmartDashboard.putNumber("AlignToAprilTag/DistanceToTag", 0.0);
        SmartDashboard.putNumber("AlignToAprilTag/DistanceError", 0.0);
        SmartDashboard.putNumber("AlignToAprilTag/XError", 0.0);
        SmartDashboard.putNumber("AlignToAprilTag/YError", 0.0);
        SmartDashboard.putNumber("AlignToAprilTag/YawError", 0.0);
        SmartDashboard.putNumber("AlignToAprilTag/XSpeed", 0.0);
        SmartDashboard.putNumber("AlignToAprilTag/YSpeed", 0.0);
        SmartDashboard.putNumber("AlignToAprilTag/ThetaSpeed", 0.0);
        SmartDashboard.putNumber("AlignToAprilTag/SpeedScale", 0.0);

        System.out.println("SmartDashboard values initialized - check 'AlignToAprilTag' table");
    }

    @Override
    public void execute() {
        // DEBUG: Show that execute() is being called
        SmartDashboard.putString("AlignToAprilTag/Status", "Execute running...");

        // Get vision result
        VisionSubsystem.VisionResult result = visionSubsystem.getRobotPoseFromAprilTag();

        SmartDashboard.putBoolean("AlignToAprilTag/ResultValid", result.valid);
        SmartDashboard.putNumber("AlignToAprilTag/ResultTagID", result.tagId);

        // If no valid vision target, stop
        if (!result.valid) {
            SmartDashboard.putString("AlignToAprilTag/Status", "No valid vision");
            System.out.println("AlignToAprilTag: No valid vision - stopping");
            driveSubsystem.stopAllMotors();
            return;
        }

        if (result.tagId != targetTagId) {
            SmartDashboard.putString("AlignToAprilTag/Status", "Wrong tag: " + result.tagId);
            System.out.println("AlignToAprilTag: Wrong tag ID " + result.tagId + " (want " + targetTagId + ")");
            driveSubsystem.stopAllMotors();
            return;
        }

        // Get target tag pose from field layout
        Pose2d targetPose = AprilTagFieldLayout.getTagPose(targetTagId, isRedAlliance);
        if (targetPose == null) {
            SmartDashboard.putString("AlignToAprilTag/Status", "Tag pose not found");
            System.out.println("AlignToAprilTag: Tag pose not found for ID " + targetTagId);
            driveSubsystem.stopAllMotors();
            return;
        }

        // Current robot pose from vision
        Pose2d currentPose = result.robotPose;
        // Debug output DISABLED - flooding console
        // System.out.println(String.format("AlignToAprilTag: Robot at (%.2f, %.2f, %.1f°), Tag at (%.2f, %.2f)",
        //     currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees(),
        //     targetPose.getX(), targetPose.getY()));

        // SIMPLIFIED APPROACH: Move directly toward the tag
        // Calculate vector from robot to tag
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double distanceToTag = Math.sqrt(dx * dx + dy * dy);

        // Calculate angle to the tag
        double angleToTag = Math.atan2(dy, dx);
        double currentYaw = currentPose.getRotation().getRadians();
        double thetaError = angleToTag - currentYaw;

        // Normalize theta error to -PI to PI
        while (thetaError > Math.PI) thetaError -= 2 * Math.PI;
        while (thetaError < -Math.PI) thetaError += 2 * Math.PI;

        // We want to maintain targetDistanceMeters from the tag
        // So we drive toward/away from tag to achieve that distance
        double distanceError = distanceToTag - targetDistanceMeters;

        // Forward/back is along the robot's heading direction
        // Strafe is perpendicular to robot's heading
        // We need to transform distanceError into robot-relative coordinates

        // In field coordinates, the direction to the tag is (dx, dy)
        // We need to rotate this by -currentYaw to get robot-relative coordinates
        double cosYaw = Math.cos(-currentYaw);
        double sinYaw = Math.sin(-currentYaw);

        // Transform field-relative error to robot-relative
        double forwardError = dx * cosYaw - dy * sinYaw;  // Forward/back
        double strafeError = dx * sinYaw + dy * cosYaw;    // Left/right

        // Use forward/strafe errors for PID
        // Keep in FIELD RELATIVE coordinates (no transformation needed)
        double xError = dx;  // Field X (strafe)
        double yError = dy;  // Field Y (forward)

        // Debug output
        SmartDashboard.putNumber("AlignToAprilTag/DistanceToTag", distanceToTag);
        SmartDashboard.putNumber("AlignToAprilTag/DistanceError", distanceError);
        SmartDashboard.putNumber("AlignToAprilTag/XError", xError);
        SmartDashboard.putNumber("AlignToAprilTag/YError", yError);
        SmartDashboard.putNumber("AlignToAprilTag/YawError", Math.toDegrees(thetaError));

        // Calculate drive outputs using PID
        double xSpeed = xController.calculate(0, xError);
        double ySpeed = yController.calculate(0, yError);
        double thetaSpeed = thetaController.calculate(0, Math.toDegrees(thetaError));

        // HARD SPEED LIMITS - Never exceed these speeds
        // Run ~4x slower to keep the tag in view while approaching (increased from 8x for faster alignment).
        double maxXSpeed = 0.075;       // 0.3 / 4 (7.5% of max speed)
        double maxYSpeed = 0.075;       // 0.3 / 4 (7.5% of max speed)
        double maxThetaSpeed = 0.025;   // 2.5% of max speed (keep rotation lower to avoid side-canceling)

        // Clamp speeds
        xSpeed = Math.max(-maxXSpeed, Math.min(maxXSpeed, xSpeed));
        ySpeed = Math.max(-maxYSpeed, Math.min(maxYSpeed, ySpeed));
        thetaSpeed = Math.max(-maxThetaSpeed, Math.min(maxThetaSpeed, thetaSpeed));

        // Slow down rotation when well-aligned to prevent losing the tag
        double headingErrorDegrees = Math.abs(Math.toDegrees(thetaError));
        double alignmentSpeedScale = 1.0;
        double rotationSpeedScale = 1.0;

        // CRITICAL: Strong deadband to prevent oscillation when facing the tag
        // Don't rotate at all if we're reasonably aligned - this prevents 360° spinning
        if (headingErrorDegrees < 5.0) {
            // Well aligned - NO rotation whatsoever
            rotationSpeedScale = 0.0;     // Completely disable rotation
            alignmentSpeedScale = 0.6;    // Slow movement for precise positioning
        } else if (headingErrorDegrees < 15.0) {
            // Moderately aligned - very slow rotation
            alignmentSpeedScale = 0.7;    // 70% speed
            rotationSpeedScale = 0.2;     // Only 20% rotation
        } else {
            // Poorly aligned - allow rotation but keep it slow
            alignmentSpeedScale = 0.8;    // 80% speed
            rotationSpeedScale = 0.5;     // Only 50% rotation
        }

        // ADDITIONAL: If very close to target distance, reduce rotation even more
        // This prevents spinning when we're at the right spot
        if (Math.abs(distanceError) < 0.3) {
            // Within 30cm of target distance - very minimal rotation
            rotationSpeedScale *= 0.3;
            if (headingErrorDegrees < 10.0) {
                rotationSpeedScale = 0.0;  // No rotation if close and reasonably aligned
            }
        }

        // Apply the speed scaling
        xSpeed *= alignmentSpeedScale;
        ySpeed *= alignmentSpeedScale;
        thetaSpeed *= rotationSpeedScale;

        // When still far from the target, prioritize translation over rotation.
        // This prevents one side's wheel commands from being canceled by rotate+translate mixing.
        if (Math.abs(distanceError) > 0.4) {
            thetaSpeed *= 0.3;
        }

        SmartDashboard.putNumber("AlignToAprilTag/XSpeed", xSpeed);
        SmartDashboard.putNumber("AlignToAprilTag/YSpeed", ySpeed);
        SmartDashboard.putNumber("AlignToAprilTag/ThetaSpeed", thetaSpeed);
        SmartDashboard.putNumber("AlignToAprilTag/SpeedScale", alignmentSpeedScale);
        SmartDashboard.putString("AlignToAprilTag/Status", "Tracking");

        // Match translation orientation with teleop DriveCommand conventions.
        // This keeps button-6 alignment behavior consistent with manual driving.
        driveSubsystem.driveFieldOriented(ySpeed, -xSpeed, thetaSpeed);
    }

    @Override
    public boolean isFinished() {
        // NEVER finish automatically - always track the moving target
        // Command only ends when button 6 is released (controlled by RobotContainer)
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("===========================================");
        System.out.println("ALIGN TO APRILTAG COMMAND ENDING");
        System.out.println("Interrupted: " + interrupted);
        System.out.println("===========================================");
        driveSubsystem.stopAllMotors();
        SmartDashboard.putString("AlignToAprilTag/Status", interrupted ? "Interrupted" : "Finished");
    }
}
