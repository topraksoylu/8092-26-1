package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.GorusAltSistemi;
import frc.robot.AprilTagSahaYerlesimi;

/**
 * Continuously tracks and follows an AprilTag.
 * Maintains a specified distance from the tag and faces it.
 * Runs continuously while button is held.
 */
public class AprilTagTakipKomutu extends Command {
    private final SurusAltSistemi SurusAltSistemi;
    private final GorusAltSistemi GorusAltSistemi;

    // Target configuration
    private final int targetTagId;
    private final double targetDistanceMeters;  // Distance to maintain from tag

    // PID controllers for tracking
    private final PIDController distanceController;  // Controls forward/back
    private final PIDController strafeController;     // Controls left/right
    private final PIDController thetaController;      // Controls rotation

    // Alliance configuration
    private boolean isRedAlliance;

    // Tracking state
    private int noTargetFrames = 0;
    private static final int NO_TARGET_TIMEOUT_FRAMES = 25;  // 0.5 seconds at 50Hz

    /**
     * Creates a new AprilTagTakipKomutu.
     *
     * @param SurusAltSistemi The drive subsystem to use
     * @param GorusAltSistemi The vision subsystem to use
     * @param targetTagId The AprilTag ID to track (1-16)
     * @param targetDistanceMeters Distance to maintain from tag (default: 1.5m)
     */
    public AprilTagTakipKomutu(
            SurusAltSistemi SurusAltSistemi,
            GorusAltSistemi GorusAltSistemi,
            int targetTagId,
            double targetDistanceMeters) {

        this.SurusAltSistemi = SurusAltSistemi;
        this.GorusAltSistemi = GorusAltSistemi;
        this.targetTagId = targetTagId;
        this.targetDistanceMeters = targetDistanceMeters;

        // Create PID controllers for tracking
        // These are tuned for smooth tracking behavior
        distanceController = new PIDController(1.5, 0, 0.1);   // Forward/back
        strafeController = new PIDController(1.5, 0, 0.1);      // Left/right
        thetaController = new PIDController(3.0, 0, 0.1);       // Rotation (faster)

        // Enable continuous input for theta (handles wraparound at -180/180)
        thetaController.enableContinuousInput(-180, 180);

        // Set tolerances (looser than alignment for smooth tracking)
        distanceController.setTolerance(0.2);  // 20cm tolerance
        strafeController.setTolerance(0.2);     // 20cm tolerance
        thetaController.setTolerance(10.0);     // 10 degree tolerance

        addRequirements(SurusAltSistemi);
    }

    @Override
    public void initialize() {
        System.out.println("===========================================");
        System.out.println("TRACK APRILTAG COMMAND STARTED");
        System.out.println("Target Tag ID: " + targetTagId);
        System.out.println("Tracking Distance: " + targetDistanceMeters + " meters");
        System.out.println("Hold button to track, release to stop");
        System.out.println("===========================================");

        // Determine alliance at command start
        isRedAlliance = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        noTargetFrames = 0;

        SmartDashboard.putString("TrackAprilTag/Status", "Initializing");
        SmartDashboard.putNumber("TrackAprilTag/TargetTag", targetTagId);
        SmartDashboard.putNumber("TrackAprilTag/TargetDistance", targetDistanceMeters);
        SmartDashboard.putBoolean("TrackAprilTag/IsRed", isRedAlliance);
    }

    @Override
    public void execute() {
        // Get vision result
        GorusAltSistemi.VisionResult result = GorusAltSistemi.aprilTagdanRobotPozuAl();

        SmartDashboard.putBoolean("TrackAprilTag/ResultValid", result.valid);
        SmartDashboard.putNumber("TrackAprilTag/ResultTagID", result.tagId);

        // If no valid vision target, stop and wait
        if (!result.valid) {
            noTargetFrames++;
            SmartDashboard.putString("TrackAprilTag/Status", "No target (" + noTargetFrames + ")");

            // Stop robot after a few frames
            if (noTargetFrames > 5) {
                SurusAltSistemi.tumMotorlariDurdur();
            }
            return;
        }

        // Reset timeout counter
        noTargetFrames = 0;

        // Check if we're seeing the right tag
        if (result.tagId != targetTagId) {
            SmartDashboard.putString("TrackAprilTag/Status", "Wrong tag: " + result.tagId);
            SurusAltSistemi.tumMotorlariDurdur();
            return;
        }

        // Get tag pose from field layout
        Pose2d targetPose = AprilTagSahaYerlesimi.etiketPozunuAl(targetTagId, isRedAlliance);
        if (targetPose == null) {
            SmartDashboard.putString("TrackAprilTag/Status", "Tag pose not found");
            SurusAltSistemi.tumMotorlariDurdur();
            return;
        }

        // Current robot pose from vision
        Pose2d currentPose = result.robotPose;

        // Calculate desired position (in front of tag, facing tag)
        double tagYaw = targetPose.getRotation().getRadians();
        double targetX = targetPose.getX() - Math.cos(tagYaw) * targetDistanceMeters;
        double targetY = targetPose.getY() - Math.sin(tagYaw) * targetDistanceMeters;

        // Calculate errors
        double xError = targetX - currentPose.getX();  // Forward error
        double yError = targetY - currentPose.getY();  // Strafe error

        // For heading, we want to face the tag
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double desiredYaw = Math.atan2(dy, dx);

        double currentYaw = currentPose.getRotation().getRadians();
        double thetaError = desiredYaw - currentYaw;

        // Normalize theta error to -PI to PI
        while (thetaError > Math.PI) thetaError -= 2 * Math.PI;
        while (thetaError < -Math.PI) thetaError += 2 * Math.PI;

        // Update SmartDashboard with debug values
        SmartDashboard.putNumber("TrackAprilTag/XError", xError);
        SmartDashboard.putNumber("TrackAprilTag/YError", yError);
        SmartDashboard.putNumber("TrackAprilTag/YawError", Math.toDegrees(thetaError));
        SmartDashboard.putNumber("TrackAprilTag/RobotX", currentPose.getX());
        SmartDashboard.putNumber("TrackAprilTag/RobotY", currentPose.getY());

        // Calculate drive outputs using PID
        double xSpeed = distanceController.calculate(xError, 0);
        double ySpeed = strafeController.calculate(yError, 0);
        double thetaSpeed = thetaController.calculate(Math.toDegrees(thetaError), 0);

        SmartDashboard.putNumber("TrackAprilTag/XSpeed", xSpeed);
        SmartDashboard.putNumber("TrackAprilTag/YSpeed", ySpeed);
        SmartDashboard.putNumber("TrackAprilTag/ThetaSpeed", thetaSpeed);

        // Clamp outputs to prevent excessive speed
        xSpeed = Math.max(-0.5, Math.min(0.5, xSpeed));
        ySpeed = Math.max(-0.5, Math.min(0.5, ySpeed));
        thetaSpeed = Math.max(-0.5, Math.min(0.5, thetaSpeed));

        // Drive the robot
        SurusAltSistemi.drive(ySpeed, xSpeed, thetaSpeed);

        SmartDashboard.putString("TrackAprilTag/Status", "Tracking");
    }

    @Override
    public boolean isFinished() {
        // This command never finishes on its own
        // It runs until the button is released
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("===========================================");
        System.out.println("TRACK APRILTAG COMMAND ENDING");
        System.out.println("Interrupted: " + interrupted);
        System.out.println("===========================================");

        SurusAltSistemi.tumMotorlariDurdur();
        SmartDashboard.putString("TrackAprilTag/Status", interrupted ? "Interrupted" : "Stopped");
    }
}

