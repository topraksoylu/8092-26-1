package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

/**
 * Rotation-only command:
 * - Rotates up to 360 degrees while searching for a target tag.
 * - If target AprilTag is seen, rotates to center it in view (tx ~= 0).
 * - Stops once centered or after a full 360-degree scan.
 */
public class RotateToAprilTag360Command extends Command {
    // Slow rotation profile (aligned with button-6 slow behavior)
    private static final double SCAN_SPEED = 0.025;       // 2x faster than 0.0125
    private static final double MAX_TRACK_SPEED = 0.025;  // keep tracking speed matched and predictable
    private static final double TX_TOLERANCE_DEG = 1.0;
    private static final double TX_KP = 0.004;
    private static final double TX_KD = 0.0008;
    private static final double MIN_VALID_AREA = 0.20;
    private static final double MAX_SCAN_DEGREES = 360.0;

    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final int targetTagId;

    private double previousYawDeg;
    private double accumulatedYawDeg;
    private boolean lockedOnTarget;
    private boolean scanComplete;
    private boolean startedWithTarget;
    private double scanDirection;
    private double lastErrorDeg;
    private double lastTimestampSec;
    private int lostTargetFrames;

    public RotateToAprilTag360Command(
            DriveSubsystem driveSubsystem,
            VisionSubsystem visionSubsystem,
            int targetTagId) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetTagId = targetTagId;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        previousYawDeg = driveSubsystem.getHeading().getDegrees();
        accumulatedYawDeg = 0.0;
        lockedOnTarget = false;
        scanComplete = false;
        startedWithTarget = visionSubsystem.hasTarget() && visionSubsystem.getTagId() == targetTagId;
        scanDirection = 1.0;
        lastErrorDeg = 0.0;
        lastTimestampSec = Timer.getFPGATimestamp();
        lostTargetFrames = 0;

        if (startedWithTarget) {
            double initialTx = visionSubsystem.getHorizontalOffset();
            if (Math.abs(initialTx) > TX_TOLERANCE_DEG) {
                scanDirection = initialTx > 0 ? -1.0 : 1.0;
            }
        } else {
            scanComplete = true;
        }

        SmartDashboard.putString("Rotate360/Status", startedWithTarget ? "Tracking" : "Tag 1 not in sight");
        SmartDashboard.putNumber("Rotate360/TargetTag", targetTagId);
        SmartDashboard.putNumber("Rotate360/AccumulatedYawDeg", accumulatedYawDeg);
        SmartDashboard.putBoolean("Rotate360/Locked", false);
        SmartDashboard.putBoolean("Rotate360/StartedWithTarget", startedWithTarget);
    }

    @Override
    public void execute() {
        double currentYawDeg = driveSubsystem.getHeading().getDegrees();
        double deltaYaw = MathUtil.inputModulus(currentYawDeg - previousYawDeg, -180.0, 180.0);
        accumulatedYawDeg += Math.abs(deltaYaw);
        previousYawDeg = currentYawDeg;

        SmartDashboard.putNumber("Rotate360/AccumulatedYawDeg", accumulatedYawDeg);

        if (!startedWithTarget) {
            driveSubsystem.stopAllMotors();
            return;
        }

        boolean hasTarget = visionSubsystem.hasTarget();
        int seenTagId = visionSubsystem.getTagId();
        double targetArea = visionSubsystem.getTargetArea();

        if (hasTarget && seenTagId == targetTagId && targetArea >= MIN_VALID_AREA) {
            double tx = visionSubsystem.getHorizontalOffset();
            double nowSec = Timer.getFPGATimestamp();
            double dt = Math.max(1e-3, nowSec - lastTimestampSec);
            lastTimestampSec = nowSec;
            SmartDashboard.putNumber("Rotate360/TX", tx);
            SmartDashboard.putNumber("Rotate360/Area", targetArea);

            if (Math.abs(tx) <= TX_TOLERANCE_DEG) {
                lockedOnTarget = true;
                driveSubsystem.stopAllMotors();
                SmartDashboard.putString("Rotate360/Status", "Locked");
                SmartDashboard.putBoolean("Rotate360/Locked", true);
                return;
            }

            // Robot-specific sign: negative tx must rotate toward the target (not away).
            double errorDeg = tx;
            double derivativeDegPerSec = (errorDeg - lastErrorDeg) / dt;
            double rawCmd = TX_KP * errorDeg + TX_KD * derivativeDegPerSec;
            double rotationCmd = MathUtil.clamp(rawCmd, -MAX_TRACK_SPEED, MAX_TRACK_SPEED);
            lastErrorDeg = errorDeg;
            scanDirection = Math.signum(rotationCmd) == 0.0 ? scanDirection : Math.signum(rotationCmd);
            lostTargetFrames = 0;
            driveSubsystem.drive(0.0, 0.0, rotationCmd);
            SmartDashboard.putString("Rotate360/Status", "Tracking");
            return;
        }

        // Prevent derivative spikes when target quality is poor/lost.
        lastErrorDeg = 0.0;
        lastTimestampSec = Timer.getFPGATimestamp();
        lostTargetFrames++;
        if (lostTargetFrames == 8) {
            scanDirection *= -1.0; // one-time reverse to reacquire if we started turning the wrong way
        }

        if (accumulatedYawDeg >= MAX_SCAN_DEGREES) {
            scanComplete = true;
            driveSubsystem.stopAllMotors();
            SmartDashboard.putString("Rotate360/Status", "Scan complete - tag not found");
            return;
        }

        // Tag temporarily lost: continue slow rotation in the last useful direction.
        driveSubsystem.drive(0.0, 0.0, SCAN_SPEED * scanDirection);
        SmartDashboard.putString("Rotate360/Status", "Scanning");
    }

    @Override
    public boolean isFinished() {
        return lockedOnTarget || scanComplete;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopAllMotors();
        if (interrupted) {
            SmartDashboard.putString("Rotate360/Status", "Interrupted");
        }
    }
}
