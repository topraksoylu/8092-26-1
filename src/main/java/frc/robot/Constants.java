package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    
    public static class MotorConstants {
        // NEO Brushless Motor V1.1 Specifications:
        // - Free Speed: 5676 RPM
        // - Stall Torque: 2.6 Nm
        // - Stall Current: 105 A
        // - Free Current: 1.8 A

        // Drive motors (verified physical mapping)
        // CAN ID 1 = Rear Right, CAN ID 2 = Front Left, CAN ID 3 = Rear Left, CAN ID 4 = Front Right
        public static final int REAR_LEFT_MOTOR_ID = 3;      // CAN ID 3 = Rear Left (Arka Sol)
        public static final int FRONT_LEFT_MOTOR_ID = 2;     // CAN ID 2 = Front Left (Ön Sol)
        public static final int REAR_RIGHT_MOTOR_ID = 1;     // CAN ID 1 = Rear Right (Arka Sağ)
        public static final int FRONT_RIGHT_MOTOR_ID = 4;    // CAN ID 4 = Front Right (Ön Sağ)

        // Mecanum drive: RIGHT side motors must be inverted relative to LEFT side
        // This is due to roller orientation differences between left and right wheels
        public static final boolean REAR_LEFT_MOTOR_INVERTED = false;     // Left side
        public static final boolean FRONT_LEFT_MOTOR_INVERTED = false;    // Left side
        public static final boolean REAR_RIGHT_MOTOR_INVERTED = true;     // Right side - INVERTED!
        public static final boolean FRONT_RIGHT_MOTOR_INVERTED = true;    // Right side - INVERTED!

        // Smart current limiting for NEO motors (prevents breaker trips)
        public static final int DRIVE_MOTOR_STALL_CURRENT_LIMIT = 60;  // Amps (NEO stall is 105A)
        public static final int DRIVE_MOTOR_FREE_CURRENT_LIMIT = 40;   // Amps
        public static final int DRIVE_MOTOR_CURRENT_LIMIT_THRESHOLD = 40;  // RPM

        public static final boolean ENABLE_NON_DRIVE_MOTORS = false;

        // Intake motor
        public static final int INTAKE_MOTOR_ID = 5;
        public static final boolean INTAKE_MOTOR_INVERTED = false;

        // Shooter motors
        public static final int LEFT_SHOOTER_MOTOR_ID = 6;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 7;
        public static final int TOP_SHOOTER_MOTOR_ID = 8;

        public static final boolean LEFT_SHOOTER_INVERTED = false;
        public static final boolean RIGHT_SHOOTER_INVERTED = true;
        public static final boolean TOP_SHOOTER_INVERTED = false;

        // Turret motor
        public static final int TURRET_MOTOR_ID = 9;
        public static final boolean TURRET_MOTOR_INVERTED = false;
    }

    public static class DriveConstants {

        public static final double TRACK_WIDTH = 0.52;
        public static final double WHEEL_BASE = 0.575;

        public static final Translation2d[] WHEEL_POSITIONS = {
            new Translation2d( WHEEL_BASE / 2,  TRACK_WIDTH / 2),   // Front Left
            new Translation2d( WHEEL_BASE / 2, -TRACK_WIDTH / 2),   // Front Right
            new Translation2d(-WHEEL_BASE / 2,  TRACK_WIDTH / 2),   // Rear Left
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)    // Rear Right
        };

        public static final double WHEEL_DIAMETER = 0.1524;  // 6 inch AndyMark Mecanum wheels
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public static final double GEARBOX_RATIO = 12.75;  // Toughbox Mini Classic 12.75:1

        // Max speed calculation for NEO with 12.75:1 gearing:
        // NEO free speed: 5676 RPM
        // After gearbox: 5676 / 12.75 = 445 RPM
        // Wheel speed: 445 / 60 * π * 0.1524 = 3.55 m/s theoretical
        // Conservative estimate with losses: ~3.0 m/s
        public static final double MAX_SPEED_METERS_PER_SECOND = 3.0;

    }

    public static class DriveControlConstants {
        // Optimized for AndyMark 6" Mecanum wheels (80a durometer rollers)
        public static final double DEADBAND = 0.08;  // Lower for precision with hard rollers

        // Axis sensitivity for fine control
        public static final double Y_AXIS_SENSITIVITY = 0.6;   // Forward/backward
        public static final double X_AXIS_SENSITIVITY = 0.65;  // Strafe (higher for mecanum efficiency loss)
        public static final double Z_AXIS_SENSITIVITY = 0.6;   // Rotation

        // Slew rate limiters for smooth acceleration (slower = smoother)
        public static final double TRANSLATION_SLEW_RATE = 1.0;  // Forward/strafe acceleration (reduced for smoothness)
        public static final double ROTATION_SLEW_RATE = 2.0;     // Rotation acceleration (slower for smooth turns)

        // Output scale factors (reduced for slower, smoother control)
        public static final double TRANSLATION_SCALE = 0.5;  // Forward/backward power (slower)
        public static final double STRAFE_SCALE = 0.6;       // Strafe power (slower but compensates for mecanum loss)
        public static final double ROTATION_SCALE = 0.5;     // Rotation power (much slower to prevent clunkiness)
    }

    public static class OIConstants {
        public static final int DRIVER_JOYSTICK_PORT = 0;
        public static final int OPERATOR_JOYSTICK_PORT = 1;

        // PS4 Physical Axes:
        // Axis 0 = Left Stick X (left/right) → controls strafe
        // Axis 1 = Left Stick Y (up/down) → controls forward/backward
        // Axis 2 = Right Stick X → controls rotation
        public static final int DRIVER_X_AXIS = 1;  // Left/right (strafe)
        public static final int DRIVER_Y_AXIS = 0;  // Forward/backward
        public static final int DRIVER_Z_AXIS = 2;  // Rotation
    }

    public static class ModuleConstants {
        public static final int INTAKE_MOTOR_ID = 5;
        public static final int SHOOTER_MOTOR_ID = 6;
        public static final int TURRET_MOTOR_ID = 7;

        public static final double INTAKE_SPEED = 0.5;
        public static final double SHOOTER_SPEED = 0.8;
        public static final double TURRET_SPEED = 0.3;
    }

    public static class VisionConstants {
        // Limelight configuration
        public static final String LIMELIGHT_NAME = "limelight";
        public static final int DESIRED_PIPELINE = 0;
        public static final int DESIRED_CAM_MODE = 0; // 0 = Vision processor
        public static final int DESIRED_LED_MODE = 0; // 0 = Pipeline control
        public static final int DESIRED_STREAM_MODE = 0; // 0 = Standard layout
        public static final double CONFIG_REAPPLY_INTERVAL_SEC = 1.0;

        // AprilTag FMAP source
        public static final String FMAP_SOURCE = "FRC2026_ANDYMARK.fmap";
        public static final int TOTAL_APRILTAGS = 32; // 16 per alliance side

        // Camera mounting (MEASURE ON ROBOT - using defaults for now)
        public static final double CAMERA_HEIGHT_METERS = 0.5;
        public static final double CAMERA_PITCH_RADIANS = Math.toRadians(25.0);

        // AprilTag settings
        public static final double APRILTAG_SIZE_METERS = 0.1651; // 6.5 inches (from FMAP)
        public static final double SPEAKER_TAG_HEIGHT_METERS = 0.889; // From FMAP Z coordinate

        // Vision measurement confidence
        public static final double VISION_MEASUREMENT_STD_DEV_SEC = 0.5;
        public static final double AMBIGUITY_THRESHOLD = 0.8; // TEMPORARY: Raised from 0.2 for testing (Limelight returning 1.0)

        // Pose update rate
        public static final double POSE_UPDATE_INTERVAL_SEC = 0.05; // 20Hz

        // Legacy constants for backward compatibility with VisionSubsystem
        public static final double TARGET_HEIGHT = SPEAKER_TAG_HEIGHT_METERS;
        public static final double LIMELIGHT_HEIGHT = CAMERA_HEIGHT_METERS;
        public static final double LIMELIGHT_ANGLE = Math.toDegrees(CAMERA_PITCH_RADIANS);
    }

    public static class NavXTestConstants {
        public static final long ZERO_SETTLE_MS = 500;
        public static final double TEST_TURN_OUTPUT = 0.3;
        public static final double MIN_EXPECTED_DELTA_DEG = 45.0;
        public static final double MAX_ABS_YAW_JUMP_DEG = 180.0;
        public static final double TURN_PHASE_TIMEOUT_SEC = 5.0;
    }

    public static class SimulationConstants {
        private SimulationConstants() {}

        public static class DrivetrainSimulation {
            // Robot fiziksel özellikleri
            public static final double ROBOT_MASS_KG = 45.0;  // Tipik FRC robot ağırlığı
            public static final double ROBOT_MOI = 6.0;       // Eylemsizlik momenti (kg*m^2)

            // Tekerlek özellikleri
            public static final double WHEEL_RADIUS = DriveConstants.WHEEL_DIAMETER / 2.0;
            public static final double WHEEL_MASS_KG = 0.5;

            // NEO Motor özellikleri
            public static final double NEO_FREE_SPEED_RPM = 5676;
            public static final double NEO_STALL_TORQUE_NM = 2.6;
            public static final double NEO_STALL_CURRENT_AMPS = 105;
            public static final double NEO_FREE_CURRENT_AMPS = 1.8;

            // Şanzıman
            public static final double GEAR_RATIO = DriveConstants.GEARBOX_RATIO;
            public static final int NUM_MOTORS_PER_SIDE = 2;

            // Ölçüm gürültü parametreleri
            public static final double ENCODER_STD_DEV = 0.001;
            public static final double GYRO_STD_DEV = 0.001;
        }
    }
}
