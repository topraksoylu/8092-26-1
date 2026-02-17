package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    
    public static class MotorConstants {
        public static final int REAR_LEFT_MOTOR_ID = 4;
        public static final int FRONT_LEFT_MOTOR_ID = 1;
        public static final int REAR_RIGHT_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_MOTOR_ID = 3;

        public static final boolean REAR_LEFT_MOTOR_INVERTED = true;
        public static final boolean FRONT_LEFT_MOTOR_INVERTED = true;
        public static final boolean REAR_RIGHT_MOTOR_INVERTED = false;
        public static final boolean FRONT_RIGHT_MOTOR_INVERTED = false;

        // Intake subsystem
        public static final int INTAKE_MOTOR_ID = 5;
        public static final boolean INTAKE_MOTOR_INVERTED = false;

    // Toggle creating non-drive SparkMax motor instances. Set to false to
    // disable intake/shooter/turret motor controllers (useful for testing
    // or to avoid CAN conflicts). When false, the subsystems will no-op
    // motor calls but remain available to commands.
    public static final boolean ENABLE_NON_DRIVE_MOTORS = false;

        // Turret subsystem
        public static final int TURRET_MOTOR_ID = 6;
        public static final boolean TURRET_MOTOR_INVERTED = false;

        // Shooter subsystem
        public static final int LEFT_SHOOTER_MOTOR_ID = 7;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 8;
        public static final int TOP_SHOOTER_MOTOR_ID = 9;
        public static final boolean LEFT_SHOOTER_INVERTED = false;
        public static final boolean RIGHT_SHOOTER_INVERTED = true;
        public static final boolean TOP_SHOOTER_INVERTED = false;
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

        public static final double WHEEL_DIAMETER = 0.1524;
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public static final double GEARBOX_RATIO = 8.14;

        public static final double MAX_SPEED_METERS_PER_SECOND = 3.25; // 50% of 4.5
        
    }

    public static final class DriveControlConstants {
        // Teleop control tuning for 4-wheel mecanum drive
        public static final double DEADBAND = 0.05;
        public static final double TRANSLATION_SCALE = 0.90;
        public static final double ROTATION_SCALE = 0.75;
        public static final double TRANSLATION_SLEW_RATE = 3.5; // units/sec
        public static final double ROTATION_SLEW_RATE = 4.0; // units/sec
    }

    public static final class NavXTestConstants {
        public static final double TEST_TURN_OUTPUT = 0.20;
        public static final double TURN_PHASE_TIMEOUT_SEC = 1.5;
        public static final double MIN_EXPECTED_DELTA_DEG = 10.0;
        public static final double MAX_ABS_YAW_JUMP_DEG = 120.0;
        public static final long ZERO_SETTLE_MS = 250;
    }

    public static final class ModuleConstants {
        // Intake constants
        public static final double INTAKE_SPEED = 0.8;

        // Turret constants
        public static final double TURRET_SPEED = 0.3;
        public static final double TURRET_MAX_ANGLE = 180.0;
        public static final double TURRET_MIN_ANGLE = -180.0;

        // Shooter constants
        public static final double SHOOTER_SPEED = 0.9;

        // Vision constants
        public static final String LIMELIGHT_NAME = "limelight";
        public static final double TARGET_HEIGHT = 2.64; // Height of April Tag target in meters
        public static final double LIMELIGHT_HEIGHT = 0.5; // Height of Limelight in meters
        public static final double LIMELIGHT_ANGLE = 15.0; // Angle of Limelight in degrees
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        // Driver axes (Driver Station USB tab order)
        public static final int DRIVER_X_AXIS = 1;
        public static final int DRIVER_Y_AXIS = 0;
        public static final int DRIVER_Z_AXIS = 2;

        // Driver buttons (DualShock 4 / CUH-ZCT2U mapping in WPILib)
        public static final int INTAKE_BUTTON = 6; // R1
        public static final int ALIGN_BUTTON = 2; // Cross (X)
        public static final int TURRET_TRACK_BUTTON = 3; // Circle

        // Operator buttons
        public static final int SHOOT_BUTTON = 1;
    }
}
