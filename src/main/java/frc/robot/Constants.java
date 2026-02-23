package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    
    public static class MotorConstants {
        public static final int REAR_LEFT_MOTOR_ID = 4;
        public static final int FRONT_LEFT_MOTOR_ID = 1;
        public static final int REAR_RIGHT_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_MOTOR_ID = 2;

        public static final boolean REAR_LEFT_MOTOR_INVERTED = true;
        public static final boolean FRONT_LEFT_MOTOR_INVERTED = false;
        public static final boolean REAR_RIGHT_MOTOR_INVERTED = true;
        public static final boolean FRONT_RIGHT_MOTOR_INVERTED = false;
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

        public static final double MAX_SPEED_METERS_PER_SECOND = 4.5; // TODO: Tune this value
        
    }
}
