package frc.robot.Subsystems;

public final class SurusMatematigi {
    private SurusMatematigi() {}

    public static double encoderPositionToMeters(double encoderPosition, double gearboxRatio, double wheelCircumference) {
        return encoderPosition / gearboxRatio * wheelCircumference;
    }

    public static double encoderVelocityRpmToMetersPerSecond(double encoderVelocityRpm, double gearboxRatio, double wheelCircumference) {
        return encoderVelocityRpm / gearboxRatio * wheelCircumference / 60.0;
    }
}
