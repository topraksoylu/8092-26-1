package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;

public final class SurusMatematigi {
    private SurusMatematigi() {}

    public static double encoderPositionToMeters(double encoderPosition, double gearboxRatio, double wheelCircumference) {
        return encoderPosition / gearboxRatio * wheelCircumference;
    }

    public static double encoderVelocityRpmToMetersPerSecond(double encoderVelocityRpm, double gearboxRatio, double wheelCircumference) {
        return encoderVelocityRpm / gearboxRatio * wheelCircumference / 60.0;
    }

    public static double wrapDeltaDegrees(double startDeg, double endDeg) {
        return MathUtil.inputModulus(endDeg - startDeg, -180.0, 180.0);
    }
}
