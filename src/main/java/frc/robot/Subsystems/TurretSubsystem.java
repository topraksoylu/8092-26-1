package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.MotorSabitleri;

public class TurretSubsystem extends SubsystemBase {
    private SparkMax turretMotor;
    private RelativeEncoder turretEncoder;
    private double lastCommandedSpeed = 0.0;

    public TurretSubsystem() {
        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            turretMotor = new SparkMax(MotorSabitleri.TARET_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(MotorSabitleri.TARET_MOTOR_TERS);
            turretMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

            turretEncoder = turretMotor.getEncoder();
        } else {
            turretMotor = null;
            turretEncoder = null;
        }
    }

    public void rotate(double speed) {
        lastCommandedSpeed = speed;
        if (turretMotor != null) {
            turretMotor.set(speed);
        }
    }

    public void stop() {
        lastCommandedSpeed = 0.0;
        if (turretMotor != null) {
            turretMotor.set(0);
        }
    }

    public double getAngle() {
        // Assuming encoder gives position in rotations, convert to degrees
        if (turretEncoder != null) {
            return turretEncoder.getPosition() * 360.0;
        }
        return 0.0;
    }

    public void setAngle(double angle) {
        // Simple proportional control - in real implementation, use PID
        double currentAngle = getAngle();
        double error = angle - currentAngle;
        double speed = error * 0.01; // Tune this
        rotate(speed);
    }

    public double getLastCommandedSpeed() {
        return lastCommandedSpeed;
    }
}
