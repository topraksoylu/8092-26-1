package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Simulation.SimpleMotorSimulator;

public class TurretSubsystem extends SubsystemBase {
    private SparkMax turretMotor;
    private RelativeEncoder turretEncoder;
    private SimpleMotorSimulator turretSim;
    private double lastCommandedSpeed = 0.0;

    // Mechanism2D visualization
    private Mechanism2d turretMech2d;
    private MechanismLigament2d turretArm;

    public TurretSubsystem() {
        if (MotorConstants.ENABLE_NON_DRIVE_MOTORS) {
            turretMotor = new SparkMax(MotorConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(MotorConstants.TURRET_MOTOR_INVERTED);
            turretMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

            turretEncoder = turretMotor.getEncoder();
        } else {
            turretMotor = null;
            turretEncoder = null;
        }

        // Create simulator for simulation mode
        if (!RobotBase.isReal()) {
            // Turret rotates in degrees - max 360 degrees/sec, 720 deg/sec^2 acceleration
            double maxVelocity = 360.0; // degrees per second
            turretSim = new SimpleMotorSimulator(maxVelocity, 720.0);

            // Create Mechanism2D visualization
            turretMech2d = new Mechanism2d(10, 10);
            turretArm = new MechanismLigament2d("Turret", 4, 0);
            turretMech2d.getRoot("TurretBase", 5, 5).append(turretArm);
            SmartDashboard.putData("Turret Mechanism", turretMech2d);
        }
    }

    @Override
    public void periodic() {
        // Update simulator
        if (!RobotBase.isReal() && turretSim != null) {
            turretSim.update(0.02, lastCommandedSpeed);

            // Update Mechanism2D visualization
            if (turretMech2d != null) {
                // Show actual turret angle
                double currentAngle = turretSim.getPosition();
                turretArm.setAngle(currentAngle);

                // Color based on motion
                if (Math.abs(lastCommandedSpeed) > 0.01) {
                    turretArm.setColor(new Color8Bit(0, 255, 0));  // Green - Moving
                } else {
                    turretArm.setColor(new Color8Bit(128, 128, 128));  // Gray - Stopped
                }
            }
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
        if (RobotBase.isReal() && turretEncoder != null) {
            // Assuming encoder gives position in rotations, convert to degrees
            return turretEncoder.getPosition() * 360.0;
        } else if (turretSim != null) {
            // Simülasyon - simüle edilmiş pozisyonu kullan (derece)
            return turretSim.getPosition();
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
