package frc.robot.Subsystems;

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

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;
    private SimpleMotorSimulator intakeSim;
    private double lastCommandedSpeed = 0.0;

    // Mechanism2D visualization
    private Mechanism2d intakeMech2d;
    private MechanismLigament2d intakeArm;
    private MechanismLigament2d intakeRoller;

    public IntakeSubsystem() {
        if (MotorConstants.ENABLE_NON_DRIVE_MOTORS) {
            intakeMotor = new SparkMax(MotorConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(MotorConstants.INTAKE_MOTOR_INVERTED);
            intakeMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        } else {
            // Stub out motor to avoid creating CAN devices when not needed
            intakeMotor = null;
        }

        // Create simulator for simulation mode
        if (!RobotBase.isReal()) {
            // Max velocity: 6000 RPM (NEO free speed) * wheel circumference / 60 / gear ratio
            // Simplified: approximate based on intake speed
            double maxVelocity = ModuleConstants.INTAKE_SPEED * 10.0; // m/s equivalent
            intakeSim = new SimpleMotorSimulator(maxVelocity, 10.0);

            // Create Mechanism2D visualization
            intakeMech2d = new Mechanism2d(10, 10);
            intakeArm = new MechanismLigament2d("Intake Arm", 5, 0);
            intakeRoller = new MechanismLigament2d("Roller", 2, 0);
            intakeArm.append(intakeRoller);
            intakeMech2d.getRoot("IntakeBase", 5, 5).append(intakeArm);
            SmartDashboard.putData("Intake Mechanism", intakeMech2d);
        }
    }

    @Override
    public void periodic() {
        // Update simulator
        if (!RobotBase.isReal() && intakeSim != null) {
            intakeSim.update(0.02, lastCommandedSpeed);

            // Update Mechanism2D visualization
            if (intakeMech2d != null) {
                // Visualize roller speed by changing angle
                double rollerAngle = intakeSim.getVelocity() * 10;
                intakeRoller.setAngle(rollerAngle);

                // Change color based on state
                if (lastCommandedSpeed > 0) {
                    intakeArm.setColor(new Color8Bit(0, 255, 0));  // Green - Intaking
                } else if (lastCommandedSpeed < 0) {
                    intakeArm.setColor(new Color8Bit(255, 0, 0));  // Red - Outtaking
                } else {
                    intakeArm.setColor(new Color8Bit(128, 128, 128));  // Gray - Stopped
                }
            }
        }
    }

    public void intake() {
        lastCommandedSpeed = ModuleConstants.INTAKE_SPEED;
        if (intakeMotor != null) {
            intakeMotor.set(ModuleConstants.INTAKE_SPEED);
        }
    }

    public void outtake() {
        lastCommandedSpeed = -ModuleConstants.INTAKE_SPEED;
        if (intakeMotor != null) {
            intakeMotor.set(-ModuleConstants.INTAKE_SPEED);
        }
    }

    public void stop() {
        lastCommandedSpeed = 0.0;
        if (intakeMotor != null) {
            intakeMotor.set(0);
        }
    }

    public double getLastCommandedSpeed() {
        return lastCommandedSpeed;
    }

    /**
     * Simülasyon modunda motor hızını döndürür.
     * Sadece simülasyon için kullanılır.
     *
     * @return Simüle edilmiş motor hızı
     */
    public double getSimulatedVelocity() {
        if (intakeSim != null) {
            return intakeSim.getVelocity();
        }
        return 0.0;
    }
}
