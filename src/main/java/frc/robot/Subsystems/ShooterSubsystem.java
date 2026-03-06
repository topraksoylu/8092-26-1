
package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Simulation.SimpleMotorSimulator;

public class ShooterSubsystem extends SubsystemBase {
    private SparkMax leftShooterMotor;
    private SparkMax rightShooterMotor;
    private SparkMax topShooterMotor;
    private SimpleMotorSimulator leftShooterSim;
    private SimpleMotorSimulator rightShooterSim;
    private SimpleMotorSimulator topShooterSim;
    private double lastCommandedSpeed = 0.0;

    // Mechanism2D visualization
    private Mechanism2d shooterMech2d;
    private MechanismLigament2d leftFlywheel;
    private MechanismLigament2d rightFlywheel;
    private MechanismLigament2d topFlywheel;

    public ShooterSubsystem() {
        if (MotorConstants.ENABLE_NON_DRIVE_MOTORS) {
            leftShooterMotor = new SparkMax(MotorConstants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
            rightShooterMotor = new SparkMax(MotorConstants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
            topShooterMotor = new SparkMax(MotorConstants.TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);

            SparkMaxConfig leftConfig = new SparkMaxConfig();
            leftConfig.inverted(MotorConstants.LEFT_SHOOTER_INVERTED);
            leftShooterMotor.configure(leftConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

            SparkMaxConfig rightConfig = new SparkMaxConfig();
            rightConfig.inverted(MotorConstants.RIGHT_SHOOTER_INVERTED);
            rightShooterMotor.configure(rightConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

            SparkMaxConfig topConfig = new SparkMaxConfig();
            topConfig.inverted(MotorConstants.TOP_SHOOTER_INVERTED);
            topShooterMotor.configure(topConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        } else {
            leftShooterMotor = null;
            rightShooterMotor = null;
            topShooterMotor = null;
        }

        // Create simulators for simulation mode
        if (!RobotBase.isReal()) {
            double maxVelocity = ModuleConstants.SHOOTER_SPEED * 20.0; // Approximate max velocity
            leftShooterSim = new SimpleMotorSimulator(maxVelocity, 15.0);
            rightShooterSim = new SimpleMotorSimulator(maxVelocity, 15.0);
            topShooterSim = new SimpleMotorSimulator(maxVelocity, 15.0);

            // Create Mechanism2D visualization
            shooterMech2d = new Mechanism2d(10, 10);
            MechanismRoot2d root = shooterMech2d.getRoot("ShooterBase", 5, 5);

            leftFlywheel = new MechanismLigament2d("Left", 3, 180);
            rightFlywheel = new MechanismLigament2d("Right", 3, 0);
            topFlywheel = new MechanismLigament2d("Top", 3, 90);

            root.append(leftFlywheel);
            root.append(rightFlywheel);
            root.append(topFlywheel);

            SmartDashboard.putData("Shooter Mechanism", shooterMech2d);
        }
    }

    @Override
    public void periodic() {
        // Update simulators
        if (!RobotBase.isReal()) {
            if (leftShooterSim != null) leftShooterSim.update(0.02, lastCommandedSpeed);
            if (rightShooterSim != null) rightShooterSim.update(0.02, lastCommandedSpeed);
            if (topShooterSim != null) topShooterSim.update(0.02, lastCommandedSpeed);

            // Update Mechanism2D visualization
            if (shooterMech2d != null) {
                // Visualize flywheel speeds by changing length
                double leftSpeed = leftShooterSim.getVelocity();
                double rightSpeed = rightShooterSim.getVelocity();
                double topSpeed = topShooterSim.getVelocity();

                leftFlywheel.setLength(3 + leftSpeed * 0.1);
                rightFlywheel.setLength(3 + rightSpeed * 0.1);
                topFlywheel.setLength(3 + topSpeed * 0.1);

                // Change color if spinning
                Color8Bit activeColor = lastCommandedSpeed > 0 ?
                    new Color8Bit(0, 255, 0) : new Color8Bit(128, 128, 128);  // Green or Gray
                leftFlywheel.setColor(activeColor);
                rightFlywheel.setColor(activeColor);
                topFlywheel.setColor(activeColor);
            }
        }
    }

    public void shoot() {
        lastCommandedSpeed = ModuleConstants.SHOOTER_SPEED;
        if (leftShooterMotor != null) leftShooterMotor.set(ModuleConstants.SHOOTER_SPEED);
        if (rightShooterMotor != null) rightShooterMotor.set(ModuleConstants.SHOOTER_SPEED);
        if (topShooterMotor != null) topShooterMotor.set(ModuleConstants.SHOOTER_SPEED);
    }

    public void stop() {
        lastCommandedSpeed = 0.0;
        if (leftShooterMotor != null) leftShooterMotor.set(0);
        if (rightShooterMotor != null) rightShooterMotor.set(0);
        if (topShooterMotor != null) topShooterMotor.set(0);
    }

    public double getLastCommandedSpeed() {
        return lastCommandedSpeed;
    }

    /**
     * Simülasyon modunda motor hızlarını döndürür.
     * Sadece simülasyon için kullanılır.
     *
     * @return Simüle edilmiş motor hızları dizisi [left, right, top]
     */
    public double[] getSimulatedVelocities() {
        if (leftShooterSim != null && rightShooterSim != null && topShooterSim != null) {
            return new double[] {
                leftShooterSim.getVelocity(),
                rightShooterSim.getVelocity(),
                topShooterSim.getVelocity()
            };
        }
        return new double[] {0.0, 0.0, 0.0};
    }
}
