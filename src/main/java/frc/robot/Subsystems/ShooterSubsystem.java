
package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.ModulSabitleri;
import frc.robot.Sabitler.MotorSabitleri;

public class ShooterSubsystem extends SubsystemBase {
    private SparkMax shooterMotor;
    private double lastCommandedSpeed = 0.0;

    public ShooterSubsystem() {
        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            shooterMotor = new SparkMax(MotorSabitleri.ATICI_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(MotorSabitleri.ATICI_MOTOR_TERS);
            shooterMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        } else {
            shooterMotor = null;
        }
    }

    public void shoot() {
        lastCommandedSpeed = ModulSabitleri.ATICI_HIZI;
        if (shooterMotor != null) shooterMotor.set(ModulSabitleri.ATICI_HIZI);
    }

    public void stop() {
        lastCommandedSpeed = 0.0;
        if (shooterMotor != null) shooterMotor.set(0);
    }

    public double getLastCommandedSpeed() {
        return lastCommandedSpeed;
    }
}
