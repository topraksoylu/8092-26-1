
package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.ModulSabitleri;
import frc.robot.Sabitler.MotorSabitleri;

public class ShooterSubsystem extends SubsystemBase {
    private SparkMax leftShooterMotor;
    private SparkMax rightShooterMotor;
    private SparkMax topShooterMotor;
    private double lastCommandedSpeed = 0.0;

    public ShooterSubsystem() {
        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            leftShooterMotor = new SparkMax(MotorSabitleri.SOL_ATICI_MOTOR_ID, MotorType.kBrushless);
            rightShooterMotor = new SparkMax(MotorSabitleri.SAG_ATICI_MOTOR_ID, MotorType.kBrushless);
            topShooterMotor = new SparkMax(MotorSabitleri.UST_ATICI_MOTOR_ID, MotorType.kBrushless);

            SparkMaxConfig leftConfig = new SparkMaxConfig();
            leftConfig.inverted(MotorSabitleri.SOL_ATICI_TERS);
            leftShooterMotor.configure(leftConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

            SparkMaxConfig rightConfig = new SparkMaxConfig();
            rightConfig.inverted(MotorSabitleri.SAG_ATICI_TERS);
            rightShooterMotor.configure(rightConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

            SparkMaxConfig topConfig = new SparkMaxConfig();
            topConfig.inverted(MotorSabitleri.UST_ATICI_TERS);
            topShooterMotor.configure(topConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        } else {
            leftShooterMotor = null;
            rightShooterMotor = null;
            topShooterMotor = null;
        }
    }

    public void shoot() {
        lastCommandedSpeed = ModulSabitleri.ATICI_HIZI;
        if (leftShooterMotor != null) leftShooterMotor.set(ModulSabitleri.ATICI_HIZI);
        if (rightShooterMotor != null) rightShooterMotor.set(ModulSabitleri.ATICI_HIZI);
        if (topShooterMotor != null) topShooterMotor.set(ModulSabitleri.ATICI_HIZI);
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
}
