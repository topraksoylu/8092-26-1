package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sabitler.ModulSabitleri;
import frc.robot.Sabitler.MotorSabitleri;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;
    private double lastCommandedSpeed = 0.0;

    public IntakeSubsystem() {
        if (MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN) {
            intakeMotor = new SparkMax(MotorSabitleri.ALIM_MOTOR_ID, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(MotorSabitleri.ALIM_MOTOR_TERS);
            intakeMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
        } else {
            // Stub out motor to avoid creating CAN devices when not needed
            intakeMotor = null;
        }
    }

    public void intake() {
        lastCommandedSpeed = ModulSabitleri.ALIM_HIZI;
        if (intakeMotor != null) {
            intakeMotor.set(ModulSabitleri.ALIM_HIZI);
        }
    }

    public void outtake() {
        lastCommandedSpeed = -ModulSabitleri.ALIM_HIZI;
        if (intakeMotor != null) {
            intakeMotor.set(-ModulSabitleri.ALIM_HIZI);
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
}
