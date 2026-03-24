package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Sabitler.MotorSabitleri;

/**
 * Basit motor test komutları - teleop modda çalışır.
 * Her test motorları sırayla çalıştırır, böylece hangi motorun
 * hangi yönde döndüğünü görebilirsiniz.
 */
public class TestModes {

    /**
     * Tek bir motoru belirli bir hızda çalıştırır.
     * SmartDashboard'dan tetiklemek için kullanışlıdır.
     */
    public static class SingleMotorTest extends Command {
        private final SurusAltSistemi drive;
        private final int motorId;
        private final double speed;

        public SingleMotorTest(SurusAltSistemi drive, int motorId, double speed) {
            this.drive = drive;
            this.motorId = motorId;
            this.speed = speed;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            System.out.println("Motor Test: ID " + motorId + " hız " + speed);
        }

        @Override
        public void execute() {
            // CAN ID'ye göre motoru seç
            switch (motorId) {
                case MotorSabitleri.ON_SOL_MOTOR_ID:
                    drive.testSetFrontLeft(speed);
                    break;
                case MotorSabitleri.ON_SAG_MOTOR_ID:
                    drive.testSetFrontRight(speed);
                    break;
                case MotorSabitleri.ARKA_SOL_MOTOR_ID:
                    drive.testSetRearLeft(speed);
                    break;
                case MotorSabitleri.ARKA_SAG_MOTOR_ID:
                    drive.testSetRearRight(speed);
                    break;
                default:
                    System.out.println("Bilinmeyen motor ID: " + motorId);
            }
        }

        @Override
        public void end(boolean interrupted) {
            drive.tumMotorlariDurdur();
            System.out.println("Motor Test durduruldu: ID " + motorId);
        }

        @Override
        public boolean isFinished() {
            return false; // Manuel iptal edilene kadar çalış
        }
    }

    /**
     * Tüm motorları aynı yönde döndürür - robotun ileri gitmesi gerekir.
     * Motor yönlerinin doğru ayarlandığını kontrol etmek için.
     */
    public static class ForwardDriveTest extends Command {
        private final SurusAltSistemi drive;
        private final double speed;

        public ForwardDriveTest(SurusAltSistemi drive, double speed) {
            this.drive = drive;
            this.speed = speed;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            System.out.println("İleri sürüş testi başladı - hız: " + speed);
        }

        @Override
        public void execute() {
            drive.testSetFrontLeft(speed);
            drive.testSetFrontRight(speed);
            drive.testSetRearLeft(speed);
            drive.testSetRearRight(speed);
        }

        @Override
        public void end(boolean interrupted) {
            drive.tumMotorlariDurdur();
            System.out.println("İleri sürüş testi bitti");
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    /**
     * Robot saat yönünde döndürür (sağ dönüş).
     */
    public static class TurnRightTest extends Command {
        private final SurusAltSistemi drive;
        private final double speed;

        public TurnRightTest(SurusAltSistemi drive, double speed) {
            this.drive = drive;
            this.speed = speed;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            System.out.println("Sağ dönüş testi başladı - hız: " + speed);
        }

        @Override
        public void execute() {
            // Sağ taraf ileri, sol taraf geri = saat yönünde dönüş
            drive.testSetFrontLeft(-speed);
            drive.testSetFrontRight(speed);
            drive.testSetRearLeft(-speed);
            drive.testSetRearRight(speed);
        }

        @Override
        public void end(boolean interrupted) {
            drive.tumMotorlariDurdur();
            System.out.println("Sağ dönüş testi bitti");
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    /**
     * Robot saat yönünün tersine döndürür (sol dönüş).
     */
    public static class TurnLeftTest extends Command {
        private final SurusAltSistemi drive;
        private final double speed;

        public TurnLeftTest(SurusAltSistemi drive, double speed) {
            this.drive = drive;
            this.speed = speed;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            System.out.println("Sol dönüş testi başladı - hız: " + speed);
        }

        @Override
        public void execute() {
            // Sol taraf ileri, sağ taraf geri = saat yönünün tersine dönüş
            drive.testSetFrontLeft(speed);
            drive.testSetFrontRight(-speed);
            drive.testSetRearLeft(speed);
            drive.testSetRearRight(-speed);
        }

        @Override
        public void end(boolean interrupted) {
            drive.tumMotorlariDurdur();
            System.out.println("Sol dönüş testi bitti");
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    /**
     * Mecanum strafe testi - robot sağa kaymalı.
     */
    public static class StrafeRightTest extends Command {
        private final SurusAltSistemi drive;
        private final double speed;

        public StrafeRightTest(SurusAltSistemi drive, double speed) {
            this.drive = drive;
            this.speed = speed;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            System.out.println("Sağa kayma testi başladı - hız: " + speed);
        }

        @Override
        public void execute() {
            // Mecanum sağa kayma için
            // Ön sol ve arka sağ: ileri
            // Ön sağ ve arka sol: geri
            drive.testSetFrontLeft(speed);
            drive.testSetFrontRight(-speed);
            drive.testSetRearLeft(-speed);
            drive.testSetRearRight(speed);
        }

        @Override
        public void end(boolean interrupted) {
            drive.tumMotorlariDurdur();
            System.out.println("Sağa kayma testi bitti");
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    /**
     * Joystick eksenlerini konsola yazdırır - calibration için.
     */
    public static class JoystickAxisTest extends Command {
        private final java.util.function.DoubleSupplier yAxis;
        private final java.util.function.DoubleSupplier xAxis;
        private final java.util.function.DoubleSupplier zAxis;
        private int printCounter = 0;

        public JoystickAxisTest(
                java.util.function.DoubleSupplier yAxis,
                java.util.function.DoubleSupplier xAxis,
                java.util.function.DoubleSupplier zAxis) {
            this.yAxis = yAxis;
            this.xAxis = xAxis;
            this.zAxis = zAxis;
        }

        @Override
        public void initialize() {
            System.out.println("=== Joystick Eksen Testi Başladı ===");
            System.out.println("Joystick'i hareket ettirin - değerler konsola yazılacak");
        }

        @Override
        public void execute() {
            // Her 10 döngüde bir yazdır (flood'u önlemek için)
            printCounter++;
            if (printCounter % 10 == 0) {
                System.out.printf("Y Axis: %+.2f | X Axis: %+.2f | Z Axis: %+.2f%n",
                    yAxis.getAsDouble(),
                    xAxis.getAsDouble(),
                    zAxis.getAsDouble()
                );
            }
        }

        @Override
        public void end(boolean interrupted) {
            System.out.println("=== Joystick Eksen Testi Bitti ===");
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }
}
