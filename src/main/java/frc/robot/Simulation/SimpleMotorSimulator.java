// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Simulation;

/**
 * Basit fizik tabanlı motor simülatörü.
 * Sürücü treni dışındaki motorlar (intake, shooter, turret vb.) için
 * hafif siklet simülasyon sağlar.
 */
public class SimpleMotorSimulator {
    private double position = 0.0;
    private double velocity = 0.0;
    private final double maxVelocity;
    private final double maxAcceleration;

    /**
     * Yeni bir basit motor simülatörü oluşturur.
     *
     * @param maxVelocity Maksimum hız (birim/saniye)
     * @param maxAcceleration Maksimum ivme (birim/saniye^2)
     */
    public SimpleMotorSimulator(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    /**
     * Simülasyonu günceller.
     *
     * @param dtSeconds Zaman adımı (saniye)
     * @param outputPercent Motor çıkışı (-1.0 ile 1.0 arası)
     */
    public void update(double dtSeconds, double outputPercent) {
        // Hedef hızı hesapla
        double targetVelocity = outputPercent * maxVelocity;

        // Basit ivme modeli
        double velocityError = targetVelocity - velocity;
        double acceleration = Math.copySign(
            Math.min(Math.abs(velocityError), maxAcceleration),
            velocityError
        );

        velocity += acceleration * dtSeconds;
        position += velocity * dtSeconds;
    }

    /**
     * Motor pozisyonunu döndürür.
     *
     * @return Pozisyon (birime bağlı olarak metre, derece vb.)
     */
    public double getPosition() {
        return position;
    }

    /**
     * Motor hızını döndürür.
     *
     * @return Hız (birim/saniye)
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * Simülasyonu sıfırlar.
     */
    public void reset() {
        position = 0.0;
        velocity = 0.0;
    }
}
