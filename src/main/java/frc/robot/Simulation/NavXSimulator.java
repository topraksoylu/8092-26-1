// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Simulation;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/**
 * NavX jiroskop simülasyonu.
 *
 * <p>Bu sınıf, simülasyon modunda NavX jiroskopunun davranışını taklit eder.
 * Gerçek NavX donanımı yerine, fizik tabanlı bir simülasyon sağlar.
 *
 * <p>Özellikler:
 * <ul>
 *   <li>Yaw (başlık) açısı simülasyonu</li>
 *   <li>Pitch/roll simülasyonu (ivme dayalı)</li>
 *   <li>Yaw rate (dönüş hızı) hesaplama</li>
 *   <li>Bağlantı durumu simülasyonu</li>
 *   <li>WPILib GyroSim entegrasyonu</li>
 * </ul>
 */
public class NavXSimulator {
    private double yaw = 0.0;
    private double pitch = 0.0;
    private double roll = 0.0;
    private double yawRate = 0.0;  // degrees per second
    private double accelX = 0.0;
    private double accelY = 0.0;

    private double lastYaw = 0.0;
    private double lastUpdateTime = 0.0;

    // WPILib HAL simulation - simüle edilmiş NavX cihazı
    private SimDeviceSim simDevice;
    private SimDouble simYaw;
    private SimDouble simPitch;
    private SimDouble simRoll;
    private SimDouble simYawRate;
    private SimBoolean simConnected;

    /**
     * Yeni bir NavX simülatörü oluşturur.
     *
     * <p>WPILib HAL simülasyonunu kullanarak gerçek bir NavX cihazı gibi davranır.
     * Bu, DriveSubsystem'teki AHRS kodunun simülasyon modunda çalışmasını sağlar.
     */
    public NavXSimulator() {
        // Simüle edilmiş NavX cihazını oluştur (SPI üzerinden MXP portu)
        simDevice = new SimDeviceSim("navX-Sensor[0]");

        // Simüle edilmiş değerleri al
        simYaw = simDevice.getDouble("Yaw");
        simPitch = simDevice.getDouble("Pitch");
        simRoll = simDevice.getDouble("Roll");
        simYawRate = simDevice.getDouble("YawRate");
        simConnected = simDevice.getBoolean("Connected");

        // Başlangıç değerlerini ayarla
        if (simConnected != null) {
            simConnected.set(true);
        }
    }

    /**
     * Simülasyon durumunu günceller.
     *
     * @param dtSeconds Son güncellemeden beri geçen süre (saniye)
     * @param angularVelocity Dönüş hızı (derece/saniye)
     */
    public void update(double dtSeconds, double angularVelocity) {
        // Yaw rate hesapla
        yawRate = angularVelocity;

        // Yaw'ı güncelle (sürekli entegrasyon)
        yaw += yawRate * dtSeconds;

        // Yaw'ı [-180, 180] aralığına normalize et
        while (yaw > 180) yaw -= 360;
        while (yaw < -180) yaw += 360;

        // Pitch ve roll simülasyonu (basit yaklaşım)
        // Gerçek robotlarda ivme sensöründen gelir
        // Simülasyonda, dönüş sırasında hafif pitch/roll ekleriz
        pitch = Math.max(-15, Math.min(15, accelX * 2));
        roll = Math.max(-15, Math.min(15, accelY * 2));

        // WPILib HAL simülasyonunu güncelle
        updateHalSimulation();
    }

    /**
     * Simülasyon durumunu ivme verileriyle günceller.
     *
     * @param dtSeconds Son güncellemeden beri geçen süre (saniye)
     * @param angularVelocity Dönüş hızı (derece/saniye)
     * @param accelX X ekseninde ivme (m/s²)
     * @param accelY Y ekseninde ivme (m/s²)
     */
    public void update(double dtSeconds, double angularVelocity, double accelX, double accelY) {
        this.accelX = accelX;
        this.accelY = accelY;
        update(dtSeconds, angularVelocity);
    }

    /**
     * WPILib HAL simülasyonunu günceller.
     *
     * <p>Bu, AHRS sınıfının simüle edilmiş değerleri okumasını sağlar.
     */
    private void updateHalSimulation() {
        if (simYaw != null) {
            simYaw.set(yaw);
        }
        if (simPitch != null) {
            simPitch.set(pitch);
        }
        if (simRoll != null) {
            simRoll.set(roll);
        }
        if (simYawRate != null) {
            simYawRate.set(yawRate);
        }
    }

    /**
     * Mevcut yaw (başlık) açısını döndürür.
     *
     * @return Yaw açısı (derece), [-180, 180] aralığında
     */
    public double getYaw() {
        return yaw;
    }

    /**
     * Mevcut pitch açısını döndürür.
     *
     * @return Pitch açısı (derece)
     */
    public double getPitch() {
        return pitch;
    }

    /**
     * Mevcut roll açısını döndürür.
     *
     * @return Roll açısı (derece)
     */
    public double getRoll() {
        return roll;
    }

    /**
     * Mevcut yaw rate (dönüş hızı) değerini döndürür.
     *
     * @return Yaw rate (derece/saniye)
     */
    public double getYawRate() {
        return yawRate;
    }

    /**
     * NavX cihazının bağlantı durumunu döndürür.
     *
     * <p>Simülasyon modunda her zaman true döner.
     *
     * @return true (her zaman bağlı)
     */
    public boolean isConnected() {
        return true;
    }

    /**
     * Yaw açısını sıfırlar.
     *
     * <p>Bu, DriveSubsystem.zeroHeading() tarafından çağrılır.
     */
    public void zeroYaw() {
        yaw = 0.0;
        updateHalSimulation();
    }

    /**
     * Simülasyon durumunu belirli bir yaw değerine ayarlar.
     *
     * <p>Bu, test amaçları için kullanılabilir.
     *
     * @param yaw Yaw açısı (derece)
     */
    public void setYaw(double yaw) {
        this.yaw = yaw;
        updateHalSimulation();
    }
}
