// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimulationConstants;

/**
 * Fizik tabanlı mekanum sürüş simülasyonu.
 * WPILib'in DifferentialDrivetrainSim sınıfını kullanarak mekanum sürüşü yaklaşık olarak modeller.
 * Not: WPILib mekanum özelinde bir simülasyon sınıfı sağlamadığı için, diferansiyel sürüş
 * simülasyonu standart yaklaşımdır ve yazılım testi için yeterlidir.
 */
public class DrivetrainSimulator {
    private final DifferentialDrivetrainSim sim;

    /**
     * Yeni bir sürücü treni simülatörü oluşturur.
     * Robot fiziksel özelliklerini ve motor özelliklerini kullanarak gerçekçi bir simülasyon sağlar.
     */
    public DrivetrainSimulator() {
        // Her taraf için 2 NEO motorlu şanzıman modeli oluştur
        DCMotor gearbox = DCMotor.getNEO(SimulationConstants.DrivetrainSimulation.NUM_MOTORS_PER_SIDE)
                .withReduction(SimulationConstants.DrivetrainSimulation.GEAR_RATIO);

        // Sürücü treni simülasyonunu oluştur
        sim = new DifferentialDrivetrainSim(
            gearbox,
            SimulationConstants.DrivetrainSimulation.GEAR_RATIO,
            SimulationConstants.DrivetrainSimulation.ROBOT_MOI,
            SimulationConstants.DrivetrainSimulation.ROBOT_MASS_KG,
            SimulationConstants.DrivetrainSimulation.WHEEL_RADIUS,
            DriveConstants.TRACK_WIDTH,
            null  // Varsayılan gürültü parametrelerini kullan
        );
    }

    /**
     * Simülasyon için motor voltaj girişlerini ayarlar.
     *
     * @param leftVoltage Sol taraf voltajı (volts)
     * @param rightVoltage Sağ taraf voltajı (volts)
     */
    public void setInputs(double leftVoltage, double rightVoltage) {
        sim.setInputs(leftVoltage, rightVoltage);
    }

    /**
     * Simülasyonu belirtilen zaman adımı kadar günceller.
     *
     * @param dtSeconds Zaman adımı (saniye)
     */
    public void update(double dtSeconds) {
        sim.update(dtSeconds);
    }

    /**
     * Sol taraf pozisyonunu metre cinsinden döndürür.
     *
     * @return Sol tekerlek pozisyonu (metre)
     */
    public double getLeftPositionMeters() {
        return sim.getLeftPositionMeters();
    }

    /**
     * Sağ taraf pozisyonunu metre cinsinden döndürür.
     *
     * @return Sağ tekerlek pozisyonu (metre)
     */
    public double getRightPositionMeters() {
        return sim.getRightPositionMeters();
    }

    /**
     * Sol taraf hızını metre/saniye cinsinden döndürür.
     *
     * @return Sol tekerlek hızı (m/s)
     */
    public double getLeftVelocityMetersPerSecond() {
        return sim.getLeftVelocityMetersPerSecond();
    }

    /**
     * Sağ taraf hızını metre/saniye cinsinden döndürür.
     *
     * @return Sağ tekerlek hızı (m/s)
     */
    public double getRightVelocityMetersPerSecond() {
        return sim.getRightVelocityMetersPerSecond();
    }

    /**
     * Robot başlığını radyan cinsinden döndürür.
     *
     * @return Robot başlığı (radyan)
     */
    public double getHeadingRadians() {
        return sim.getHeading().getRadians();
    }

    /**
     * Robot başlığını derece cinsinden döndürür.
     *
     * @return Robot başlığı (derece)
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(getHeadingRadians());
    }

    /**
     * Alt seviyeli WPILib DifferentialDrivetrainSim nesnesine erişim sağlar.
     * Gelişmiş simülasyon özellikleri için kullanılır.
     *
     * @return Altta yatan DifferentialDrivetrainSim nesnesi
     */
    public DifferentialDrivetrainSim getSim() {
        return sim;
    }
}
