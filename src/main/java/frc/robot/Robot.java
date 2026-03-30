// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public class Robot extends TimedRobot {
  private Command m_otonomKomut;

  private final RobotKapsayici m_robotKapsayici;

  public Robot() {
    m_robotKapsayici = new RobotKapsayici();
  }

  @Override
  public void robotInit() {
    Elastic.selectTab("Ayarlama");
    Elastic.sendNotification(new Notification(NotificationLevel.INFO,
        "Robot Hazir", "Sistem baslatildi. Kontroller ve sensörleri dogrulayın."));
  }

  @Override
  public void robotPeriodic() {
    // Kapsayici tarafindaki tanilama akislarini her dongude gunceller.
    m_robotKapsayici.periyodik();

    // Komut zamanlayicisini her dongude calistirir.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    Elastic.selectTab("Ayarlama");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Elastic.selectTab("Auto");
    Elastic.sendNotification(new Notification(NotificationLevel.INFO,
        "Otonom Başladı", "Sensörler sıfırlandı, otonom rota çalışıyor."));

    m_robotKapsayici.sensorleriSifirla();

    m_otonomKomut = m_robotKapsayici.otonomKomutAl();

    if (m_otonomKomut != null) {
      CommandScheduler.getInstance().schedule(m_otonomKomut);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Elastic.selectTab("Teleop");
    Elastic.sendNotification(new Notification(NotificationLevel.INFO,
        "Teleop Başladı", "Süruş kontrolü sürücüye devredildi."));

    m_robotKapsayici.sensorleriSifirla();

    if (m_otonomKomut != null) {
      m_otonomKomut.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
