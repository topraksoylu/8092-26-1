// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.GorusAltSistemi;
import frc.robot.Subsystems.TaretAltSistemi;
import frc.robot.Subsystems.AlimAltSistemi;
import frc.robot.Subsystems.AticiAltSistemi;
import frc.robot.Commands.SurusKomutu;
import frc.robot.Commands.AprilTagaHizalamaKomutu;
import frc.robot.Commands.AprilTagTakipKomutu;
import frc.robot.Commands.TaretHomingKomutu;
// TaretTakipKomutu ve PozTabanliTaretKomutu: otomatik taret devre disi (simdilik)
// import frc.robot.Commands.TaretTakipKomutu;
// import frc.robot.Commands.PozTabanliTaretKomutu;
import frc.robot.Sabitler.*;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public class RobotKapsayici {
  private GorusAltSistemi gorusAltSistemi;
  private SurusAltSistemi surusAltSistemi;
  private TaretAltSistemi taretAltSistemi;
  private AlimAltSistemi alimAltSistemi;
  private AticiAltSistemi aticiAltSistemi;
  private GenericHID surucuKontrolcusu = new GenericHID(OISabitleri.SURUCU_JOYSTICK_PORTU);
  private boolean surucuBagliOncekiDurum = false;
  private double dugme10BaslangicZamani = 0.0;

  // Elastic bildirim hız sınırı
  private boolean limelightHataBildirimGonderildi = false;
  private boolean aticiHazirBildirimGonderildi = false;

  private SendableChooser<Command> otonomSecici;

  /**
   * SmartDashboard, "/" iceren anahtarlari alt tablo gibi yorumlayabilir.
   * Bu nedenle tek seviyeli gorunum icin anahtarlar "_" ile yazilir.
   */
  private static String surucuIstasyonuAnahtari(String ad) {
    return "SurucuIstasyonu_" + ad;
  }

  public RobotKapsayici() {
    // Surus pozu guncellemesinde kullanilacagi icin gorus alt sistemi once kurulur.
    gorusAltSistemi = new GorusAltSistemi();

    // Gorus verisiyle odometri duzeltmesi yapabilmek icin suruse aktarilir.
    surusAltSistemi = new SurusAltSistemi(
      MotorSabitleri.ON_SOL_MOTOR_ID,
      MotorSabitleri.ON_SAG_MOTOR_ID,
      MotorSabitleri.ARKA_SOL_MOTOR_ID,
      MotorSabitleri.ARKA_SAG_MOTOR_ID,
      new Pose2d(),
      gorusAltSistemi
    );
    taretAltSistemi = new TaretAltSistemi();
    alimAltSistemi = new AlimAltSistemi();
    aticiAltSistemi = new AticiAltSistemi();
    SmartDashboard.putNumber("Ayarlama/TaretHizi", ModulSabitleri.TARET_HIZI);
    baglamalariYapilandir();

    // Not: PS4 kontrolcusu USB ile bagli olmali ve Driver Station'da gorunmelidir.
    // Driver Station Joystick sekmesinden 0. port baglantisini dogrulayin.

    surusAltSistemi.setDefaultCommand(
        new SurusKomutu(
            () -> guvenliEksenOku(OISabitleri.SURUCU_X_EKSENI),
            () -> guvenliEksenOku(OISabitleri.SURUCU_Y_EKSENI),
            () -> -guvenliEksenOku(OISabitleri.SURUCU_Z_EKSENI),
            surusAltSistemi
        )
    );
    pathPlannerKomutlariniKaydet();

    try {
      otonomSecici = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", otonomSecici);
    } catch (Exception e) {
      DriverStation.reportError("PathPlanner otonomlari yuklenemedi: " + e.getMessage(), true);
      e.printStackTrace();
      otonomSecici = new SendableChooser<>();
      // PathPlanner hatasinda bos secenekle devam eder.
      otonomSecici.setDefaultOption("Yok (PathPlanner Hatasi)", null);
      SmartDashboard.putData("Auto Chooser", otonomSecici);
    }

    // Baslangicta kontrolcu durumunu kaydeder.
    System.out.println("===================================================");
    System.out.println("ROBOT KAPSAYICI BASLATILDI");
    System.out.println("Surucu Kontrolcu Portu: " + OISabitleri.SURUCU_JOYSTICK_PORTU);
    System.out.println("PS4 kontrolcusunun USB ile bagli oldugundan emin olun!");
    System.out.println("===================================================");
  }

  /**
   * Bu metot periyodik olarak (20ms) cagrilir.
   * Kontrolcu durumunu denetleyip buton durumlarini kaydeder.
   */
  public void periyodik() {
    girdiBaglantiDurumunuGuncelle();
    elasticDurumKontrol();

    // Ham buton durumlarini hata ayiklama icin okur.
    boolean dugme1 = guvenliDugmeOku(1);
    boolean dugme2 = guvenliDugmeOku(2);
    boolean dugme3 = guvenliDugmeOku(3);
    boolean dugme4 = guvenliDugmeOku(4);
    boolean dugme5 = guvenliDugmeOku(5);
    boolean dugme6 = guvenliDugmeOku(6);
    boolean dugme7 = guvenliDugmeOku(7);
    boolean dugme8 = guvenliDugmeOku(8);
    boolean dugme9 = guvenliDugmeOku(9);
    boolean dugme10 = guvenliDugmeOku(10);
    boolean dugme11 = guvenliDugmeOku(11);
    boolean dugme12 = guvenliDugmeOku(12);

    // SmartDashboard'e buton durumlarini yazar.
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme1-Kare"), dugme1);
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme2-Carpi"), dugme2);
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme3-Daire"), dugme3);
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme4-Ucgen"), dugme4);
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme5-L1"), dugme5);
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme6-R1"), dugme6);
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme7-L2"), dugme7);
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme8-R2"), dugme8);
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme9-Paylas"), dugme9);
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme10-Secenekler"), dugme10);
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme11-L3"), dugme11);
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme12-R3"), dugme12);

    // Tum eksen degerlerini okur (PS4'te 6 eksen vardir).
    double eksen0 = guvenliEksenOku(0);  // Sol Analog X
    double eksen1 = guvenliEksenOku(1);  // Sol Analog Y
    double eksen2 = guvenliEksenOku(2);  // Sag Analog X
    double eksen3 = guvenliEksenOku(3);  // Sol Tetik (L2)
    double eksen4 = guvenliEksenOku(4);  // Sag Tetik (R2)
    double eksen5 = guvenliEksenOku(5);  // Sag Analog Y

    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen0-SolX"), eksen0);
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen1-SolY"), eksen1);
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen2-SagX"), eksen2);
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen3-L2"), eksen3);
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen4-R2"), eksen4);
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen5-SagY"), eksen5);

    // POV (D-pad) degerini okur.
    int yonTusu = guvenliPovOku();
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("POV"), yonTusu);

    // Basili butonlari kaydeder.
    if (dugme1 || dugme2 || dugme3 || dugme4 || dugme5 || dugme6 ||
        dugme7 || dugme8 || dugme9 || dugme10 || dugme11 || dugme12) {
      String basiliDugmeler = "";
      if (dugme1) basiliDugmeler += "1(Kare) ";
      if (dugme2) basiliDugmeler += "2(Carpi) ";
      if (dugme3) basiliDugmeler += "3(Daire) ";
      if (dugme4) basiliDugmeler += "4(Ucgen) ";
      if (dugme5) basiliDugmeler += "5(L1) ";
      if (dugme6) basiliDugmeler += "6(R1) ";
      if (dugme7) basiliDugmeler += "7(L2) ";
      if (dugme8) basiliDugmeler += "8(R2) ";
      if (dugme9) basiliDugmeler += "9(Paylas) ";
      if (dugme10) basiliDugmeler += "10(Secenekler) ";
      if (dugme11) basiliDugmeler += "11(L3) ";
      if (dugme12) basiliDugmeler += "12(R3) ";

      SmartDashboard.putString(surucuIstasyonuAnahtari("BasiliDugmeler"), basiliDugmeler);
    } else {
      SmartDashboard.putString(surucuIstasyonuAnahtari("BasiliDugmeler"), "Yok");
    }

    // Eksen gorunum metnini olusturur.
    String eksenBilgisi = String.format("L:(%.2f,%.2f) R:(%.2f,%.2f) LT:%.2f RT:%.2f",
        eksen0, eksen1, eksen2, eksen5, eksen3, eksen4);
    SmartDashboard.putString(surucuIstasyonuAnahtari("EksenBilgisi"), eksenBilgisi);

    // POV yon gorunum metnini olusturur.
    String yonBilgisi = "POV: " + (yonTusu == -1 ? "Merkez" :
                       yonTusu == 0 ? "Yukari" :
                       yonTusu == 45 ? "Yukari-Sag" :
                       yonTusu == 90 ? "Sag" :
                       yonTusu == 135 ? "Asagi-Sag" :
                       yonTusu == 180 ? "Asagi" :
                       yonTusu == 225 ? "Asagi-Sol" :
                       yonTusu == 270 ? "Sol" :
                       yonTusu == 315 ? "Yukari-Sol" : yonTusu);
    SmartDashboard.putString(surucuIstasyonuAnahtari("POVBilgisi"), yonBilgisi);
  }

  private void baglamalariYapilandir() {
    // 1 (Kare): Alim (PWM 9)
    new JoystickButton(surucuKontrolcusu, 1)
        .whileTrue(new RunCommand(() -> alimAltSistemi.al(), alimAltSistemi))
        .onFalse(new InstantCommand(() -> alimAltSistemi.durdur(), alimAltSistemi));

    // 2 (Carpi): Geri at (PWM 9)
    new JoystickButton(surucuKontrolcusu, 2)
        .whileTrue(new RunCommand(() -> alimAltSistemi.geriAt(), alimAltSistemi))
        .onFalse(new InstantCommand(() -> alimAltSistemi.durdur(), alimAltSistemi));

    // 3 (Daire): Yukari tasiyici (PWM 8)
    new JoystickButton(surucuKontrolcusu, 3)
        .whileTrue(new RunCommand(() -> alimAltSistemi.depodanAticiyaYukariTasimaBaslat(), alimAltSistemi))
        .onFalse(new InstantCommand(() -> alimAltSistemi.depodanAticiyaYukariTasimaDurdur(), alimAltSistemi));

    // 4 (Ucgen): Atici (CAN 5)
    new JoystickButton(surucuKontrolcusu, 4)
        .whileTrue(new RunCommand(() -> aticiAltSistemi.at(), aticiAltSistemi))
        .onFalse(new InstantCommand(() -> aticiAltSistemi.durdur(), aticiAltSistemi));

    // 5 (L1): Taret sola (CAN 6)
    new JoystickButton(surucuKontrolcusu, 5)
        .whileTrue(new RunCommand(() -> taretAltSistemi.dondur(-SmartDashboard.getNumber("Ayarlama/TaretHizi", ModulSabitleri.TARET_HIZI)), taretAltSistemi))
        .onFalse(new InstantCommand(() -> taretAltSistemi.durdur(), taretAltSistemi));

    // 6 (R1): Taret saga (CAN 6)
    new JoystickButton(surucuKontrolcusu, 6)
        .whileTrue(new RunCommand(() -> taretAltSistemi.dondur(SmartDashboard.getNumber("Ayarlama/TaretHizi", ModulSabitleri.TARET_HIZI)), taretAltSistemi))
        .onFalse(new InstantCommand(() -> taretAltSistemi.durdur(), taretAltSistemi));

    // 10 (Secenekler): Atici hemen baslar, 1sn sonra yukari tasiyici baslar.
    new JoystickButton(surucuKontrolcusu, 10)
        .onTrue(new InstantCommand(() -> {
          dugme10BaslangicZamani = Timer.getFPGATimestamp();
          aticiAltSistemi.at();
        }, aticiAltSistemi))
        .whileTrue(new RunCommand(() -> {
          if (Timer.getFPGATimestamp() - dugme10BaslangicZamani >= 1.0) {
            alimAltSistemi.depodanAticiyaYukariTasimaBaslat();
          }
        }, alimAltSistemi))
        .onFalse(new InstantCommand(() -> {
          alimAltSistemi.depodanAticiyaYukariTasimaDurdur();
          aticiAltSistemi.durdur();
        }, alimAltSistemi, aticiAltSistemi));

    // 8 (R2): Poz tabanli taret takibi — simdilik devre disi (taret onte, kablo guvenligi)
    // Yeniden etkinlestirmek icin: .toggleOnTrue(new PozTabanliTaretKomutu(taretAltSistemi, surusAltSistemi));

    // 7 (L2): Taret homing — ilk basista baslar, ikinci basista durdurur
    new JoystickButton(surucuKontrolcusu, 7)
        .toggleOnTrue(new TaretHomingKomutu(taretAltSistemi));

    // 9 (Paylas): Yukari tasiyiciyi 0.5 sn ters cevirir, sonra otomatik durdurur.
    new JoystickButton(surucuKontrolcusu, 9)
        .onTrue(
            new RunCommand(() -> alimAltSistemi.depodanAticiyaYukariTasimaTersBaslat(), alimAltSistemi)
                .withTimeout(0.5)
                .andThen(new InstantCommand(() -> alimAltSistemi.depodanAticiyaYukariTasimaDurdur(), alimAltSistemi))
        );
  }

  private void pathPlannerKomutlariniKaydet() {
    NamedCommands.registerCommand(
        "AprilTagHizala",
        new AprilTagaHizalamaKomutu(surusAltSistemi, gorusAltSistemi, 1, 1.0).withTimeout(2.0));
    NamedCommands.registerCommand(
        "AprilTagTakipKisa",
        new AprilTagTakipKomutu(surusAltSistemi, gorusAltSistemi, 1, 1.5).withTimeout(1.5));
    NamedCommands.registerCommand(
        "SurusuDurdur",
        new RunCommand(() -> surusAltSistemi.tumMotorlariDurdur(), surusAltSistemi).withTimeout(0.1));
  }

  private void girdiBaglantiDurumunuGuncelle() {
    boolean bagli = DriverStation.isJoystickConnected(OISabitleri.SURUCU_JOYSTICK_PORTU);
    if (bagli != surucuBagliOncekiDurum) {
      surucuBagliOncekiDurum = bagli;
      if (bagli) {
        Elastic.sendNotification(new Notification(NotificationLevel.INFO,
            "Kontrolcü Bağlandı", "PS4 kontrolcüsü port " + OISabitleri.SURUCU_JOYSTICK_PORTU + "'e bağlandı."));
      } else {
        Elastic.sendNotification(new Notification(NotificationLevel.ERROR,
            "Kontrolcü Koptu!", "PS4 kontrolcüsü bağlantısı kesildi — port " + OISabitleri.SURUCU_JOYSTICK_PORTU));
      }
    }
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("KontrolcuBagli"), bagli);
  }

  /** Kritik sistem durumlarını kontrol eder ve tek seferlik Elastic bildirimi gönderir */
  private void elasticDurumKontrol() {
    // Limelight config hatası — DriverStation aktif ve config yanlışsa uyar
    if (DriverStation.isEnabled()) {
      boolean limelightOk = gorusAltSistemi.isLimelightConfigOk();
      if (!limelightOk && !limelightHataBildirimGonderildi) {
        Elastic.sendNotification(new Notification(NotificationLevel.WARNING,
            "Limelight Config Hatası",
            gorusAltSistemi.getLimelightConfigStatus(), 5000));
        limelightHataBildirimGonderildi = true;
      } else if (limelightOk) {
        limelightHataBildirimGonderildi = false; // Config düzelince bayrak sıfırla
      }

      // Atıcı hedef RPM'e ulaştığında bildir (tek seferlik)
      boolean aticiHazir = aticiAltSistemi.isHizaUlasti();
      if (aticiHazir && !aticiHazirBildirimGonderildi) {
        Elastic.sendNotification(new Notification(NotificationLevel.INFO,
            "Atıcı Hazır", String.format("%.0f RPM — atış yapılabilir.", aticiAltSistemi.getAktuelRPM()), 2000));
        aticiHazirBildirimGonderildi = true;
      } else if (!aticiHazir) {
        aticiHazirBildirimGonderildi = false;
      }
    } else {
      // Devre dışıyken bayrakları sıfırla
      limelightHataBildirimGonderildi = false;
      aticiHazirBildirimGonderildi = false;
    }
  }

  private double guvenliEksenOku(int eksen) {
    int port = OISabitleri.SURUCU_JOYSTICK_PORTU;
    if (!DriverStation.isJoystickConnected(port)) {
      return 0.0;
    }
    int eksenSayisi = DriverStation.getStickAxisCount(port);
    if (eksen < 0 || eksen >= eksenSayisi) {
      return 0.0;
    }
    return surucuKontrolcusu.getRawAxis(eksen);
  }

  private boolean guvenliDugmeOku(int dugme) {
    int port = OISabitleri.SURUCU_JOYSTICK_PORTU;
    if (!DriverStation.isJoystickConnected(port)) {
      return false;
    }
    if (dugme <= 0 || dugme > 32) {
      return false;
    }
    int dugmeMaskesi = DriverStation.getStickButtons(port);
    return (dugmeMaskesi & (1 << (dugme - 1))) != 0;
  }

  private int guvenliPovOku() {
    int port = OISabitleri.SURUCU_JOYSTICK_PORTU;
    if (!DriverStation.isJoystickConnected(port)) {
      return -1;
    }
    int povSayisi = DriverStation.getStickPOVCount(port);
    if (povSayisi <= 0) {
      return -1;
    }
    return surucuKontrolcusu.getPOV();
   }

  public void sensorleriSifirla() {
    surusAltSistemi.yonuSifirla();
    surusAltSistemi.encoderlariSifirla();
  }

  public Command otonomKomutAl() {
    return otonomSecici.getSelected();
  }

  /**
   * Gorus alt sistemi ornegini dondurur.
   * @return GorusAltSistemi
   */
  public GorusAltSistemi gorusAltSistemiAl() {
    return gorusAltSistemi;
  }

}
