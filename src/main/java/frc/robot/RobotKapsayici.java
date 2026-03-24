// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.GorusAltSistemi;
import frc.robot.Commands.SurusKomutu;
import frc.robot.Commands.AprilTagaHizalamaKomutu;
import frc.robot.Commands.AprilTagTakipKomutu;
import frc.robot.Sabitler.*;

public class RobotKapsayici {
  private GorusAltSistemi gorusAltSistemi;
  private SurusAltSistemi surusAltSistemi;
  private GenericHID surucuKontrolcusu = new GenericHID(OISabitleri.SURUCU_JOYSTICK_PORTU);
  private final PWMVictorSPX pwmTestMotoru = new PWMVictorSPX(1);
  private boolean surucuBagliOncekiDurum = false;

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
    baglamalariYapilandir();

    // Not: PS4 kontrolcusu USB ile bagli olmali ve Driver Station'da gorunmelidir.
    // Driver Station Joystick sekmesinden 0. port baglantisini dogrulayin.

    surusAltSistemi.setDefaultCommand(
        new SurusKomutu(
            () -> guvenliEksenOku(OISabitleri.SURUCU_Y_EKSENI),
            () -> guvenliEksenOku(OISabitleri.SURUCU_X_EKSENI),
            () -> -guvenliEksenOku(OISabitleri.SURUCU_Z_EKSENI),
            surusAltSistemi
        )
    );
    pathPlannerKomutlariniKaydet();

    try {
      otonomSecici = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Otonom Secici", otonomSecici);
    } catch (Exception e) {
      DriverStation.reportError("PathPlanner otonomlari yuklenemedi: " + e.getMessage(), true);
      e.printStackTrace();
      otonomSecici = new SendableChooser<>();
      // PathPlanner hatasinda bos secenekle devam eder.
      otonomSecici.setDefaultOption("Yok (PathPlanner Hatasi)", null);
      SmartDashboard.putData("Otonom Secici", otonomSecici);
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

    // PWM1 test motoru: 1->%30, 2->%60, 3->%90, 4->%100 (oncelik 4 > 3 > 2 > 1)
    double pwmTestHizi = dugme4 ? 1.00 : (dugme3 ? 0.90 : (dugme2 ? 0.80 : (dugme1 ? 0.75 : 0.0)));
    pwmTestMotoru.set(pwmTestHizi);
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("PwmTestHizi"), pwmTestHizi);

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
    double eksen2 = guvenliEksenOku(2);  // Sol Tetik (L2)
    double eksen3 = guvenliEksenOku(3);  // Sag Analog X
    // Bazi kontrolcu/surucu kombinasyonlarinda axis 4/5 mevcut degil.
    // DS uyarisi spamini onlemek icin bu eksenler zorunlu okunmaz.
    double eksen4 = 0.0;
    double eksen5 = 0.0;

    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen0-SolX"), eksen0);
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen1-SolY"), eksen1);
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen2-L2"), eksen2);
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen3-SagX"), eksen3);
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen4-SagY"), eksen4);
    SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen5-R2"), eksen5);

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
        eksen0, eksen1, eksen3, eksen4, eksen2, eksen5);
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
    // Buton baglamalari acikca tutuldu: sahada hata ayiklama hizli olsun.
    // PWM1 test motor hizi periyodikte 1/2/3 dugmelerinden okunur.
    new JoystickButton(surucuKontrolcusu, 1)
        .onTrue(new InstantCommand(() -> {
            System.out.println("===========================================");
            System.out.println("DUGME 1 (KARE) BASILDI");
            System.out.println("Eylem: PWM1 Victor SPX test motoru %30");
            System.out.println("===========================================");
            SmartDashboard.putString(surucuIstasyonuAnahtari("SonDugme"), "Kare (1) - PWM1 Victor SPX");
        }));

    new JoystickButton(surucuKontrolcusu, 2)
        .onTrue(new InstantCommand(() -> {
            System.out.println("===========================================");
            System.out.println("DUGME 2 (CARPI) BASILDI");
            System.out.println("Eylem: PWM1 Victor SPX test motoru %60");
            System.out.println("===========================================");
            SmartDashboard.putString(surucuIstasyonuAnahtari("SonDugme"), "Carpi (2) - PWM1 Victor SPX");
        }));

    new JoystickButton(surucuKontrolcusu, 3)
        .onTrue(new InstantCommand(() -> {
            System.out.println("===========================================");
            System.out.println("DUGME 3 (DAIRE) BASILDI");
            System.out.println("Eylem: PWM1 Victor SPX test motoru %90");
            System.out.println("===========================================");
            SmartDashboard.putString(surucuIstasyonuAnahtari("SonDugme"), "Daire (3) - PWM1 Victor SPX");
        }));

    // L1 dugmesi (5): gorusten AprilTag ile poz sifirlar.
    new JoystickButton(surucuKontrolcusu, 5)
        .onTrue(new RunCommand(() -> {
            System.out.println("===========================================");
            System.out.println("DUGME 5 (L1) BASILDI");
            System.out.println("Eylem: AprilTag gorusunden robot pozu sifirlaniyor");
            System.out.println("===========================================");
            SmartDashboard.putString(surucuIstasyonuAnahtari("SonDugme"), "L1 (5) - Poz Sifirla");
            surusAltSistemi.gorusIlePozuSifirla();
        }, surusAltSistemi));

    // R1 dugmesi (6): AprilTag hedefiyle hizalar.
    // Robotu AprilTag 1'e yonelecek sekilde surer.
    new JoystickButton(surucuKontrolcusu, 6)
        .onTrue(new RunCommand(() -> {
            System.out.println("===========================================");
            System.out.println("DUGME 6 (R1) BASILDI");
            System.out.println("Eylem: AprilTag ID 1 hizalamasi baslatiliyor");
            System.out.println("Hedef: Tagin 1 metre onunde dur");
            System.out.println("===========================================");
            SmartDashboard.putString(surucuIstasyonuAnahtari("SonDugme"), "R1 (6) - AprilTag Hizala");
        }, surusAltSistemi))
        .whileTrue(new AprilTagaHizalamaKomutu(
            surusAltSistemi,
            gorusAltSistemi,
            1,      // Hedef AprilTag kimligi
            1.0     // Tagin 1 metre onunde durur
        ));

    // R2 dugmesi (8): AprilTag'i surekli takip eder.
    // Basili tuttukca takip eder, birakinca durur.
    // Tag ile 1.5 metre mesafeyi korur.
    new JoystickButton(surucuKontrolcusu, 8)
        .onTrue(new RunCommand(() -> {
            System.out.println("===========================================");
            System.out.println("DUGME 8 (R2) BASILDI");
            System.out.println("Eylem: Surekli AprilTag takibi baslatiliyor");
            System.out.println("Hedef: Tag ID 1, Mesafe: 1.5 metre");
            System.out.println("Takip icin basili tutun, durdurmak icin birakin");
            System.out.println("===========================================");
            SmartDashboard.putString(surucuIstasyonuAnahtari("SonDugme"), "R2 (8) - Tag Takibi");
        }, surusAltSistemi))
        .whileTrue(new AprilTagTakipKomutu(
            surusAltSistemi,
            gorusAltSistemi,
            1,      // Hedef AprilTag kimligi
            1.5     // 1.5 metre takip mesafesi
        ));
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
      DriverStation.reportWarning(
          bagli ? "Surucu kontrolcusu baglandi (port " + OISabitleri.SURUCU_JOYSTICK_PORTU + ")"
              : "Surucu kontrolcusu baglantisi koptu (port " + OISabitleri.SURUCU_JOYSTICK_PORTU + ")",
          false);
    }
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("KontrolcuBagli"), bagli);
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
