// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AprilTagTakipKomutu;
import frc.robot.Commands.AprilTagaHizalamaKomutu;
import frc.robot.Commands.OtomatikTaretKomutu;
import frc.robot.Commands.SurusKomutu;
import frc.robot.Commands.TaretHomingKomutu;
import frc.robot.Sabitler.*;
import frc.robot.Subsystems.AlimAltSistemi;
import frc.robot.Subsystems.AticiAltSistemi;
import frc.robot.Subsystems.GorusAltSistemi;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.Subsystems.TaretAltSistemi;
import frc.robot.util.Elastic;
import frc.robot.util.AtisHesaplayici;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.KontrolcuProfili;
import frc.robot.util.PS4BasitProfili;
import frc.robot.util.PS4TamProfili;
import frc.robot.util.XboxProfili;

public class RobotKapsayici {

  // ── Alt sistemler ──────────────────────────────────────────────────────────
  private final GorusAltSistemi   gorusAltSistemi;
  private final SurusAltSistemi   surusAltSistemi;
  private final TaretAltSistemi   taretAltSistemi;
  private final AlimAltSistemi    alimAltSistemi;
  private final AticiAltSistemi   aticiAltSistemi;

  // ── Kontrolcü ─────────────────────────────────────────────────────────────
  private final GenericHID surucuKontrolcusu = new GenericHID(OISabitleri.SURUCU_JOYSTICK_PORTU);

  /**
   * Elastic / SmartDashboard'dan runtime'da seçilebilir kontrolcü profili.
   * Yeni kol takılınca veya kol değişince deploy gerekmez.
   */
  private final SendableChooser<KontrolcuProfili> profilSecici = new SendableChooser<>();

  // ── Durum izleme ──────────────────────────────────────────────────────────
  private boolean surucuBagliOncekiDurum        = false;
  private boolean limelightHataBildirimGonderildi = false;
  private boolean aticiHazirBildirimGonderildi   = false;

  // ── Otonomi ───────────────────────────────────────────────────────────────
  private SendableChooser<Command> otonomSecici;

  // ── SmartDashboard anahtar yardımcısı ────────────────────────────────────
  private static String surucuIstasyonuAnahtari(String ad) {
    return "SurucuIstasyonu_" + ad;
  }

  public RobotKapsayici() {
    // Görüş alt sistemi önce kurulur — sürüşe poz fuzyonu için gerekli
    gorusAltSistemi = new GorusAltSistemi();
    surusAltSistemi = new SurusAltSistemi(
        MotorSabitleri.ON_SOL_MOTOR_ID,
        MotorSabitleri.ON_SAG_MOTOR_ID,
        MotorSabitleri.ARKA_SOL_MOTOR_ID,
        MotorSabitleri.ARKA_SAG_MOTOR_ID,
        new Pose2d(),
        gorusAltSistemi
    );
    taretAltSistemi  = new TaretAltSistemi();
    alimAltSistemi   = new AlimAltSistemi();
    aticiAltSistemi  = new AticiAltSistemi();

    kontrolcuProfilleriniKur();
    defaultKomutlariniKur();
    baglamalariYapilandir();
    pathPlannerKomutlariniKaydet();
    otonomSeciciKur();

    SmartDashboard.putNumber("Ayarlama/TaretHizi", ModulSabitleri.TARET_HIZI);

    System.out.println("===================================================");
    System.out.println("ROBOT KAPSAYICI BASLATILDI");
    System.out.println("Surucu Kontrolcu Portu: " + OISabitleri.SURUCU_JOYSTICK_PORTU);
    System.out.println("Kontrolcu profili Elastic'ten secilir: 'Kontrolcu/Profil'");
    System.out.println("===================================================");
  }

  // ── Profil kurulumu ────────────────────────────────────────────────────────

  private void kontrolcuProfilleriniKur() {
    profilSecici.setDefaultOption(
        "PS4 Tam (Axis 4/5 analog)",  new PS4TamProfili(surucuKontrolcusu));
    profilSecici.addOption(
        "PS4 Basit (analog trigger yok)", new PS4BasitProfili(surucuKontrolcusu));
    profilSecici.addOption(
        "Xbox",                        new XboxProfili(surucuKontrolcusu));

    SmartDashboard.putData("Kontrolcu/Profil", profilSecici);
  }

  /** Seçili profili null-safe döndürür; seçim yoksa PS4 Tam fallback. */
  private KontrolcuProfili profil() {
    KontrolcuProfili secili = profilSecici.getSelected();
    // SendableChooser setDefaultOption sonrası null olmaz, yine de güvenli
    return secili != null ? secili : new PS4TamProfili(surucuKontrolcusu);
  }

  // ── Default komutlar ──────────────────────────────────────────────────────

  private void defaultKomutlariniKur() {
    // Sürüş — profil lambdaları her döngüde getSelected() çağırır
    surusAltSistemi.setDefaultCommand(
        new SurusKomutu(
            () -> profil().yanal(),
            () -> profil().ileriGeri(),
            () -> profil().donus(),
            surusAltSistemi
        )
    );

    // Otomatik taret — homing yapılmamışsa durdurur, yapılmışsa Limelight/odometri
    taretAltSistemi.setDefaultCommand(
        new OtomatikTaretKomutu(taretAltSistemi, gorusAltSistemi, surusAltSistemi)
    );
  }

  // ── Buton bağlamaları ─────────────────────────────────────────────────────

  private void baglamalariYapilandir() {

    // ── Profil tabanlı Trigger'lar ─────────────────────────────────────────
    // Her Trigger, profil değişince otomatik yeni profili kullanır (lambda capture)

    Trigger alimTetik         = new Trigger(() -> profil().alimBasili());
    Trigger geriAtTetik       = new Trigger(() -> profil().geriAtBasili());
    Trigger tasiyiciTetik     = new Trigger(() -> profil().tasiyiciBasili());
    Trigger tasiyiciTersTetik = new Trigger(() -> profil().tasiyiciTersBasili());
    Trigger spinupTetik       = new Trigger(() -> profil().aticiSpinupBasili());
    Trigger atesTetik         = new Trigger(() -> profil().atesBasili());
    Trigger taretSolaTetik    = new Trigger(() -> profil().taretSolaBasili());
    Trigger taretSagaTetik    = new Trigger(() -> profil().taretSagaBasili());
    Trigger taretHomingTetik  = new Trigger(() -> profil().taretHomingBasili());
    Trigger gyroSifirTetik    = new Trigger(() -> profil().gyroSifirlaBasili());

    // ── Atıcı hazır Trigger ────────────────────────────────────────────────
    Trigger aticiHazirTetik = new Trigger(aticiAltSistemi::isHizaUlasti);

    // ── Alım ──────────────────────────────────────────────────────────────
    alimTetik
        .whileTrue(new RunCommand(() -> alimAltSistemi.al(), alimAltSistemi))
        .onFalse(new InstantCommand(() -> alimAltSistemi.durdur(), alimAltSistemi));

    geriAtTetik
        .whileTrue(new RunCommand(() -> alimAltSistemi.geriAt(), alimAltSistemi))
        .onFalse(new InstantCommand(() -> alimAltSistemi.durdur(), alimAltSistemi));

    // ── Taşıyıcı ──────────────────────────────────────────────────────────
    tasiyiciTetik
        .whileTrue(new RunCommand(
            () -> alimAltSistemi.depodanAticiyaYukariTasimaBaslat(), alimAltSistemi))
        .onFalse(new InstantCommand(
            () -> alimAltSistemi.depodanAticiyaYukariTasimaDurdur(), alimAltSistemi));

    // Unjam: 0.5 sn ters çevirir, sonra otomatik durur
    tasiyiciTersTetik.onTrue(
        new RunCommand(
            () -> alimAltSistemi.depodanAticiyaYukariTasimaTersBaslat(), alimAltSistemi)
            .withTimeout(0.5)
            .andThen(new InstantCommand(
                () -> alimAltSistemi.depodanAticiyaYukariTasimaDurdur(), alimAltSistemi))
    );

    // ── Atıcı spin-up (mesafe tabanlı RPM — hedef yoksa sabit fallback) ────
    spinupTetik
        .whileTrue(new RunCommand(this::hedefMesafesineGoreAticiCalistir, aticiAltSistemi))
        .onFalse(new InstantCommand(() -> aticiAltSistemi.durdur(), aticiAltSistemi));

    // ── Ateş kilidi: sadece atıcı hedef RPM'deyse taşıyıcı çalışır ────────
    atesTetik.and(aticiHazirTetik)
        .whileTrue(new RunCommand(
            () -> alimAltSistemi.depodanAticiyaYukariTasimaBaslat(), alimAltSistemi))
        .onFalse(new InstantCommand(
            () -> alimAltSistemi.depodanAticiyaYukariTasimaDurdur(), alimAltSistemi));

    // ── Titreşim: atıcı hedefe ulaşınca bildir ────────────────────────────
    aticiHazirTetik
        .onTrue(new InstantCommand(() -> profil().titrestir(0.6)))
        .onFalse(new InstantCommand(() -> profil().titrestir(0.0)));

    // ── Taret fallback (manuel) — otomatik taret komutunu keser ───────────
    taretSolaTetik
        .whileTrue(new RunCommand(
            () -> taretAltSistemi.dondur(
                -SmartDashboard.getNumber("Ayarlama/TaretHizi", ModulSabitleri.TARET_HIZI)),
            taretAltSistemi))
        .onFalse(new InstantCommand(() -> taretAltSistemi.durdur(), taretAltSistemi));

    taretSagaTetik
        .whileTrue(new RunCommand(
            () -> taretAltSistemi.dondur(
                SmartDashboard.getNumber("Ayarlama/TaretHizi", ModulSabitleri.TARET_HIZI)),
            taretAltSistemi))
        .onFalse(new InstantCommand(() -> taretAltSistemi.durdur(), taretAltSistemi));

    // ── Taret homing ──────────────────────────────────────────────────────
    taretHomingTetik.toggleOnTrue(new TaretHomingKomutu(taretAltSistemi));

    // ── Gyro sıfırla ──────────────────────────────────────────────────────
    gyroSifirTetik.onTrue(new InstantCommand(
        () -> surusAltSistemi.yonuSifirla(), surusAltSistemi));
  }

  // ── PathPlanner ───────────────────────────────────────────────────────────

  private void pathPlannerKomutlariniKaydet() {
    NamedCommands.registerCommand(
        "AprilTagHizala",
        new AprilTagaHizalamaKomutu(surusAltSistemi, gorusAltSistemi, 1, 1.0)
            .withTimeout(2.0));
    NamedCommands.registerCommand(
        "AprilTagTakipKisa",
        new AprilTagTakipKomutu(surusAltSistemi, gorusAltSistemi, 1, 1.5)
            .withTimeout(1.5));
    NamedCommands.registerCommand(
        "SurusuDurdur",
        new RunCommand(() -> surusAltSistemi.tumMotorlariDurdur(), surusAltSistemi)
            .withTimeout(0.1));
  }

  private void otonomSeciciKur() {
    try {
      otonomSecici = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", otonomSecici);
    } catch (Exception e) {
      DriverStation.reportError("PathPlanner otonomlari yuklenemedi: " + e.getMessage(), true);
      otonomSecici = new SendableChooser<>();
      otonomSecici.setDefaultOption("Yok (PathPlanner Hatasi)", null);
      SmartDashboard.putData("Auto Chooser", otonomSecici);
    }
  }

  // ── Periyodik ─────────────────────────────────────────────────────────────

  public void periyodik() {
    girdiBaglantiDurumunuGuncelle();
    elasticDurumKontrol();

    // Aktif profil adını dashboard'a yaz
    KontrolcuProfili aktifProfil = profilSecici.getSelected();
    SmartDashboard.putString(surucuIstasyonuAnahtari("AktifProfil"),
        aktifProfil != null ? aktifProfil.getClass().getSimpleName() : "?");

    // Ham buton durumları (debug)
    for (int i = 1; i <= 12; i++) {
      SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme" + i), guvenliDugmeOku(i));
    }

    // Eksen değerleri (debug)
    for (int i = 0; i <= 5; i++) {
      SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen" + i), guvenliEksenOku(i));
    }

    SmartDashboard.putNumber(surucuIstasyonuAnahtari("POV"), guvenliPovOku());
  }

  private void hedefMesafesineGoreAticiCalistir() {
    if (gorusAltSistemi.isHedefTagGorunuyor()) {
      double mesafeMetre = gorusAltSistemi.getDistanceToTarget();
      double hedefRpm = AtisHesaplayici.hesaplaHedefRpm(mesafeMetre);
      aticiAltSistemi.atRPM(hedefRpm);
      SmartDashboard.putString("Atici/AtisModu", "MesafeTabanli");
      SmartDashboard.putNumber("Atici/HedefMesafe_m", mesafeMetre);
      SmartDashboard.putNumber("Atici/HesaplananRPM", hedefRpm);
      return;
    }
    aticiAltSistemi.at();
    SmartDashboard.putString("Atici/AtisModu", "SabitRPM_Fallback");
  }

  // ── Bağlantı / Elastic bildirimleri ──────────────────────────────────────

  private void girdiBaglantiDurumunuGuncelle() {
    boolean bagli = DriverStation.isJoystickConnected(OISabitleri.SURUCU_JOYSTICK_PORTU);
    if (bagli != surucuBagliOncekiDurum) {
      surucuBagliOncekiDurum = bagli;
      if (bagli) {
        Elastic.sendNotification(new Notification(NotificationLevel.INFO,
            "Kontrolcü Bağlandı",
            "PS4/Xbox port " + OISabitleri.SURUCU_JOYSTICK_PORTU + "'e bağlandı."));
      } else {
        Elastic.sendNotification(new Notification(NotificationLevel.ERROR,
            "Kontrolcü Koptu!",
            "Port " + OISabitleri.SURUCU_JOYSTICK_PORTU + " bağlantısı kesildi."));
      }
    }
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("KontrolcuBagli"), bagli);
  }

  private void elasticDurumKontrol() {
    if (DriverStation.isEnabled()) {
      // Limelight config hatası
      boolean llOk = gorusAltSistemi.isLimelightConfigOk();
      if (!llOk && !limelightHataBildirimGonderildi) {
        Elastic.sendNotification(new Notification(NotificationLevel.WARNING,
            "Limelight Config Hatası",
            gorusAltSistemi.getLimelightConfigStatus(), 5000));
        limelightHataBildirimGonderildi = true;
      } else if (llOk) {
        limelightHataBildirimGonderildi = false;
      }

      // Atıcı RPM hazır bildirimi (tek seferlik — titreşim de ayrıca çalışır)
      boolean aticiHazir = aticiAltSistemi.isHizaUlasti();
      if (aticiHazir && !aticiHazirBildirimGonderildi) {
        Elastic.sendNotification(new Notification(NotificationLevel.INFO,
            "Atıcı Hazır",
            String.format("%.0f RPM — atış yapılabilir.", aticiAltSistemi.getAktuelRPM()),
            2000));
        aticiHazirBildirimGonderildi = true;
      } else if (!aticiHazir) {
        aticiHazirBildirimGonderildi = false;
      }

      // Taret homing uyarısı
      if (!taretAltSistemi.isHomingTamamlandi() && !limelightHataBildirimGonderildi) {
        SmartDashboard.putString("Taret/OtoMod", "HOMING_GEREKLI");
      }
    } else {
      limelightHataBildirimGonderildi = false;
      aticiHazirBildirimGonderildi    = false;
    }
  }

  // ── Güvenli HID okuma yardımcıları ───────────────────────────────────────

  private double guvenliEksenOku(int eksen) {
    int port = OISabitleri.SURUCU_JOYSTICK_PORTU;
    if (!DriverStation.isJoystickConnected(port)) return 0.0;
    if (eksen < 0 || eksen >= DriverStation.getStickAxisCount(port)) return 0.0;
    return surucuKontrolcusu.getRawAxis(eksen);
  }

  private boolean guvenliDugmeOku(int dugme) {
    int port = OISabitleri.SURUCU_JOYSTICK_PORTU;
    if (!DriverStation.isJoystickConnected(port)) return false;
    if (dugme <= 0 || dugme > 32) return false;
    return (DriverStation.getStickButtons(port) & (1 << (dugme - 1))) != 0;
  }

  private int guvenliPovOku() {
    int port = OISabitleri.SURUCU_JOYSTICK_PORTU;
    if (!DriverStation.isJoystickConnected(port)) return -1;
    if (DriverStation.getStickPOVCount(port) <= 0) return -1;
    return surucuKontrolcusu.getPOV();
  }

  // ── Dışa açık metodlar ───────────────────────────────────────────────────

  public void sensorleriSifirla() {
    surusAltSistemi.yonuSifirla();
    surusAltSistemi.encoderlariSifirla();
  }

  public Command otonomKomutAl() {
    return otonomSecici.getSelected();
  }

  public GorusAltSistemi gorusAltSistemiAl() {
    return gorusAltSistemi;
  }
}
