// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AprilTagTakipKomutu;
import frc.robot.Commands.AprilTagaHizalamaKomutu;
import frc.robot.Commands.LimelightMerkezlemeKomutu;
import frc.robot.Commands.SurusKomutu;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Sabitler.*;
import frc.robot.Subsystems.AlimAltSistemi;
import frc.robot.Subsystems.AticiAltSistemi;
import frc.robot.Subsystems.GorusAltSistemi;
import frc.robot.Subsystems.SurusAltSistemi;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import frc.robot.util.PS4TamProfili;

public class RobotKapsayici {

  //  Alt sistemler 
  private final GorusAltSistemi   gorusAltSistemi;
  private final SurusAltSistemi   surusAltSistemi;
  private final AlimAltSistemi    alimAltSistemi;
  private final AticiAltSistemi   aticiAltSistemi;

  //  Kontrolc 
  private final GenericHID surucuKontrolcusu = new GenericHID(OISabitleri.SURUCU_JOYSTICK_PORTU);
  private final PS4TamProfili surucuProfili = new PS4TamProfili(surucuKontrolcusu);

  //  Durum izleme 
  private boolean surucuBagliOncekiDurum        = false;
  private boolean limelightHataBildirimGonderildi = false;
  private boolean aticiHazirBildirimGonderildi   = false;

  //  Otonomi 
  private SendableChooser<Command> otonomSecici;

  //  SmartDashboard anahtar yardmcs 
  private static String surucuIstasyonuAnahtari(String ad) {
    return "SurucuIstasyonu_" + ad;
  }

  public RobotKapsayici() {
    // Gr alt sistemi nce kurulur  sre poz fuzyonu iin gerekli
    gorusAltSistemi = new GorusAltSistemi();
    surusAltSistemi = new SurusAltSistemi(
        MotorSabitleri.ON_SOL_MOTOR_ID,
        MotorSabitleri.ON_SAG_MOTOR_ID,
        MotorSabitleri.ARKA_SOL_MOTOR_ID,
        MotorSabitleri.ARKA_SAG_MOTOR_ID,
        new Pose2d(),
        gorusAltSistemi
    );
    alimAltSistemi   = new AlimAltSistemi();
    aticiAltSistemi  = new AticiAltSistemi();
    defaultKomutlariniKur();
    baglamalariYapilandir();
    pathPlannerKomutlariniKaydet();
    otonomSeciciKur();

    System.out.println("===================================================");
    System.out.println("ROBOT KAPSAYICI BASLATILDI");
    System.out.println("Surucu Kontrolcu Portu: " + OISabitleri.SURUCU_JOYSTICK_PORTU);
    System.out.println("PS4 tek surucu modu aktif.");
    System.out.println("===================================================");
  }

  //  Profil kurulumu 
  private void defaultKomutlariniKur() {
    // Sr  profil lambdalar her dngde getSelected() arr
    surusAltSistemi.setDefaultCommand(
        new SurusKomutu(
            () -> surucuProfili.yanal(),
            () -> surucuProfili.ileriGeri(),
            () -> surucuProfili.donus(),
            surusAltSistemi
        )
    );

    // Taret varsaylan komutu yok - sadece manuel kontrol (L1/R1/L2)
  }

  //  Buton balamalar 

  private void baglamalariYapilandir() {

    //  Profil tabanl Trigger'lar 
    // Her Trigger, profil deiince otomatik yeni profili kullanr (lambda capture)

    // Intake/Reverse butonlari profile'den bagimsiz sabit: 1 ve 2
    Trigger alimTetik         = new Trigger(() -> surucuKontrolcusu.getRawButton(1));
    Trigger geriAtTetik       = new Trigger(() -> surucuKontrolcusu.getRawButton(2));
    Trigger tasiyiciTetik     = new Trigger(() -> surucuProfili.tasiyiciBasili());
    Trigger tasiyiciTersTetik = new Trigger(() -> surucuProfili.tasiyiciTersBasili());
    Trigger yakinAtisTetik    = new Trigger(() -> surucuProfili.yakinAtisBasili());
    Trigger ortaAtisTetik    = new Trigger(() -> surucuProfili.ortaAtisBasili());
    Trigger uzakAtisTetik    = new Trigger(() -> surucuProfili.uzakAtisBasili());
    Trigger limelightHizalaTetik = new Trigger(() -> surucuKontrolcusu.getRawButton(8));
    Trigger gyroSifirTetik    = new Trigger(() -> surucuProfili.gyroSifirlaBasili());
    Trigger gecikmeliAtisTetik = new Trigger(() -> surucuProfili.gecikmeliAtisBasili());

    //  Atc hazr Trigger 
    Trigger aticiHazirTetik = new Trigger(aticiAltSistemi::isHizaUlasti);

    //  Alm 
    alimTetik
        .whileTrue(new RunCommand(() -> alimAltSistemi.al(), alimAltSistemi))
        .onFalse(new InstantCommand(() -> alimAltSistemi.durdur(), alimAltSistemi));

    geriAtTetik
        .whileTrue(new RunCommand(() -> alimAltSistemi.geriAt(), alimAltSistemi))
        .onFalse(new InstantCommand(() -> alimAltSistemi.durdur(), alimAltSistemi));

    //  Tayc 
    tasiyiciTetik
        .whileTrue(new RunCommand(
            () -> alimAltSistemi.depodanAticiyaYukariTasimaBaslat(), alimAltSistemi))
        .onFalse(new InstantCommand(
            () -> alimAltSistemi.depodanAticiyaYukariTasimaDurdur(), alimAltSistemi));

    // Unjam: 0.5 sn ters evirir, sonra otomatik durur
    tasiyiciTersTetik.onTrue(
        new RunCommand(
            () -> alimAltSistemi.depodanAticiyaYukariTasimaTersBaslat(), alimAltSistemi)
            .withTimeout(0.5)
            .andThen(new InstantCommand(
                () -> alimAltSistemi.depodanAticiyaYukariTasimaDurdur(), alimAltSistemi))
    );

    //  Atc kontrolleri - 3 mesafe iin D-Pad butonlar 
    // Yakn, Orta, Uzak at - shooter belirtilen RPM'e ulanca titreim
    // Tayc manuel olarak Triangle ile alr

    yakinAtisTetik
        .whileTrue(new RunCommand(() -> aticiAltSistemi.atYakin(), aticiAltSistemi))
        .onFalse(new InstantCommand(() -> aticiAltSistemi.durdur(), aticiAltSistemi));

    ortaAtisTetik
        .whileTrue(new RunCommand(() -> aticiAltSistemi.atOrta(), aticiAltSistemi))
        .onFalse(new InstantCommand(() -> aticiAltSistemi.durdur(), aticiAltSistemi));

    uzakAtisTetik
        .whileTrue(new RunCommand(() -> aticiAltSistemi.atUzak(), aticiAltSistemi))
        .onFalse(new InstantCommand(() -> aticiAltSistemi.durdur(), aticiAltSistemi));

    //  Titreim: atc hedefe ulanca bildir 
    aticiHazirTetik
        .onTrue(new InstantCommand(() -> surucuProfili.titrestir(0.6)))
        .onFalse(new InstantCommand(() -> surucuProfili.titrestir(0.0)));

    //  Limelight hizalama → conveyor başlat → 0.3s bekle → orta atış
    limelightHizalaTetik
        .whileTrue(
            new LimelightMerkezlemeKomutu(surusAltSistemi, gorusAltSistemi)
                .andThen(new RunCommand(
                        () -> alimAltSistemi.depodanAticiyaYukariTasimaBaslat(), alimAltSistemi)
                    .withTimeout(0.3))
                .andThen(new ParallelCommandGroup(
                    new RunCommand(() -> alimAltSistemi.depodanAticiyaYukariTasimaBaslat(), alimAltSistemi),
                    new RunCommand(() -> aticiAltSistemi.atOrta(), aticiAltSistemi)
                ))
        )
        .onFalse(new InstantCommand(() -> {
            aticiAltSistemi.durdur();
            alimAltSistemi.depodanAticiyaYukariTasimaDurdur();
        }, aticiAltSistemi, alimAltSistemi));

    //  Gyro sfrla 
    gyroSifirTetik.onTrue(new InstantCommand(
        () -> surusAltSistemi.yonuSifirla(), surusAltSistemi));

    //  Gecikmeli at (Button 10 - Options) 
    // nce shooter' 5000 RPM'e altr, sonra conveyor'u balat
    gecikmeliAtisTetik
        .whileTrue(
            new ParallelCommandGroup(
                new RunCommand(() -> aticiAltSistemi.atRPM(5000.0), aticiAltSistemi),
                new WaitUntilCommand(aticiAltSistemi::isHizaUlasti)
                    .andThen(new WaitCommand(0.2))
                    .andThen(new RunCommand(
                        () -> alimAltSistemi.depodanAticiyaYukariTasimaBaslat(), alimAltSistemi))
            )
        )
        .onFalse(new InstantCommand(() -> {
            aticiAltSistemi.durdur();
            alimAltSistemi.depodanAticiyaYukariTasimaDurdur();
        }, aticiAltSistemi, alimAltSistemi));

    // Taret iptal edildi - robot govdesi limelight ile dogrudan hizalanir
  }

  //  PathPlanner 

  private void pathPlannerKomutlariniKaydet() {
    //  Temel yardmc komutlar 
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

    // Otonom devre disi / bekleme komutu
    NamedCommands.registerCommand(
        "OtonomYapma",
        new WaitCommand(0.1)
    );
  }

  private void otonomSeciciKur() {
    try {
      otonomSecici = AutoBuilder.buildAutoChooser();
      otonomSecici.setDefaultOption(
          "Otonom Yapma",
          new InstantCommand(() -> SmartDashboard.putString("Auto/OzelBaslangic", "YAPMA")));
      otonomSecici.addOption("Ozel Oto - LEFT", ozelOtoKomutuOlustur("LEFT", -45.0));
      otonomSecici.addOption("Ozel Oto - MIDDLE", ozelOtoKomutuOlustur("MIDDLE", 0.0));
      otonomSecici.addOption("Ozel Oto - RIGHT", ozelOtoKomutuOlustur("RIGHT", 45.0));
      SmartDashboard.putData("Auto Chooser", otonomSecici);
    } catch (Exception e) {
      DriverStation.reportError("PathPlanner otonomlari yuklenemedi: " + e.getMessage(), true);
      otonomSecici = new SendableChooser<>();
      otonomSecici.setDefaultOption(
          "Otonom Yapma",
          new InstantCommand(() -> SmartDashboard.putString("Auto/OzelBaslangic", "YAPMA")));
      otonomSecici.addOption("Ozel Oto - LEFT", ozelOtoKomutuOlustur("LEFT", -45.0));
      otonomSecici.addOption("Ozel Oto - MIDDLE", ozelOtoKomutuOlustur("MIDDLE", 0.0));
      otonomSecici.addOption("Ozel Oto - RIGHT", ozelOtoKomutuOlustur("RIGHT", 45.0));
      SmartDashboard.putData("Auto Chooser", otonomSecici);
    }
  }

  private Command ozelOtoKomutuOlustur(String baslangic, double donusDerece) {
    final double hedefMesafeMetre = 1.20;
    final double surusHizi = 0.35;

    Command donusKomutu = Math.abs(donusDerece) < 0.01
        ? new InstantCommand(() -> {}, surusAltSistemi)
        : hedefeDonusKomutu(donusDerece);

    Command ilerlemeKomutu = mesafeIlerleKomutu(hedefMesafeMetre, surusHizi);

    Command atisKomutu = new ParallelCommandGroup(
        new RunCommand(() -> aticiAltSistemi.atOrta(), aticiAltSistemi)
            .withTimeout(10.0),
        new WaitCommand(0.3)
            .andThen(new RunCommand(
                () -> alimAltSistemi.depodanAticiyaYukariTasimaBaslat(), alimAltSistemi)
                .withTimeout(9.7)))
        .finallyDo(() -> {
          aticiAltSistemi.durdur();
          alimAltSistemi.depodanAticiyaYukariTasimaDurdur();
        });

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          SmartDashboard.putString("Auto/OzelBaslangic", baslangic);
          surusAltSistemi.tumMotorlariDurdur();
        }, surusAltSistemi),
        donusKomutu,
        ilerlemeKomutu,
        atisKomutu
    );
  }

  private Command hedefeDonusKomutu(double donusDerece) {
    PIDController donusPid = new PIDController(0.02, 0.0, 0.001);
    donusPid.enableContinuousInput(-180.0, 180.0);
    donusPid.setTolerance(2.0);
    final double[] hedefBaslik = new double[1];

    return new FunctionalCommand(
        () -> {
          donusPid.reset();
          hedefBaslik[0] = surusAltSistemi.getHeading().getDegrees() + donusDerece;
          donusPid.setSetpoint(hedefBaslik[0]);
        },
        () -> {
          double cikis = donusPid.calculate(surusAltSistemi.getHeading().getDegrees());
          cikis = MathUtil.clamp(cikis, -0.35, 0.35);
          surusAltSistemi.drive(0.0, 0.0, cikis);
        },
        interrupted -> surusAltSistemi.drive(0.0, 0.0, 0.0),
        donusPid::atSetpoint,
        surusAltSistemi
    ).withTimeout(2.0);
  }

  private Command mesafeIlerleKomutu(double hedefMesafeMetre, double hiz) {
    final Translation2d[] baslangicPozu = new Translation2d[1];

    return new FunctionalCommand(
        () -> baslangicPozu[0] = surusAltSistemi.getPose().getTranslation(),
        () -> surusAltSistemi.drive(hiz, 0.0, 0.0),
        interrupted -> surusAltSistemi.drive(0.0, 0.0, 0.0),
        () -> {
          double gidilen = surusAltSistemi.getPose().getTranslation().getDistance(baslangicPozu[0]);
          return gidilen >= hedefMesafeMetre;
        },
        surusAltSistemi
    ).withTimeout(3.0);
  }

  //  Periyodik 

  public void periyodik() {
    girdiBaglantiDurumunuGuncelle();
    elasticDurumKontrol();

    SmartDashboard.putString(surucuIstasyonuAnahtari("AktifProfil"), "PS4TamProfili");

    // Ham buton durumlar (debug)
    for (int i = 1; i <= 12; i++) {
      SmartDashboard.putBoolean(surucuIstasyonuAnahtari("Dugme" + i), guvenliDugmeOku(i));
    }

    // Eksen deerleri (debug)
    for (int i = 0; i <= 5; i++) {
      SmartDashboard.putNumber(surucuIstasyonuAnahtari("Eksen" + i), guvenliEksenOku(i));
    }

    SmartDashboard.putNumber(surucuIstasyonuAnahtari("POV"), guvenliPovOku());
  }

  //  Balant / Elastic bildirimleri 

  private void girdiBaglantiDurumunuGuncelle() {
    boolean surucuBagli = DriverStation.isJoystickConnected(OISabitleri.SURUCU_JOYSTICK_PORTU);
    if (surucuBagli != surucuBagliOncekiDurum) {
      surucuBagliOncekiDurum = surucuBagli;
      if (surucuBagli) {
        Elastic.sendNotification(new Notification(NotificationLevel.INFO,
            "Kontrolc Baland",
            "PS4/Xbox port " + OISabitleri.SURUCU_JOYSTICK_PORTU + "'e baland."));
      } else {
        Elastic.sendNotification(new Notification(NotificationLevel.ERROR,
            "Kontrolc Koptu!",
            "Port " + OISabitleri.SURUCU_JOYSTICK_PORTU + " balants kesildi."));
      }
    }
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("KontrolcuBagli"), surucuBagli);

  }

  private void elasticDurumKontrol() {
    if (DriverStation.isEnabled()) {
      // Limelight config hatas
      boolean llOk = gorusAltSistemi.isLimelightConfigOk();
      if (!llOk && !limelightHataBildirimGonderildi) {
        Elastic.sendNotification(new Notification(NotificationLevel.WARNING,
            "Limelight Config Hatas",
            gorusAltSistemi.getLimelightConfigStatus(), 5000));
        limelightHataBildirimGonderildi = true;
      } else if (llOk) {
        limelightHataBildirimGonderildi = false;
      }

      // Atc RPM hazr bildirimi (tek seferlik  titreim de ayrca alr)
      boolean aticiHazir = aticiAltSistemi.isHizaUlasti();
      if (aticiHazir && !aticiHazirBildirimGonderildi) {
        Elastic.sendNotification(new Notification(NotificationLevel.INFO,
            "Atc Hazr",
            String.format("%.0f RPM  at yaplabilir.", aticiAltSistemi.getAktuelRPM()),
            2000));
        aticiHazirBildirimGonderildi = true;
      } else if (!aticiHazir) {
        aticiHazirBildirimGonderildi = false;
      }

    } else {
      limelightHataBildirimGonderildi = false;
      aticiHazirBildirimGonderildi    = false;
    }
  }

  //  Gvenli HID okuma yardmclar 

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

  //  Da ak metodlar 

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




