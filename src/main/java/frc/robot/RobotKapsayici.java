// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  public RobotKapsayici() {
    // Gr alt sistemi nce kurulur  sre poz fuzyonu iin gerekli
    gorusAltSistemi = new GorusAltSistemi();
    surusAltSistemi = new SurusAltSistemi(
        MotorSabitleri.ON_SOL_MOTOR_ID,
        MotorSabitleri.ON_SAG_MOTOR_ID,
        MotorSabitleri.ARKA_SOL_MOTOR_ID,
        MotorSabitleri.ARKA_SAG_MOTOR_ID
    );
    alimAltSistemi   = new AlimAltSistemi();
    aticiAltSistemi  = new AticiAltSistemi();
    defaultKomutlariniKur();
    baglamalariYapilandir();
    // otonomSeciciKur();

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
    Trigger yakinAtisTetik   = new Trigger(() -> surucuProfili.yakinAtisBasili());
    Trigger ortaAtisTetik   = new Trigger(() -> surucuProfili.ortaAtisBasili());
    Trigger uzakAtisTetik   = new Trigger(() -> surucuProfili.uzakAtisBasili());
    Trigger cokUzakAtisTetik = new Trigger(() -> surucuProfili.cokUzakAtisBasili());
    Trigger limelightHizalaTetik  = new Trigger(() -> surucuKontrolcusu.getRawButton(8));
    Trigger gecikmeliAtisTetik    = new Trigger(() -> surucuProfili.gecikmeliAtisBasili());
    Trigger shooterDirektTetik    = new Trigger(() -> surucuProfili.shooterDirektBasili());

    //  Atc hazr Trigger 
    Trigger sikisikGiderTetik = new Trigger(() -> surucuKontrolcusu.getRawButton(5));
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

    //  D-Pad: Manuel atış (4 hız) — hizalama yok, atıcı ısın → RPM'e ulaş → konveyör
    yakinAtisTetik
        .whileTrue(manuelAtisKomutu(() -> aticiAltSistemi.atYakin()))
        .onFalse(atisTemizleKomutu());

    ortaAtisTetik
        .whileTrue(manuelAtisKomutu(() -> aticiAltSistemi.atOrta()))
        .onFalse(atisTemizleKomutu());

    uzakAtisTetik
        .whileTrue(manuelAtisKomutu(() -> aticiAltSistemi.atUzak()))
        .onFalse(atisTemizleKomutu());

    cokUzakAtisTetik
        .whileTrue(manuelAtisKomutu(() -> aticiAltSistemi.atCokUzak()))
        .onFalse(atisTemizleKomutu());

    //  L1: Sıkışık top giderici — 0.25 s sağ + 0.25 s sol strafe
    sikisikGiderTetik.onTrue(
        new SequentialCommandGroup(
            new RunCommand(() -> surusAltSistemi.drive(0, 0.5, 0), surusAltSistemi).withTimeout(0.25),
            new RunCommand(() -> surusAltSistemi.drive(0, -0.5, 0), surusAltSistemi).withTimeout(0.25),
            new InstantCommand(() -> surusAltSistemi.drive(0, 0, 0), surusAltSistemi)
        )
    );

    //  Titreim: atc hedefe ulanca bildir
    aticiHazirTetik
        .onTrue(new InstantCommand(() -> surucuProfili.titrestir(0.6)))
        .onFalse(new InstantCommand(() -> surucuProfili.titrestir(0.0)));

    //  R2: Hizala → RPM'e ulaş → Konveyör
    limelightHizalaTetik
        .whileTrue(
            new SequentialCommandGroup(
                // 1. Hizala + ısın (max 3 s; hizalanınca veya timeout'ta race biter)
                new ParallelRaceGroup(
                    new LimelightMerkezlemeKomutu(surusAltSistemi, gorusAltSistemi).withTimeout(3.0),
                    new RunCommand(
                        () -> aticiAltSistemi.atMesafeyeGore(gorusAltSistemi.getMesafeHedef()),
                        aticiAltSistemi)
                ),
                // 2. RPM'e ulaşana kadar bekle — shooter çalışmaya devam eder (max 2 s)
                new ParallelRaceGroup(
                    new WaitUntilCommand(aticiAltSistemi::isHizaUlasti).withTimeout(2.0),
                    new RunCommand(
                        () -> aticiAltSistemi.atMesafeyeGore(gorusAltSistemi.getMesafeHedef()),
                        aticiAltSistemi)
                ),
                // 3. Atıcı + konveyör döngüsü (2 sn çalış, 0.5 sn dur) — düğme bırakılana kadar
                new ParallelCommandGroup(
                    new RunCommand(
                        () -> aticiAltSistemi.atMesafeyeGore(gorusAltSistemi.getMesafeHedef()),
                        aticiAltSistemi),
                    new RepeatCommand(
                        new SequentialCommandGroup(
                            new RunCommand(
                                () -> alimAltSistemi.depodanAticiyaYukariTasimaBaslat(),
                                alimAltSistemi).withTimeout(1.0),
                            new InstantCommand(
                                () -> alimAltSistemi.depodanAticiyaYukariTasimaDurdur(),
                                alimAltSistemi)
                                .andThen(new WaitCommand(0.5))
                        )
                    )
                )
            )
        )
        .onFalse(new InstantCommand(() -> {
            aticiAltSistemi.durdur();
            alimAltSistemi.depodanAticiyaYukariTasimaDurdur();
        }, aticiAltSistemi, alimAltSistemi));

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

    //  Touchpad: Manuel shooter (konveyör yok)
    shooterDirektTetik
        .whileTrue(new RunCommand(
            () -> aticiAltSistemi.atRPM(ModulSabitleri.ATICI_HEDEF_RPM), aticiAltSistemi))
        .onFalse(new InstantCommand(() -> aticiAltSistemi.durdur(), aticiAltSistemi));
  }

  /** Atıcı ısın → RPM'e ulaş (max 3s) → konveyör + atıcı birlikte. */
  private Command manuelAtisKomutu(Runnable atisMetodu) {
    return new ParallelCommandGroup(
        new RunCommand(atisMetodu::run, aticiAltSistemi),
        new SequentialCommandGroup(
            new WaitUntilCommand(aticiAltSistemi::isHizaUlasti).withTimeout(3.0),
            new RunCommand(
                () -> alimAltSistemi.depodanAticiyaYukariTasimaBaslat(), alimAltSistemi)
        )
    );
  }

  private Command atisTemizleKomutu() {
    return new InstantCommand(() -> {
        aticiAltSistemi.durdur();
        alimAltSistemi.depodanAticiyaYukariTasimaDurdur();
    }, aticiAltSistemi, alimAltSistemi);
  }

  private void otonomSeciciKur() {
    otonomSecici = new SendableChooser<>();
    otonomSecici.setDefaultOption("Geri Git + Hizala + At", geriGitHizalaAtisKomutu());
    SmartDashboard.putData("Auto Chooser", otonomSecici);
  }

  /**
   * Otonom:
   *   1. Hedef AprilTag görünene kadar geri git (max 2 m / ~1.9 s)
   *   2. Crosshair'a göre hedefe hizalan (max 4 s)
   *   3. Mesafeye göre RPM'e ulaş → konveyör başlat → at (max 10 s)
   */
  private Command geriGitHizalaAtisKomutu() {
    // 1. Tag görünene kadar geri — isHedefTagGorunuyor() biter ya da 2m timeout
    double maxSureS = 2.0 / (0.35 * SurusSabitleri.MAKS_HIZ_METRE_SANIYE);
    Command geriKomutu = new FunctionalCommand(
        () -> {},
        () -> surusAltSistemi.drive(-0.35, 0.0, 0.0),
        interrupted -> surusAltSistemi.drive(0.0, 0.0, 0.0),
        gorusAltSistemi::isHedefTagGorunuyor,
        surusAltSistemi
    ).withTimeout(maxSureS);

    // 2. Crosshair'a göre dön
    Command hizalaKomutu = new LimelightMerkezlemeKomutu(surusAltSistemi, gorusAltSistemi)
        .withTimeout(4.0);

    // 3. Mesafeye göre atış: ısın → RPM'e ulaş (max 2s) → konveyör
    Command atisKomutu = new ParallelCommandGroup(
        new RunCommand(
            () -> aticiAltSistemi.atMesafeyeGore(gorusAltSistemi.getMesafeHedef()),
            aticiAltSistemi),
        new WaitUntilCommand(aticiAltSistemi::isHizaUlasti)
            .withTimeout(2.0)
            .andThen(new RunCommand(
                () -> alimAltSistemi.depodanAticiyaYukariTasimaBaslat(), alimAltSistemi))
    ).withTimeout(10.0)
     .finallyDo(() -> {
        aticiAltSistemi.durdur();
        alimAltSistemi.depodanAticiyaYukariTasimaDurdur();
     });

    return new SequentialCommandGroup(
        geriKomutu,
        hizalaKomutu,
        atisKomutu
    );
  }

  //  Periyodik 

  public void periyodik() {
    girdiBaglantiDurumunuGuncelle();
    elasticDurumKontrol();
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
    SmartDashboard.putBoolean("SurucuIstasyonu_KontrolcuBagli", surucuBagli);

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

  //  Da ak metodlar

  public void sensorleriSifirla() {
    // Gyro ve odometri kaldırıldı — sıfırlanacak sensör yok
  }

  public Command otonomKomutAl() {
    return null; // otonomSecici.getSelected();
  }

  public GorusAltSistemi gorusAltSistemiAl() {
    return gorusAltSistemi;
  }
}




