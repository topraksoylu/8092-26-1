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
  private final AlimAltSistemi    alimAltSistemi;
  private final AticiAltSistemi   aticiAltSistemi;

  // ── Kontrolcü ─────────────────────────────────────────────────────────────
  private final GenericHID surucuKontrolcusu = new GenericHID(OISabitleri.SURUCU_JOYSTICK_PORTU);
  private final GenericHID operatorKontrolcusu = new GenericHID(OISabitleri.OPERATOR_JOYSTICK_PORTU);

  private enum SurucuModu {
    TEK_SURUCU,
    IKI_SURUCU
  }

  /**
   * Elastic / SmartDashboard'dan runtime'da seçilebilir kontrolcü profili.
   * Yeni kol takılınca veya kol değişince deploy gerekmez.
   */
  private final SendableChooser<KontrolcuProfili> profilSecici = new SendableChooser<>();
  private final SendableChooser<KontrolcuProfili> operatorProfilSecici = new SendableChooser<>();
  private final SendableChooser<SurucuModu> surucuModuSecici = new SendableChooser<>();

  /**
   * Limelight hedef görmediğinde kullanılacak manuel atış mesafe ön ayarı.
   * Elastic'ten maç öncesinde veya sırasında seçilebilir.
   */
  private final SendableChooser<Sabitler.ManuelAtisModu> atisModuSecici = new SendableChooser<>();

  // ── Durum izleme ──────────────────────────────────────────────────────────
  private boolean surucuBagliOncekiDurum        = false;
  private boolean operatorBagliOncekiDurum      = false;
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
    alimAltSistemi   = new AlimAltSistemi();
    aticiAltSistemi  = new AticiAltSistemi();

    kontrolcuProfilleriniKur();
    atisModuSeciciKur();
    defaultKomutlariniKur();
    baglamalariYapilandir();
    pathPlannerKomutlariniKaydet();
    otonomSeciciKur();

    System.out.println("===================================================");
    System.out.println("ROBOT KAPSAYICI BASLATILDI");
    System.out.println("Surucu Kontrolcu Portu: " + OISabitleri.SURUCU_JOYSTICK_PORTU);
    System.out.println("Operator Kontrolcu Portu: " + OISabitleri.OPERATOR_JOYSTICK_PORTU);
    System.out.println("Kontrol modu Elastic'ten secilir: 'Kontrolcu/SurucuModu'");
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

    operatorProfilSecici.setDefaultOption(
        "PS4 Tam (Axis 4/5 analog)",  new PS4TamProfili(operatorKontrolcusu));
    operatorProfilSecici.addOption(
        "PS4 Basit (analog trigger yok)", new PS4BasitProfili(operatorKontrolcusu));
    operatorProfilSecici.addOption(
        "Xbox",                        new XboxProfili(operatorKontrolcusu));
    SmartDashboard.putData("Kontrolcu/OperatorProfil", operatorProfilSecici);

    surucuModuSecici.setDefaultOption("Tek Surucu", SurucuModu.TEK_SURUCU);
    surucuModuSecici.addOption("Iki Surucu", SurucuModu.IKI_SURUCU);
    SmartDashboard.putData("Kontrolcu/SurucuModu", surucuModuSecici);
  }

  private void atisModuSeciciKur() {
    // Varsayılan: orta mesafe — en sık kullanılan senaryo
    atisModuSecici.setDefaultOption(
        Sabitler.ManuelAtisModu.ORTA.etiket, Sabitler.ManuelAtisModu.ORTA);
    atisModuSecici.addOption(
        Sabitler.ManuelAtisModu.YAKIN.etiket, Sabitler.ManuelAtisModu.YAKIN);
    atisModuSecici.addOption(
        Sabitler.ManuelAtisModu.UZAK.etiket,  Sabitler.ManuelAtisModu.UZAK);
    SmartDashboard.putData("Atici/ManuelMod", atisModuSecici);
  }

  /** Surucu profili null-safe döndürür; seçim yoksa PS4 Tam fallback. */
  private KontrolcuProfili surucuProfili() {
    KontrolcuProfili secili = profilSecici.getSelected();
    // SendableChooser setDefaultOption sonrası null olmaz, yine de güvenli
    return secili != null ? secili : new PS4TamProfili(surucuKontrolcusu);
  }

  /** Operator profili null-safe döndürür; seçim yoksa PS4 Tam fallback. */
  private KontrolcuProfili operatorProfili() {
    KontrolcuProfili secili = operatorProfilSecici.getSelected();
    return secili != null ? secili : new PS4TamProfili(operatorKontrolcusu);
  }

  private SurucuModu aktifSurucuModu() {
    SurucuModu secili = surucuModuSecici.getSelected();
    return secili != null ? secili : SurucuModu.TEK_SURUCU;
  }

  private KontrolcuProfili atisKonveyorProfili() {
    return aktifSurucuModu() == SurucuModu.IKI_SURUCU ? operatorProfili() : surucuProfili();
  }

  // ── Default komutlar ──────────────────────────────────────────────────────

  private void defaultKomutlariniKur() {
    // Sürüş — profil lambdaları her döngüde getSelected() çağırır
    surusAltSistemi.setDefaultCommand(
        new SurusKomutu(
            () -> surucuProfili().yanal(),
            () -> surucuProfili().ileriGeri(),
            () -> surucuProfili().donus(),
            surusAltSistemi
        )
    );

    // Taret varsayılan komutu yok - sadece manuel kontrol (L1/R1/L2)
  }

  // ── Buton bağlamaları ─────────────────────────────────────────────────────

  private void baglamalariYapilandir() {

    // ── Profil tabanlı Trigger'lar ─────────────────────────────────────────
    // Her Trigger, profil değişince otomatik yeni profili kullanır (lambda capture)

    // Intake/Reverse butonlari profile'den bagimsiz sabit: 1 ve 2
    Trigger alimTetik         = new Trigger(() -> surucuKontrolcusu.getRawButton(1));
    Trigger geriAtTetik       = new Trigger(() -> surucuKontrolcusu.getRawButton(2));
    Trigger tasiyiciTetik     = new Trigger(() -> atisKonveyorProfili().tasiyiciBasili());
    Trigger tasiyiciTersTetik = new Trigger(() -> atisKonveyorProfili().tasiyiciTersBasili());
    Trigger yakinAtisTetik    = new Trigger(() -> atisKonveyorProfili().yakinAtisBasili());
    Trigger ortaAtisTetik    = new Trigger(() -> atisKonveyorProfili().ortaAtisBasili());
    Trigger uzakAtisTetik    = new Trigger(() -> atisKonveyorProfili().uzakAtisBasili());
    Trigger limelightHizalaTetik = new Trigger(() -> surucuKontrolcusu.getRawButton(8));
    Trigger gyroSifirTetik    = new Trigger(() -> surucuProfili().gyroSifirlaBasili());
    Trigger gecikmeliAtisTetik = new Trigger(() -> atisKonveyorProfili().gecikmeliAtisBasili());

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

    // ── Atıcı kontrolleri - 3 mesafe için D-Pad butonları ─────────────────
    // Yakın, Orta, Uzak atış - shooter belirtilen RPM'e ulaşınca titreşim
    // Taşıyıcı manuel olarak Triangle ile çalışır

    yakinAtisTetik
        .whileTrue(new RunCommand(() -> aticiAltSistemi.atYakin(), aticiAltSistemi))
        .onFalse(new InstantCommand(() -> aticiAltSistemi.durdur(), aticiAltSistemi));

    ortaAtisTetik
        .whileTrue(new RunCommand(() -> aticiAltSistemi.atOrta(), aticiAltSistemi))
        .onFalse(new InstantCommand(() -> aticiAltSistemi.durdur(), aticiAltSistemi));

    uzakAtisTetik
        .whileTrue(new RunCommand(() -> aticiAltSistemi.atUzak(), aticiAltSistemi))
        .onFalse(new InstantCommand(() -> aticiAltSistemi.durdur(), aticiAltSistemi));

    // ── Titreşim: atıcı hedefe ulaşınca bildir ────────────────────────────
    aticiHazirTetik
        .onTrue(new InstantCommand(() -> atisKonveyorProfili().titrestir(0.6)))
        .onFalse(new InstantCommand(() -> atisKonveyorProfili().titrestir(0.0)));

    // ── Limelight ile robotu hedefe ortala (Tag 9/10/25/26) ───────────────
    limelightHizalaTetik.whileTrue(new LimelightMerkezlemeKomutu(surusAltSistemi, gorusAltSistemi));

    // ── Gyro sıfırla ──────────────────────────────────────────────────────
    gyroSifirTetik.onTrue(new InstantCommand(
        () -> surusAltSistemi.yonuSifirla(), surusAltSistemi));

    // ── Gecikmeli atış (Button 10 - Options) ────────────────────────────────
    // Önce shooter'ı 5000 RPM'e çalıştır, sonra conveyor'u başlat
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

  // ── PathPlanner ───────────────────────────────────────────────────────────

  private void pathPlannerKomutlariniKaydet() {
    // ── Temel yardımcı komutlar ──────────────────────────────────────────
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

  // ── Periyodik ─────────────────────────────────────────────────────────────

  public void periyodik() {
    girdiBaglantiDurumunuGuncelle();
    elasticDurumKontrol();

    // Aktif profil adını dashboard'a yaz
    KontrolcuProfili aktifProfil = profilSecici.getSelected();
    KontrolcuProfili aktifOperatorProfil = operatorProfilSecici.getSelected();
    SmartDashboard.putString(surucuIstasyonuAnahtari("AktifProfil"),
        aktifProfil != null ? aktifProfil.getClass().getSimpleName() : "?");
    SmartDashboard.putString("OperatorIstasyonu_AktifProfil",
        aktifOperatorProfil != null ? aktifOperatorProfil.getClass().getSimpleName() : "?");
    SmartDashboard.putString("Kontrolcu/AktifSurucuModu", aktifSurucuModu().name());

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
    // 1. Öncelik: Limelight hedefi görüyor → interpolasyon tablosuyla hassas RPM
    if (gorusAltSistemi.isHedefTagGorunuyor()) {
      double mesafeMetre = gorusAltSistemi.getDistanceToTarget();
      double hedefRpm    = AtisHesaplayici.hesaplaHedefRpm(mesafeMetre);
      aticiAltSistemi.atRPM(hedefRpm);
      SmartDashboard.putString("Atici/AtisModu",       "Limelight_Otomatik");
      SmartDashboard.putNumber("Atici/HedefMesafe_m",  mesafeMetre);
      SmartDashboard.putNumber("Atici/HesaplananRPM",  hedefRpm);
      return;
    }

    // 2. Fallback: Limelight yok → sürücünün Elastic'ten seçtiği manuel ön ayar
    Sabitler.ManuelAtisModu mod = atisModuSecici.getSelected();
    if (mod == null) mod = Sabitler.ManuelAtisModu.ORTA; // null-safe koruma

    aticiAltSistemi.atRPM(mod.rpm);
    SmartDashboard.putString("Atici/AtisModu",      "Manuel_" + mod.name());
    SmartDashboard.putNumber("Atici/HedefMesafe_m", Double.NaN); // mesafe bilinmiyor
    SmartDashboard.putNumber("Atici/HesaplananRPM", mod.rpm);
  }

  private void aticiyiSabitHizdaCalistir() {
    aticiAltSistemi.atRPM(ModulSabitleri.ATICI_HEDEF_RPM);
    SmartDashboard.putString("Atici/AtisModu", "SabitRPM");
    SmartDashboard.putNumber("Atici/HedefMesafe_m", Double.NaN);
    SmartDashboard.putNumber("Atici/HesaplananRPM", ModulSabitleri.ATICI_HEDEF_RPM);
  }

  // ── Bağlantı / Elastic bildirimleri ──────────────────────────────────────

  private void girdiBaglantiDurumunuGuncelle() {
    boolean surucuBagli = DriverStation.isJoystickConnected(OISabitleri.SURUCU_JOYSTICK_PORTU);
    if (surucuBagli != surucuBagliOncekiDurum) {
      surucuBagliOncekiDurum = surucuBagli;
      if (surucuBagli) {
        Elastic.sendNotification(new Notification(NotificationLevel.INFO,
            "Kontrolcü Bağlandı",
            "PS4/Xbox port " + OISabitleri.SURUCU_JOYSTICK_PORTU + "'e bağlandı."));
      } else {
        Elastic.sendNotification(new Notification(NotificationLevel.ERROR,
            "Kontrolcü Koptu!",
            "Port " + OISabitleri.SURUCU_JOYSTICK_PORTU + " bağlantısı kesildi."));
      }
    }
    SmartDashboard.putBoolean(surucuIstasyonuAnahtari("KontrolcuBagli"), surucuBagli);

    boolean operatorBagli = DriverStation.isJoystickConnected(OISabitleri.OPERATOR_JOYSTICK_PORTU);
    if (operatorBagli != operatorBagliOncekiDurum) {
      operatorBagliOncekiDurum = operatorBagli;
      if (operatorBagli) {
        Elastic.sendNotification(new Notification(NotificationLevel.INFO,
            "Operator Kontrolcu Baglandi",
            "PS4/Xbox port " + OISabitleri.OPERATOR_JOYSTICK_PORTU + "'e baglandi."));
      } else {
        Elastic.sendNotification(new Notification(NotificationLevel.WARNING,
            "Operator Kontrolcu Koptu",
            "Port " + OISabitleri.OPERATOR_JOYSTICK_PORTU + " baglantisi kesildi."));
      }
    }
    SmartDashboard.putBoolean("OperatorIstasyonu_KontrolcuBagli", operatorBagli);
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


