# TODO.md — 2026 FRC Cross-Check Findings

> Kaynak: 2026 sezonu GitHub repoları, WPILib ornekleri, Chief Delphi, REV ornekleri ve Andymark dokumanları ile capraz kontrol.

---

## 1. KRITIK — Hemen Duzeltilmeli

### 1.1 NavX Yaw Negation Bug
- **Dosya:** `SurusAltSistemi.java` (~satir 579, `getHeading()`)
- **Sorun:** NavX CW-pozitif, WPILib CCW-pozitif kullanir. `Rotation2d.fromDegrees(navx.getYaw())` negatif almadan donduruluyor.
- **Etki:** Alan-yonlu surus ve odometri ters yonde calisiyor.
- **Duzeltme:**
  ```java
  // Yanlis:
  return Rotation2d.fromDegrees(navx.getYaw());
  // Dogru:
  return Rotation2d.fromDegrees(-navx.getYaw());
  ```
- **Kaynak:** AdvantageKit resmi NavX entegrasyonu, Mechanical Advantage

### 1.2 REVLib ve SparkMax Firmware Guncellemesi
- **Mevcut:** REVLib `2026.0.1`
- **Hedef:** REVLib `2026.0.5` + SparkMax firmware `spline-26.1.3`
- **Dosya:** `vendordeps/REVLib.json` ve REV Hardware Client
- **Neden:** 26.1.3 hiz ortalamasi, aci offseti geri yukleme ve LED yonuge hatalarini duzeltiyor.

---

## 2. YUKSEK — Performansi Etkiliyor

### 2.1 SparkMax IdleMode Eksik
- **Dosya:** `SurusAltSistemi.java`, `AticiAltSistemi.java`, `TaretAltSistemi.java`
- **Sorun:** hicbir SparkMax konfigurasyonunda `idleMode` belirtilmemis.
- **Duzeltme:**
  - Surus motorleri → `IdleMode.kBrake` (aninda durma)
  - Atici motoru → `IdleMode.kCoast` (atak momentumuyla serbest donus)
  - Taret motoru → `IdleMode.kBrake` (pozisyon tutma)
  ```java
  reversedConfig.idleMode(IdleMode.kBrake);
  ```

### 2.2 Vision: MegaTag2 ve LimelightHelpers Gecisi
- **Dosya:** `GorusAltSistemi.java`, `SurusAltSistemi.java`
- **Sorunlar:**
  - Ham `botpose_wpiblue` dizisi okunuyor (MegaTag1)
  - `SetRobotOrientation()` hic cagrilmiyor → MegaTag2 kullanilmiyor
  - `addVisionMeasurement()` standart sapma parametresiz cagriliyor
- **2026 Standart:** Tum rekabetci takimlar MegaTag2 kullaniyor (jiro destekli, daha kararli)
- **Duzeltme plani:**
  1. `LimelightHelpers` v1.12 ekle
  2. Her dongude `SetRobotOrientation(name, gyroYaw, ...)` cagir
  3. `getBotPoseEstimate_wpiBlue_MegaTag2()` kullan
  4. Standart sapmalari gec:
     ```java
     VecBuilder.fill(0.1, 0.1, 10.0)  // x, y std dev kucuk; theta std dev buyuk (jiro daha guvenilir)
     ```
- **Kaynak:** `wcpllc/2026CompetitiveConcept`, `frc1678/C2025-Public`

### 2.3 Vision Filtreleri Eksik
- **Sorun:** Hicbir goruntu olcumu reddedilmiyor.
- **Eklenecek filtreler:**
  - **Tag sayisi:** 2+ tag VEYA 1 buyuk tag (tek kucuk tag reddet)
  - **Donus hizi:** Robot >360 deg/s donuyorsa olcumu reddet
  - **Mesafe olcekleme:** `avgTagDist` ile standart sapmalari carp (uzak tag = az guven)
- **Kaynak:** `FRCTeam3255/2025_Robot_Code`, `NewTechProgrammers/2026Rebuilt`

### 2.4 Atici: Velocity PID + Feedforward
- **Dosya:** `AticiAltSistemi.java`
- **Mevcut:** `aticiMotoru.set(0.90)` — yuzde cikti, RPM geri bildirimi yok
- **Hedef:** SparkMax kapali dongu hiz kontrolü
- **Duzeltme:**
  ```java
  // SparkMaxConfig'a ekle:
  config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.0001)
      .i(0)
      .d(0)
      .outputRange(-1, 1)
      .feedForward.kV(12.0 / 5767);  // REV resmi kV degeri
  ;
  // Calistirma:
  closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  ```
- **Onemli:** `feedbackSensor` acikca belirtilmeli. `ControlType.kVelocity` kullanilmali (`kMAXVelocity` DEGIL).
- **Kaynak:** `REVrobotics/REVLib-Examples`, `Earl-Of-March-FRC/2026-7476-Rebuilt`, Chief Delphi FRC4327 konusu

---

## 3. ORTA — Kaliteyi Artirir

### 3.1 Taret: SparkMax Donanim Soft Limitleri
- **Dosya:** `TaretAltSistemi.java`
- **Mevcut:** Yazilim seviyesinde aci sinirlama (`Math.min/max`)
- **Sorun:** RoboRIO kodu cokerse sinirlar kalkar
- **Duzeltme:** SparkMax `softLimit` kullan:
  ```java
  config.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit((float) degreesToMotorRotations(90))
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit((float) degreesToMotorRotations(-90));
  ```
- **Kaynak:** `wavelength3572/Robot-2026`

### 3.2 Taret: MAXMotion Trapezoidal Profil
- **Mevcut:** Sadece P * hata kontrolü
- **Oneri:** SparkMax `ControlType.kMAXMotionPositionControl` ile cruise hiz/ivme sinirlari
- **Fayda:** Daha yumusak tareti hareket, daha az asinma
- **Kaynak:** `Nanuet-Knightronz/2026-Codebase`

### 3.3 Taret: Absolute Encoder ile Baslangic Referansi
- **Mevcut:** Limit switch ile homing (her mac basi)
- **Oneri:** Baslangicta relative enkoderi absolute enkoderden seed et → homing gereksiz
- **Kaynak:** `wavelength3572/Robot-2026`, `Nanuet-Knightronz/2026-Codebase`

### 3.4 Taret: kI Terimi Eksik
- **Mevcut:** `KP = 0.01`, kI = 0, kD = 0
- **Sorun:** Statik suretme hatasi olusabilir (kucuk hatada motor surtunmeyi yenemez)
- **Oneri:** Kucuk bir kI degeri ekle (ornegin 0.001)
- **Kaynak:** `wavelength3572/Robot-2026` TODO yorumu

### 3.5 Surus: Velocity PID + Feedforward (Otonom icin)
- **Dosya:** `SurusAltSistemi.java`
- **Mevcut:** `motor.set(percent)` — batarya voltajina bagli, otonom yolu takip hatasi buyuk
- **Oneri:** `SimpleMotorFeedforward` + `PIDController` ile `setVoltage()` kullan
- **Alternatif:** SparkMax onboard velocity PID
- **Kaynak:** WPILib `mecanumbot` ornegi

### 3.6 Surus: PoseEstimator Standart Sapmalari
- **Dosya:** `SurusAltSistemi.java`
- **Mevcut:** Default constructor (std dev parametresiz)
- **Oneri:**
  ```java
  new MecanumDrivePoseEstimator(
      kinematics, getHeading(), getWheelPositions(), initialPose,
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),  // state std devs
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))    // vision std devs
  );
  ```
- **Kaynak:** WPILib `mecanumdriveposeestimator` ornegi

### 3.7 Voltage Compensation
- **Dosya:** Tum SparkMax konfigürasyonları
- **Oneri:** `.voltageCompensation(12)` ekle → batarya voltaj dususunda tutarli davranis
- **Kaynak:** REV resmi 2025 Starter Bot

---

## 4. DUSUK — Gelecek Planlama

### 4.1 SmartDashboard → Elastic Gecisi
- **Sorun:** SmartDashboard 2027'de kaldirilacak (WPILib 2026 changelog)
- **Oneri:** Mevcut `Elastic.java` alt yapisini kullanarak kademeli gecis yap
- **Aciliyet:** 2026 sezonu icin zorunlu degil, 2027 oncesi planlanmali

### 4.2 Atici: Mesafe-RPM Polinom Egrisi
- **Oneri:** Farkli mesafeler icin hedef RPM belirleme
- **Ornek:** Team 7476 → `RPM = 43.5*d² + 119*d + 2372`
- **Kaynak:** `Earl-Of-March-FRC/2026-7476-Rebuilt`

### 4.3 Atis-Hareket Entegrasyonu (Shoot-While-Moving)
- **Oneri:** Robot hizi ve ivmesini hesaba katarak ongorulu atis
- **Kaynak:** `Earl-Of-March-FRC/2026-7476-Rebuilt` LaunchHelpers.java

### 4.4 Victor SPX / CIM PWM Notlari
- **Mevcut:** `PWMSparkMax` ile PWM kontrolu — dogru yaklasim
- **Not:** CTRE Phoenix kutuphanesi PWM Victor SPX icin gereksiz. Mevcut yapi dogru.

---

## Referans Repoları

| Repo | Konu | URL |
|---|---|---|
| WPILib MecanumBot | Resmi mecanum ornegi | `wpilibsuite/allwpilib` |
| WPILib MecanumPoseEstimator | Vision-fused odometry | `wpilibsuite/allwpilib` |
| REVLib-Examples | SparkMax velocity PID | `REVrobotics/REVLib-Examples` |
| wavelength3572/Robot-2026 | Turret (abs encoder + soft limit) | GitHub |
| Nanuet-Knightronz/2026-Codebase | Turret (MAXMotion) | GitHub |
| Earl-Of-March-FRC/2026-7476-Rebuilt | Tam atis sistemi | GitHub |
| wcpllc/2026CompetitiveConcept | MegaTag2 vision | GitHub |
| frc1678/C2025-Public | Mesafe-olcekli vision std dev | GitHub |
| FRCTeam3255/2026_Robot_Code | Cok-kamerali vision oncelik | GitHub |
| FRC-Team-3140/2026-Rebuilt-Bot | Turret + onguru atis | GitHub |
| FRC4048/Java_2026 | Cift-slot PID turret | GitHub |
| REV 2025 Starter Bot | SparkMaxConfig ornekleri | `REVrobotics/2025-REV-ION-FRC-Starter-Bot` |

---

*Son guncelleme: 2026-03-29 — 6 paralel arastirma ajenti ile olusturuldu*

  NavX Yaw Negation (SurusAltSistemi.java line 579):
  // Current (WRONG):
  return Rotation2d.fromDegrees(navx.getYaw());
  // Should be:
  return Rotation2d.fromDegrees(-navx.getYaw());
  NavX uses CW-positive rotation, WPILib uses CCW-positive. Without negation, field-oriented drive
  and odometry have inverted heading. Confirmed by AdvantageKit's official NavX integration and
  Mechanical Advantage.

  ---
  Mecanum Drive

  ┌─────────────────────┬────────────────────────────┬──────────────────────────────┬──────────┐
  │       Aspect        │         Your Code          │      2026 Best Practice      │  Status  │
  ├─────────────────────┼────────────────────────────┼──────────────────────────────┼──────────┤
  │ Spark Max API       │ New com.revrobotics.spark  │ Same                         │ Correct  │
  ├─────────────────────┼────────────────────────────┼──────────────────────────────┼──────────┤
  │ Motor inversion     │ Left side inverted,        │ Either side OK if consistent │ Correct  │
  │                     │ consistent                 │                              │          │
  ├─────────────────────┼────────────────────────────┼──────────────────────────────┼──────────┤
  │ Deadband            │ 0.08 + quadratic curve     │ 0.04 built-in + custom       │ Good     │
  ├─────────────────────┼────────────────────────────┼──────────────────────────────┼──────────┤
  │ PoseEstimator       │ With vision fusion         │ Same                         │ Correct  │
  ├─────────────────────┼────────────────────────────┼──────────────────────────────┼──────────┤
  │ Slew rate limiting  │ Yes (1.0/2.0)              │ SlewRateLimiter(3) typical   │ Good     │
  ├─────────────────────┼────────────────────────────┼──────────────────────────────┼──────────┤
  │ Drive PID/FF        │ Raw set() percent          │ PID + feedforward +          │ Gap      │
  │                     │                            │ setVoltage()                 │          │
  ├─────────────────────┼────────────────────────────┼──────────────────────────────┼──────────┤
  │ ChassisSpeeds       │ Not used                   │ Standard in advanced         │ Minor    │
  │ discretize          │                            │ examples                     │ gap      │
  ├─────────────────────┼────────────────────────────┼──────────────────────────────┼──────────┤
  │ PoseEstimator std   │ Default (none)             │ VecBuilder.fill(0.05, 0.05,  │ Improve  │
  │ devs                │                            │ 5deg)                        │          │
  ├─────────────────────┼────────────────────────────┼──────────────────────────────┼──────────┤
  │ IdleMode            │ Not set                    │ IdleMode.kBrake for drive    │ Missing  │
  └─────────────────────┴────────────────────────────┴──────────────────────────────┴──────────┘

  Key gap: Autonomous path accuracy will suffer without velocity PID + feedforward. WPILib
  mecanumbot example uses PIDController + SimpleMotorFeedforward with setVoltage() for all drive
  motors.

  ---
  Limelight / AprilTag Vision

  ┌───────────────────┬────────────────────┬─────────────────────────────────────────┬──────────┐
  │      Aspect       │     Your Code      │           2026 Best Practice            │  Status  │
  ├───────────────────┼────────────────────┼─────────────────────────────────────────┼──────────┤
  │ NT access         │ Raw getDouble()    │ LimelightHelpers v1.12                  │ Improve  │
  ├───────────────────┼────────────────────┼─────────────────────────────────────────┼──────────┤
  │ Pose mode         │ Raw botpose        │ MegaTag2 with SetRobotOrientation()     │ Major    │
  │                   │ (MegaTag1)         │                                         │ gap      │
  ├───────────────────┼────────────────────┼─────────────────────────────────────────┼──────────┤
  │ Vision std devs   │ Not passed         │ VecBuilder.fill(0.1, 0.1, 10) or        │ Missing  │
  │                   │                    │ distance-scaled                         │          │
  ├───────────────────┼────────────────────┼─────────────────────────────────────────┼──────────┤
  │ Tag count filter  │ None               │ 2+ tags OR 1 big tag                    │ Missing  │
  ├───────────────────┼────────────────────┼─────────────────────────────────────────┼──────────┤
  │ Rotation rate     │ None               │ Reject when spinning >360 deg/s         │ Missing  │
  │ reject            │                    │                                         │          │
  ├───────────────────┼────────────────────┼─────────────────────────────────────────┼──────────┤
  │ Latency           │ Manual (correct)   │ Auto via LimelightHelpers               │ OK       │
  │ compensation      │                    │                                         │          │
  ├───────────────────┼────────────────────┼─────────────────────────────────────────┼──────────┤
  │ Ambiguity filter  │ < 0.2              │ Same                                    │ Correct  │
  └───────────────────┴────────────────────┴─────────────────────────────────────────┴──────────┘

  Major finding: All competitive 2026 teams use MegaTag2 (getBotPoseEstimate_wpiBlue_MegaTag2) which
   is gyro-assisted and more stable. Your code reads raw botpose_wpiblue array (MegaTag1) and never
  calls SetRobotOrientation(). Teams like frc1678 scale standard deviations by avgTagDist for
  distance-based trust.

  ---
  Turret (Andymark 6.875" AM-3936)

  ┌────────────────┬───────────────────────┬─────────────────────────────────────┬────────────┐
  │     Aspect     │       Your Code       │         2026 Best Practice          │   Status   │
  ├────────────────┼───────────────────────┼─────────────────────────────────────┼────────────┤
  │ PID            │ KP = 0.01, roboRIO    │ 0.01–0.6 or SparkMax onboard        │ OK (low    │
  │                │ WPILib                │                                     │ end)       │
  ├────────────────┼───────────────────────┼─────────────────────────────────────┼────────────┤
  │ Soft limits    │ Software-only         │ SparkMax hardware softLimit         │ Improve    │
  │                │ clamping              │                                     │            │
  ├────────────────┼───────────────────────┼─────────────────────────────────────┼────────────┤
  │ Motion         │ None                  │ MAXMotion trapezoidal               │ Missing    │
  │ profiling      │                       │                                     │            │
  ├────────────────┼───────────────────────┼─────────────────────────────────────┼────────────┤
  │ Absolute       │ Limit switch homing   │ Absolute encoder seeding at startup │ Improve    │
  │ encoder        │                       │                                     │            │
  ├────────────────┼───────────────────────┼─────────────────────────────────────┼────────────┤
  │ Angle          │ None                  │ Shortest-path unwrap within limits  │ Missing    │
  │ unwrapping     │                       │                                     │            │
  ├────────────────┼───────────────────────┼─────────────────────────────────────┼────────────┤
  │ kI term        │ None                  │ Add small kI for friction           │ Missing    │
  │                │                       │ steady-state error                  │            │
  └────────────────┴───────────────────────┴─────────────────────────────────────┴────────────┘

  Key finding: Team Nanuet uses ControlType.kMAXMotionTrapezoidal on SparkMax for smooth turret
  movement with cruise velocity/acceleration limits. Team wavelength3572 seeds the relative encoder
  from an absolute encoder at power-on, eliminating homing routines.

  ---
  Shooter (Andymark Launcher in a Box AM-5780)

  ┌────────────────────┬───────────────────┬───────────────────────────────────────┬───────────┐
  │       Aspect       │     Your Code     │          2026 Best Practice           │  Status   │
  ├────────────────────┼───────────────────┼───────────────────────────────────────┼───────────┤
  │ Control            │ set(0.90) percent │ Velocity PID + feedforward            │ Major gap │
  ├────────────────────┼───────────────────┼───────────────────────────────────────┼───────────┤
  │ RPM measurement    │ None              │ encoder.getVelocity() for RPM         │ Missing   │
  ├────────────────────┼───────────────────┼───────────────────────────────────────┼───────────┤
  │ Idle mode          │ Not set           │ kCoast for flywheel                   │ Missing   │
  ├────────────────────┼───────────────────┼───────────────────────────────────────┼───────────┤
  │ FF                 │ None              │ kV = 12.0/5676 ≈ 0.002 (REV official) │ Missing   │
  ├────────────────────┼───────────────────┼───────────────────────────────────────┼───────────┤
  │ Distance-based RPM │ None              │ Polynomial: 43.5d² + 119d + 2372      │ Missing   │
  └────────────────────┴───────────────────┴───────────────────────────────────────┴───────────┘

  Key finding: REV's official 2026 example uses ControlType.kVelocity with ClosedLoopSlot.kSlot1, kV
   = 12.0/5767, and P = 0.0001. Team 7476 (2026) has a complete shoot-while-moving implementation
  with distance-to-RPM polynomial curves.

  Critical 2026 pitfall: Must explicitly set feedbackSensor(FeedbackSensor.kPrimaryEncoder) and use
  ControlType.kVelocity (NOT kMAXVelocity).

  ---
  NavX + Spark Max Configuration

  ┌───────────────────┬────────────────────────────────┬───────────────────────────────────────┐
  │       Item        │             Status             │             Action Needed             │
  ├───────────────────┼────────────────────────────────┼───────────────────────────────────────┤
  │ NavX library      │ Correct (com.studica.frc.AHRS  │ None                                  │
  │                   │ v2026)                         │                                       │
  ├───────────────────┼────────────────────────────────┼───────────────────────────────────────┤
  │ NavX yaw sign     │ BUG — not negated              │ Fix immediately                       │
  ├───────────────────┼────────────────────────────────┼───────────────────────────────────────┤
  │ REVLib version    │ 2026.0.1                       │ Update to 2026.0.5                    │
  ├───────────────────┼────────────────────────────────┼───────────────────────────────────────┤
  │ SparkMax firmware │ Unknown                        │ Update to spline-26.1.3               │
  ├───────────────────┼────────────────────────────────┼───────────────────────────────────────┤
  │ IdleMode on drive │ Missing                        │ Add IdleMode.kBrake                   │
  ├───────────────────┼────────────────────────────────┼───────────────────────────────────────┤
  │ IdleMode on       │ Missing                        │ Add IdleMode.kCoast                   │
  │ shooter           │                                │                                       │
  ├───────────────────┼────────────────────────────────┼───────────────────────────────────────┤
  │ IdleMode on       │ Missing                        │ Add IdleMode.kBrake                   │
  │ turret            │                                │                                       │
  ├───────────────────┼────────────────────────────────┼───────────────────────────────────────┤
  │ Voltage           │ Missing                        │ Add .voltageCompensation(12)          │
  │ compensation      │                                │                                       │
  ├───────────────────┼────────────────────────────────┼───────────────────────────────────────┤
  │ SmartDashboard    │ Used extensively               │ Deprecating in 2027 — plan Elastic    │
  │                   │                                │ migration                             │
  └───────────────────┴────────────────────────────────┴───────────────────────────────────────┘

  ---
  Priority Action Items

  Immediate (affects correctness):
  1. Fix NavX yaw negation in getHeading()
  2. Update REVLib to 2026.0.5 + SparkMax firmware to spline-26.1.3

  High (affects performance):
  3. Add IdleMode to all SparkMax configs
  4. Switch vision to LimelightHelpers + MegaTag2 + pass std devs to addVisionMeasurement
  5. Add velocity PID + feedforward to shooter (NEO encoder → RPM control)

  Medium (improves quality):
  6. Add SparkMax hardware soft limits for turret
  7. Add drive velocity PID/FF for autonomous accuracy
  8. Add vision rejection filters (tag count, rotation rate, distance)
  9. Consider MAXMotion trapezoidal for turret positioning

  Reference repos found:
  - wpilibsuite/allwpilib — official mecanum examples
  - wavelength3572/Robot-2026 — turret with absolute encoder seeding
  - Nanuet-Knightronz/2026-Codebase — MAXMotion turret
  - Earl-Of-March-FRC/2026-7476-Rebuilt — full shooter with shoot-while-moving
  - wcpllc/2026CompetitiveConcept — advanced MegaTag2 vision
  - frc1678/C2025-Public — distance-scaled vision std devs
  - REVrobotics/REVLib-Examples — official 2026 velocity PID