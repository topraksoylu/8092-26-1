# CLAUDE.md

Bu dosya, bu depodaki kodlarla çalışırken Claude Code (claude.ai/code) için rehberlik sağlar.

## Proje Genel Bakış

Bu, 2026 sezonu için bir FRC (FIRST Robotics Competition) robot kod projesidir. Komut tabanlı programlama modelini kullanan Java tabanlı bir WPILib projesidir.

### Temel Donanım

- **Sürücü Sistemi**: CAN veriyolu üzerinden REV Spark MAX kontrolörleri tarafından kontrol edilen NEO fırçasız motorlarla 4 tekerlekli mekanum sürüş
- **Jiroskop**: Alan odaklı sürüş için NavX (Studica)
- **Görüntüleme**: AprilTag hedef takibi için Limelight 3
- **Ek alt sistemler**: Intake (top toplama), Shooter (atıcı), Turret (taret) (motorlar CAN ID 5-9'da)

### CAN ID Eşleşmesi

Fiziksel CAN kablo bağlantıları (doğrulanmış):
- `CAN ID 1` → Arka Sağ NEO ( Rear Right) - **TERS: true**
- `CAN ID 2` → Ön Sol NEO (Front Left) - **TERS: false**
- `CAN ID 3` → Arka Sol NEO (Rear Left) - **TERS: false**
- `CAN ID 4` → Ön Sağ NEO (Front Right) - **TERS: true**

**Önemli:** Mecanum sürüşünde SAĞ taraf motorları ters çevrilmiştir (inverted: true), SOL taraf motorları normaldir (inverted: false).

## Derleme ve Dağıtım Komutları

```bash
# Projeyi derle
./gradlew build

# Robota dağıt (bağlı roboRIO gerekir)
./gradlew deploy

# Masaüstü simülasyonunu çalıştır
./gradlew simulate

# Testleri çalıştır
./gradlew test

# Belirli bir test sınıfını çalıştır
./gradlew test --tests <SınıfAdı>

# Derleme yapıtlarını temizle
./gradlew clean
```

## Mimari

### Paket Yapısı

```
frc.robot/
├── Main.java              # Giriş noktası
├── Robot.java             # TimedRobot yaşam döngüsü yönetimi
├── RobotContainer.java    # Alt sistemler, komutlar ve tuş bağları
├── Constants.java         # Tüm yapılandırma sabitleri
├── FieldConstants.java    # 2026 saha boyutları ve AprilTag pozisyonları
├── Commands/              # Komut sınıfları
└── Subsystems/            # Alt sistem implementasyonları
```

### Alt Sistem Mimaris

Kod WPILib'in komut tabanlı desenini takip eder:

- **DriveSubsystem**: Odometri, alan odaklı kontrol ve PathPlanner entegrasyonlu mekanum sürüş
- **VisionSubsystem**: Hedef algılama için Limelight'a NetworkTables arayüzü
- **IntakeSubsystem**, **ShooterSubsystem**, **TurretSubsystem**: Oyun parçası manipülasyonu

### Sürüş Kontrol Notları

- **Kontrolcü eşleşmesi**: Driver PS4 kullanır (port 0), Operator Joystick kullanır (port 1)
- **Eksenler**: `DRIVER_Y_AXIS=1` (Left Stick Y - ileri/geri), `DRIVER_X_AXIS=0` (Left Stick X - sağ/sol), `DRIVER_Z_AXIS=2` (Right Stick X)
- **İleri yönü**: Joystick yukarı itildiğinde robot ileri gider
- **Robot odaklı sürüş**: Alan odaklı sürüş YOK - robot kendi yönüne göre hareket eder (NavX jiroskop kullanılmaz)
- **Önemli Karar**: Bu robot sadece **ROBOT ORIENTED DRIVE** kullanır - robot kendi yönüne göre hareket eder, sahadaki yöne göre değil

### Sabitler Organizasyonu

Tüm ayarlanabilir parametreler `Constants.java` dosyasında iç içe sınıflar halinde:
- `MotorConstants`: CAN ID'ler ve motor ters çevirme
- `DriveConstants`: Fiziksel boyutlar, kinematik, maksimum hızlar
- `DriveControlConstants`: Deadband, slew oranları, giriş ölçekleme
- `ModuleConstants`: Intake/turret/shooter hızları, vizyon geometrisi
- `OIConstants`: Kontrolcü port/tuş/eksen eşleşmeleri
- `NavXTestConstants`: Yaw doğrulama test parametreleri

### Önemli: ENABLE_NON_DRIVE_MOTORS Bayrağı

Intake/shooter/turret motor kontrolörlerini etkinleştirmek için `MotorConstants.ENABLE_NON_DRIVE_MOTORS = true` yapın. `false` olduğunda, bu alt sistemler motor çağrılarını no-op yapar (test için veya CAN çakışmalarını önlemek için kullanışlıdır).

## NavX Yaw Doğrulama Özelliği

`DriveSubsystem`, disabled modunda çalışan yerleşik bir NavX yaw doğrulama testi içerir. Robotu saat yönünde ve saat yönünün tersine döndürerek jiroskopun çalıştığını doğrular.

**Kullanım:**
1. SmartDashboard'da `NavXTest/Run = true` yapın (sadece Disabled durumundayken çalışır)
2. Test otomatik olarak durum makinesi üzerinden çalışır
3. Sonuçlar `NavXTest/Status`, `NavXTest/ErrorCode` ve dashboard değerlerinde görünür

**Hata kodları:** NAVX_E001'den NAVX_E007'ye kadar (açıklamalar için ROBOT_SETUP.md'ye bakın)

## PathPlanner Entegrasyonu

Otonom yollar PathPlanner lib üzerinden yapılandırılır. Auto dosyaları `src/main/deploy/pathplanner/` içinde bulunur ve SmartDashboard "Auto Chooser" üzerinden seçilebilir.

DriveSubsystem, PathPlanner path takibi için holonomic sürüş kontrolörü ile `AutoBuilder.configure()` implemente eder.

## Test ve Simülasyon

- Birim testleri JUnit 5 kullanır (`test/` dizinine bakın)
- Masaüstü simülasyonu WPILib GUI üzerinden desteklenir
- Kod, gerçek donanım vs simüle edilmiş değerler koşullu olarak kullanmak için `RobotBase.isReal()` kontrol eder
- SmartDashboard'daki motor test toggle'ları disabled durumdayken bireysel motor testine izin verir

## Vendor Bağımlılıkları

`vendordeps/` içinde bulunur:
- `REVLib.json`: Spark MAX motor kontrolcü desteği
- `PathplannerLib-2026.1.2.json`: Otonom path takibi
- `Studica.json`: NavX jiroskop desteği
- `WPILibNewCommands.json`: Komut tabanlı framework

## Referans Dokümantasyon

- ROBOT_SETUP.md doğrulanmış motor eşleşmeleri ve kontrol ayar değerleriyle (Türkçe) ayrıntılı donanım kurulum notları içerir
- WPILib dokümantasyonu: https://docs.wpilib.org/
- REVLib (Spark MAX) dokümantasyonu: https://docs.revrobotics.com/
- PathPlanner dokümantasyonu: https://pathplanner.dev/
