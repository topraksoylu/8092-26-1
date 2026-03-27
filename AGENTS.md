# AGENTS.md

Bu dosya, bu depoda calisirken gelistirici yardimcisina hizli teknik baglam verir.

## Proje Ozeti

- Dil: Java (WPILib command-based)
- Robot: mecanum surus + AprilTag vision
- Ana siniflar:
  - Robot.java
  - RobotKapsayici.java
  - Sabitler.java

## Guncel Donanim

### Surus
- 4x NEO + Spark Max (CAN)
- CAN: 1 (on sol), 2 (on sag), 3 (arka sag), 4 (arka sol)

### Mekanizma
- Intake: 1x CIM (PWM)
- Depodan aticiya yukari tasima: 1x CIM (PWM)
- Shooter: NEO + Spark Max
- Turret: NEO + Spark Max

### Sensor
- NavX (MXP SPI)
- Limelight 3

## Derleme ve Deploy

```bash
./gradlew build
./gradlew test
./gradlew deploy
```

## Onemli Notlar

- Surus disi alt sistemleri fiziksel olarak etkinlestirmek icin:
  - `Sabitler.MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN = true`
- L1/R1 su anda CAN ID 3 motor testine atanmistir (%10 geri/ileri).

## Referans Dokumanlar

- ROBOT_USER_GUIDE.md: kullanim, butonlar, test akisi (birlestirilmis)
- ROBOT_SETUP.md: donanim ve baglanti haritasi
- HARDWARE_VALIDATION.md: dogrulama artifact akisi
