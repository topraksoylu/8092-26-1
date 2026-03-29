# CLAUDE.md

Bu dosya, bu depoda calisirken gelistirici yardimcisina hizli teknik baglam verir.

## Proje Ozeti

- Dil: Java (WPILib command-based)
- Robot: Mecanum surus + AprilTag vision + taret
- Ana siniflar:
  - `Robot.java`
  - `RobotKapsayici.java`
  - `Sabitler.java`

## Guncel Donanim

### Surus
- 4x NEO + Spark Max (CAN)
- CAN: 3 (on sol), 4 (on sag), 1 (arka sag), 2 (arka sol)

### Mekanizma
| Alt Sistem | Motor | Baglanti | CAN/PWM | Sabit |
|---|---|---|---|---|
| Alim (intake) | CIM | PWM | 9 | `ALIM_CIM_PWM_KANALI` |
| Yukari tasiyici | CIM | PWM | 8 | `DEPO_ATICI_YUKARI_TASIYICI_CIM_PWM_KANALI` |
| Atici (shooter) | NEO | CAN | 5 | `ATICI_MOTOR_ID` |
| Taret (turret) | NEO | CAN | 6 | `TARET_MOTOR_ID` |

### Sensor
- NavX (MXP SPI)
- Limelight 3

## Kod Yapisi

### Alt Sistemler (`Subsystems/`)
- `SurusAltSistemi` — mecanum surus, odometri, NavX
- `GorusAltSistemi` — Limelight, AprilTag poz tahmini
- `AlimAltSistemi` — alim CIM + yukari tasiyici CIM
- `AticiAltSistemi` — atici NEO
- `TaretAltSistemi` — taret NEO, ±90° yazilim siniri

### Komutlar (`Commands/`)
- `SurusKomutu` — joystick ile mecanum surus
- `AlimKomutu` — alim calistir
- `AtisKomutu` — atis + tasiyici
- `TaretTakipKomutu` — Limelight ile taret takibi
- `HedefeHizalamaKomutu` — tareti hedefe hizala
- `OtonomAtisKomutu` — hizala + at siralisi
- `AprilTagaHizalamaKomutu` / `AprilTagTakipKomutu` — vision tabanli surus

## Buton Yerlesimi (PS4, Port 0)

| Buton | PS4 | Eylem | Motor |
|---|---|---|---|
| 1 | Kare | Alim | PWM 9 |
| 2 | Carpi | Geri at | PWM 9 |
| 3 | Daire | Yukari tasiyici | PWM 8 |
| 4 | Ucgen | Atici | CAN 5 |
| 5 | L1 | Taret sola | CAN 6 |
| 6 | R1 | Taret saga | CAN 6 |

Tum butonlar `whileTrue`: basili tutulurken calisir, birakilinca durur.

## Calisma Hizlari (`Sabitler.ModulSabitleri`)

| Sabit | Deger | Aciklama |
|---|---|---|
| `ALIM_HIZI` | 0.5 | Alim/geri-at hizi |
| `DEPO_ATICI_YUKARI_TASIYICI_HIZI` | 0.5 | Tasiyici hizi |
| `ATICI_HIZI` | 0.8 | Atici hizi |
| `TARET_HIZI` | 0.25 | Manuel taret hizi |

## Taret Sinirlari

- Yazilim siniri: **±90°** (toplam 180° hareket alani)
- Kablo nedeniyle tam tur donusu yapilmamalidir
- Limit switch (sifirlama icin) henuz eklenmemistir

## Derleme ve Deploy

```bash
./gradlew build
./gradlew test
./gradlew deploy
```

## Onemli Notlar

- Surus disi alt sistemleri fiziksel olarak etkinlestirmek icin:
  - `Sabitler.MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN = true`
- Taret enkoderi goreli (relative) calisir; her acilista 0'dan baslar.
- Limit switch eklendiginde `TaretAltSistemi`'ne homing rutini eklenmelidir.

## Referans Dokuman

- `ROBOT.md` — donanim haritasi, buton yerlesimi, test akisi, sorun giderme
