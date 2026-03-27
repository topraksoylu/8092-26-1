# FRC 2026 Robot Dokumantasyonu

## 1) Donanim Haritasi

### Surus — Mecanum (CAN)

| Sabit | CAN ID | Konum |
|---|---|---|
| `ON_SOL_MOTOR_ID` | 1 | On sol |
| `ON_SAG_MOTOR_ID` | 2 | On sag |
| `ARKA_SAG_MOTOR_ID` | 3 | Arka sag |
| `ARKA_SOL_MOTOR_ID` | 4 | Arka sol |

Sol taraf ters, sag taraf duz. Disli orani: 12.75:1. Maks hiz: ~3.0 m/s.

### Mekanizma

| Alt Sistem | Motor | Baglanti | ID | Hiz |
|---|---|---|---|---|
| Alim | CIM | PWM | 9 | %50 |
| Yukari tasiyici | CIM | PWM | 8 | %50 |
| Atici | NEO + Spark Max | CAN | 5 | %80 |
| Taret | NEO + Spark Max | CAN | 6 | %25 |

> Taret yazilim siniri: **±90°** (toplam 180°). Kablo nedeniyle tam tur donusu yapilmaz.
> Limit switch henuz eklenmemistir; enkoder her acilista 0'dan baslar.

### Sensor ve Ag

| Birim | Baglanti | Adres |
|---|---|---|
| NavX | MXP SPI | roboRIO uzerinde |
| Limelight 3 | Ethernet | 10.80.92.200 |
| roboRIO | — | 10.80.92.2 |
| Radio | — | 10.80.92.11 |

---

## 2) Kontroller (PS4, Port 0)

### Analog Eksenler — Surus

| Eksen | Hareket |
|---|---|
| Sol analog Y | Ileri / geri |
| Sol analog X | Yanal |
| Sag analog X | Donus |

### Buton Atamalari

| Buton | PS4 | Eylem | Motor |
|---|---|---|---|
| 1 | Kare | Alim | PWM 9 |
| 2 | Carpi | Geri at | PWM 9 |
| 3 | Daire | Yukari tasiyici | PWM 8 |
| 4 | Ucgen | Atici | CAN 5 |
| 5 | L1 | Taret sola | CAN 6 |
| 6 | R1 | Taret saga | CAN 6 |

Tum butonlar `whileTrue`: basili tutulurken calisir, birakilinca durur.

---

## 3) Konfigurasyon

`Sabitler.MotorSabitleri.SURUS_DISI_MOTORLARI_ETKIN`

- `true` — alim, tasiyici, atici, taret motorlari fiziksel cikis verir
- `false` — bu alt sistemler simule modda calisir (varsayilan)

Calisma hizlari `Sabitler.ModulSabitleri` icerisinde tanimlidir.

---

## 4) Test Akisi

1. DriverStation + SmartDashboard baglantisini dogrula.
2. `Vision/HasTarget` ve `Vision/FmapTagCount` degerlerini kontrol et.
3. `SURUS_DISI_MOTORLARI_ETKIN = false` iken butonlarin SmartDashboard'a yazdigini dogrula.
4. `SURUS_DISI_MOTORLARI_ETKIN = true` yap, DISABLED modda her butonu teker teker test et.
5. ENABLED modda surus eksenlerini kontrol et.

---

## 5) Donanim Dogrulama Akisi

Her test oturumunda asagidaki kontroller yapilmalidir:

1. Batarya >= 10.0V, brownout yok.
2. Her motor kanali (CAN 1-6, PWM 8-9) birer kez test edildi.
3. Enkoder yonu ve akim davranisi beklendigi gibi.
4. Limelight pipeline / camMode / ledMode dogru; FMAP tag sayisi eslesiyir.
5. NavX yaw degerleri mantikli.

### Dogrulama Artifact Uretimi

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run-hardware-validation.ps1 -BuildHash "<git-hash>"
```

Cikti: `build/hardware-validation/results.txt`

Her satir formati:

```
DURUM | KONTROL_ADI | ZAMAN_DAMGASI | BUILD_HASH | DETAYLAR
```

---

## 6) Sorun Giderme

| Belirti | Kontrol Edilecek |
|---|---|
| Robot hareket etmiyor | E-Stop, enable durumu, joystick port 0 baglantisi |
| Vision calismıyor | Limelight IP, pipeline, tag mesafesi, LED modu |
| Motor tepki vermiyor | `SURUS_DISI_MOTORLARI_ETKIN`, CAN/PWM kablo baglantisi, ters yon ayari |
| Taret siniri calisiyor | Enkoder sifirdan cok uzaklasti; robotu kapat/ac |
