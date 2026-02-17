# Robot Kurulum Notları

## Sürüş Sistemi
- 4 adet NEO fırçasız motor kullanılıyor.
- 4 adet mecanum teker kullanılıyor.
- Tüm sürüş motorlarında encoder mevcut.
- Motor sürücüler: Spark MAX.
- Haberleşme: CAN bus.

## Sürüş Kontrol Eksenleri (doğrulanmış)
- `DRIVER_X_AXIS = 1`
- `DRIVER_Y_AXIS = 0`
- `DRIVER_Z_AXIS = 2`
- İleri hareket: `Y axis -`.

## Kontrolcü
- Driver kontrolcü: `CUH-ZCT2U`.
- Driver joystick/controller portu: `0`.
- Operator joystick portu: `1`.
- Driver joystick eksenleri:
  - `X = Axis 1`
  - `Y = Axis 0`
  - `Z = Axis 2`
- Forward tanımı: `Y axis -` ileri.

## Ana Donanım
- `roboRIO 1` bağlı.
- `VividHosting VH-109` mevcut.

## Sensör ve Görüntüleme
- İlk sürüm NavX sensörü var.
- Limelight 3 mevcut, ancak şu anda bağlı değil.

## Motor/CAN Notu
- Sürüş tarafında Spark MAX üzerinden CAN bağlantısı kullanılıyor.
- CAN ID'ler sahadaki mevcut kablolama ve mekanik yön testine göre doğrulanmış.

## CAN ID -> Motor Eşleşmesi (Sürüş)
- `CAN ID 1` -> `Front Left (Ön Sol) NEO`
- `CAN ID 3` -> `Front Right (Ön Sağ) NEO`
- `CAN ID 4` -> `Rear Left (Arka Sol) NEO`
- `CAN ID 2` -> `Rear Right (Arka Sağ) NEO`

## Motor Ters Çevirme (Inverted) Durumu
- `Front Left (CAN ID 1)` -> `Inverted = true`
- `Front Right (CAN ID 3)` -> `Inverted = false`
- `Rear Left (CAN ID 4)` -> `Inverted = true`
- `Rear Right (CAN ID 2)` -> `Inverted = false`

## Sürüş Optimizasyon Ayarları (Mecanum)
- Deadband: `0.05`
- Translation scale: `0.90`
- Rotation scale: `0.75`
- Translation slew rate: `3.5`
- Rotation slew rate: `4.0`
- Uygulanan yöntem: deadband + squared input shaping + slew rate limiting (field-oriented sürüşte).

## NavX Yaw Doğrulama Prosedürü
- Test tipi: `Yaw-only` (CW ve CCW kısa dönüş doğrulaması).
- Tetikleme: SmartDashboard üzerinden `NavXTest/Run = true`.
- Testin çalışması için robotun `Disabled` durumda olması gerekir.
- Dashboard alanları:
  - `NavXTest/Run`
  - `NavXTest/Status`
  - `NavXTest/ErrorCode`
  - `NavXTest/LastYawStartDeg`
  - `NavXTest/LastYawEndDeg`
  - `NavXTest/DeltaDeg`
- Sonuç:
  - Başarılı: `Status = PASS`, `ErrorCode = OK`
  - Başarısız: `Status = FAIL`, ilgili hata kodu yazılır
- Test tamamlandığında `NavXTest/Run` otomatik olarak `false` yapılır.

## NavX Test Parametreleri
- `TEST_TURN_OUTPUT = 0.20`
- `TURN_PHASE_TIMEOUT_SEC = 1.5`
- `MIN_EXPECTED_DELTA_DEG = 10.0`
- `MAX_ABS_YAW_JUMP_DEG = 120.0`
- `ZERO_SETTLE_MS = 250`

## NavX Hata Kodları
- `OK`: Test geçti
- `NAVX_E001`: NavX bağlı değil / okuma geçersiz
- `NAVX_E002`: CW sign mismatch
- `NAVX_E003`: CCW sign mismatch
- `NAVX_E004`: Delta too small
- `NAVX_E005`: Faz timeout
- `NAVX_E006`: Disabled dışında tetikleme
- `NAVX_E007`: Yaw jump / outlier
