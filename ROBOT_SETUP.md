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

- `CAN ID 2` -> `Front Left (Ön Sol) NEO`
- `CAN ID 3` -> `Front Right (Ön Sağ) NEO`
- `CAN ID 4` -> `Rear Left (Arka Sol) NEO`
- `CAN ID 1` -> `Rear Right (Arka Sağ) NEO`

## Motor Ters Çevirme (Inverted) Durumu

- `Front Left (CAN ID 2)` -> `Inverted = false` (Normal yön)
- `Front Right (CAN ID 3)` -> `Inverted = true` (Ters çevrildi)
- `Rear Left (CAN ID 4)` -> `Inverted = true` (Ters çevrildi)
- `Rear Right (CAN ID 1)` -> `Inverted = false` (Normal yön)

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

## Main Controller

- `NI roboRIO 1.0` - Main robot controller running WPILib
- Network Address: `10.80.92.2`

## Power Distribution

- `CTRE Power Distribution Panel (PDP)` - Distributes power to robot components
- `CTRE Voltage Regulator Module (VRM)` - Provides regulated 12V and 5V power
- `OptiFuse 120A Main Breaker` - Main circuit protection

## Networking

- `Vivid-Hosting VH-109 FRC Radio` - Robot wireless communication
- Network Address: `10.80.92.11`
- Team: `8092`

## Vision System

- The robot uses a `Limelight 3` camera for AprilTag detection.
- `LimelightHelpers.java` - Complete Limelight API wrapper (vendor library helper)
- Camera: `OV5647` color rolling shutter (`640x480 @ 90 FPS`)
- Field of View: `62.5°` horizontal, `48.9°` vertical
- Network address: `10.80.92.200`
- Web interface: `http://10.80.92.200:5801`
- AprilTag pipeline for game piece alignment
- Targets tags `12` and `15`
- Constants in `VisionConstants` include PID tuning and speed limits

## Network Addresses

- Radio: `10.80.92.11` (`Vivid-Hosting VH-109`)
- roboRIO: `10.80.92.2` (`NI roboRIO 1.0`)
- Limelight: `10.80.92.200` (`Limelight 3`)

## MXP SPI

- `NavX Gyroscope`

## NEO Motor Specifications

- Model: `REV-21-1650` (`NEO V1.1`)
- Nominal voltage: `12V`
- `Kv`: `473`
- Free speed: `5676 RPM`
- Free current: `1.8A`
- Stall current (empirical): `105A`
- Stall torque (empirical): `2.6 Nm`
- Peak output power (empirical): `406W`
- Typical output power @ `40A`: `380W`
- Hall encoder: `42 counts/rev`
- Shaft: `8mm keyed`, length `35mm`
- Weight: `0.425 kg` (`0.938 lb`)

## roboRIO Verified Config (Captured 2026-03-05)

- Hostname: `roboRIO-8092-FRC`
- DNS Name: `roboRIO-8092-FRC.lan`
- Vendor/Model: `National Instruments roboRIO`
- Serial Number: `031885C1`
- Firmware Version: `25.5.0f112`
- OS: `NI Linux Real-Time ARMv7-A 4.14.146-rt67`
- Image Version: `FRC_roboRIO_2026_v1.2`
- IP (eth0): `10.80.92.2/24`
- MAC (eth0): `00:80:2F:24:AB:01`
- Gateway: `10.80.92.4`
- DNS Server: `10.80.92.1`
- IPv4 mode (eth0): `DHCP or Link Local`
- USB (usb0): `DHCP Only`, currently `0.0.0.0`
- CPU: `2 cores`, `2 logical processors`
