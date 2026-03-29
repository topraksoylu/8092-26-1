package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Sabitler {
    
    public static class MotorSabitleri {
        // NEO Fircasiz Motor V1.1 Ozellikleri:
        // - Bos Hiz: 5676 RPM
        // - Durma Torku: 2.6 Nm
        // - Durma Akimi: 105 A
        // - Bos Akim: 1.8 A

        // Surus motorlari
        public static final int ARKA_SOL_MOTOR_ID = 2;
        public static final int ON_SOL_MOTOR_ID = 3;
        public static final int ARKA_SAG_MOTOR_ID = 1;
        public static final int ON_SAG_MOTOR_ID = 4;

        // WPILib MecanumDrive ileri komutunda tum motorlara +1 gonderir.
        // Sag motorlar fiziksel olarak ayna monteli oldugu icin inverted olmalidir.
        public static final boolean ARKA_SOL_MOTOR_TERS = false;
        public static final boolean ON_SOL_MOTOR_TERS = false;
        public static final boolean ARKA_SAG_MOTOR_TERS = true;
        public static final boolean ON_SAG_MOTOR_TERS = true;

        // NEO motorlar icin akilli akim sinirlama \(sigorta atmasini azaltir\)
        public static final int SURUS_MOTORU_DURMA_AKIM_SINIRI = 60;  // Amper \(NEO durma akimi 105A\)
        public static final int SURUS_MOTORU_BOSTA_AKIM_SINIRI = 40;  // Amps
        public static final int SURUS_MOTORU_AKIM_SINIR_ESIGI = 40;   // RPM

        public static final boolean SURUS_DISI_MOTORLARI_ETKIN = true;

        // Alim ve tasiyici CIM motorlari (PWM)
        public static final boolean ALIM_MOTOR_TERS = false;
        public static final int ALIM_CIM_PWM_KANALI = 9;
        public static final int DEPO_ATICI_YUKARI_TASIYICI_CIM_PWM_KANALI = 8;
        public static final boolean DEPO_ATICI_YUKARI_TASIYICI_TERS = false;

        // Atici motoru (tek Spark Max)
        public static final int ATICI_MOTOR_ID = 5;
        public static final boolean ATICI_MOTOR_TERS = false;

        // Taret motoru (tek Spark Max)
        public static final int TARET_MOTOR_ID = 6;
        public static final boolean TARET_MOTOR_TERS = false;

        // Taret disli orani: 200T ring / 19T pinion = ~10.526:1
        public static final double TARET_DISLI_ORANI = 200.0 / 19.0;

        // Poz tabanli taret takibi
        public static final double TARET_POZ_KP = 0.01;           // derece hata -> motor hizi
        public static final double TARET_KI = 0.001;              // statik surunme hatasini giderir
        public static final double TARET_HIZALAMA_ESIGI_DERECE = 2.0;

        // MAXMotion trapezoidal profil (SparkMax onboard)
        public static final double TARET_MAXMOTION_CRUISE_RPM = 200.0;    // Motor cruise hizi (RPM)
        public static final double TARET_MAXMOTION_ACCEL_RPM_S = 400.0;   // Motor ivme (RPM/s)
        public static final double TARET_MAXMOTION_HATA_TOLERANSI = 0.1;  // Motor rotasyon hatasi
        // Taret one bakiyor: 0° = robotun onu, offset sifir
        // (Eski: TARET_ARKA_OFFSET_DERECE = 180.0 — taret arkadaydi)
        public static final double TARET_ON_OFFSET_DERECE = 0.0;

        // Taret aci sinirlari (toplam 180 derece hareket alani)
        // 0° = robotun onu, -90° = limit switch pozisyonu (baslangic)
        public static final double TARET_MAKS_ACI = 90.0;   // derece
        public static final double TARET_MIN_ACI = -90.0;   // derece (limit switch)

        // Taret limit switch (homing icin, normally closed)
        public static final int TARET_LIMIT_SWITCH_DIO = 9;
        // Homing sirasinda tareti limit switch'e dogru dondurmek icin hiz
        // Manuel hizla esit (TARET_HIZI = 0.04), negatif = sol yön (limit switch tarafi, -90°)
        public static final double TARET_HOMING_HIZI = -0.04;
    }

    public static class SurusSabitleri {

        public static final double IZ_GENISLIGI = 0.52;
        public static final double AKS_MESAFESI = 0.575;

        public static final Translation2d[] TEKER_POZISYONLARI = {
            new Translation2d( AKS_MESAFESI / 2,  IZ_GENISLIGI / 2),   // On Sol
            new Translation2d( AKS_MESAFESI / 2, -IZ_GENISLIGI / 2),   // On Sag
            new Translation2d(-AKS_MESAFESI / 2,  IZ_GENISLIGI / 2),   // Arka Sol
            new Translation2d(-AKS_MESAFESI / 2, -IZ_GENISLIGI / 2)    // Arka Sag
        };

        public static final double TEKER_CAPI = 0.1524;  // 6 inc AndyMark Mecanum tekerleri
        public static final double TEKER_CEVRESI = Math.PI * TEKER_CAPI;
        public static final double DISLI_ORANI = 12.75;  // Toughbox Mini Classic 12.75:1

        // 12.75:1 disli oraniyla NEO azami hiz hesabi:
        // NEO Bos Hiz: 5676 RPM
        // Disli kutusundan sonra: 5676 / 12.75 = 445 RPM
        // Teker hizi: 445 / 60 * pi * 0.1524 = 3.55 m/s teorik
        // Kayiplar dahil temkinli tahmin: ~3.0 m/s
        public static final double MAKS_HIZ_METRE_SANIYE = 3.0;

        // Velocity PID + Feedforward sabitleri (otonom yol takibi icin)
        // kS: motorun harekete gecmesi icin gereken minimum voltaj
        // kV: hiz basvurusu basi dusurulmesi gereken voltaj (12V / maks_hiz)
        public static final double SURUS_FF_KS = 0.1;   // Volt (statik katsayi)
        public static final double SURUS_FF_KV = 12.0 / MAKS_HIZ_METRE_SANIYE; // V/(m/s)
        public static final double SURUS_PID_KP = 0.3;  // Volt / (m/s hata)

    }

    public static class SurusKontrolSabitleri {
        // AndyMark 6" Mecanum tekerler icin optimize edildi \(80a rulo\)
        public static final double OLU_BOLGE = 0.08;  // Sert rulolarda hassasiyet icin daha dusuk

        // Hassas kontrol icin eksen duyarliligi
        public static final double Y_EKSEN_HASSASIYETI = 0.6;   // Ileri/geri
        public static final double X_EKSEN_HASSASIYETI = 0.65;  // Yanal (higher for mecanum efficiency loss)
        public static final double Z_EKSEN_HASSASIYETI = 0.6;   // Rotation

        // Yumusak ivmelenme icin sinirlama \(daha dusuk = daha yumusak\)
        public static final double OTELEME_SINIRLAMA_ORANI = 1.0;  // Forward/Yanal acceleration (reduced for smoothness)
        public static final double DONUS_SINIRLAMA_ORANI = 2.0;     // Donus ivmesi \(yumusak donus icin dusuk\)

        // Cikis olcek katsayilari
        public static final double OTELEME_OLCEGI = 0.8;        // Ileri/geri (arttirildi - kare yanit + dusuk olcek robotu hareket ettirmiyordu)
        public static final double YANLAMASINA_OLCEK = 0.8;     // Yanal (mecanum kayip telafisi)
        public static final double DONUS_OLCEGI = 0.6;          // Donus gucu
    }

    public static class OISabitleri {
        public static final int SURUCU_JOYSTICK_PORTU = 0;
        public static final int OPERATOR_JOYSTICK_PORTU = 1;

        public static final int SURUCU_X_EKSENI = 0;  // PS4 Left X (yanal)
        public static final int SURUCU_Y_EKSENI = 1;  // PS4 Left Y (ileri/geri)
        public static final int SURUCU_Z_EKSENI = 2;  // PS4 Right X (donus)
    }

    public static class ModulSabitleri {
        public static final double ALIM_HIZI = 0.75;
        public static final double DEPO_ATICI_YUKARI_TASIYICI_HIZI = 0.75;
        public static final double ATICI_HIZI = 1;
        public static final double ATICI_HEDEF_RPM = 4000.0; // Velocity PID hedef hizi (NEO max: 5676 RPM)
        public static final double ATICI_KP = 0.0001;        // Velocity PID P kazanci (REV resmi)
        public static final double ATICI_KV = 12.0 / 5676.0; // Feedforward: 12V / NEO max RPM
        public static final double TARET_HIZI = 0.04;
    }

    public static class GorusSabitleri {
        // Limelight yapilandirmasi
        public static final String LIMELIGHT_ADI = "limelight";
        public static final int ISTENEN_HAT = 0;
        public static final int ISTENEN_KAMERA_MODU = 0; // 0 = Gorus isleyici
        public static final int ISTENEN_LED_MODU = 0; // 0 = Hat kontrolu
        public static final int ISTENEN_YAYIN_MODU = 0; // 0 = Standart gorunum
        public static final double YAPILANDIRMA_YENIDEN_UYGULAMA_ARALIGI_SN = 1.0;

        // AprilTag FMAP kaynagi
        public static final String FMAP_KAYNAGI = "FRC2026_ANDYMARK.fmap";
        public static final int TOPLAM_APRILTAG = 32; // Her ittifak tarafi icin 16

        // Kamera montaji \(ROBOT UZERINDE OLCEK - simdilik varsayilan\)
        public static final double KAMERA_YUKSEKLIGI_METRE = 0.5;
        public static final double KAMERA_EGIMI_RADYAN = Math.toRadians(25.0);

        // AprilTag ayarlari
        public static final double APRILTAG_BOYUTU_METRE = 0.1651; // 6.5 inches (from FMAP)
        public static final double SPEAKER_TAG_YUKSEKLIGI_METRE = 0.889; // FMAP Z koordinatindan

        // Atış hedefi AprilTag ID'leri (saha haritasına göre)
        // Kırmızı taraf scoring yapıları çevresindeki taglar
        public static final int[] KIRMIZI_HEDEF_TAGLERI = {2, 3, 4, 5, 8, 9, 10, 11};
        // Mavi taraf scoring yapıları çevresindeki taglar
        public static final int[] MAVI_HEDEF_TAGLERI = {18, 19, 20, 21, 24, 25, 26, 27};

        // Gorus olcum guvenilirligi
        public static final double GORUS_OLCUM_STD_SAPMA_SN = 0.5;
        public static final double BELIRSIZLIK_ESIGI = 0.2;
        

        // Poz guncelleme hizi
        public static final double POZ_GUNCELLEME_ARALIGI_SN = 0.05; // 20Hz

        // GorusAltSistemi ile geriye donuk uyumluluk sabitleri
        public static final double HEDEF_YUKSEKLIGI = SPEAKER_TAG_YUKSEKLIGI_METRE;
        public static final double LIMELIGHT_YUKSEKLIGI = KAMERA_YUKSEKLIGI_METRE;
        public static final double LIMELIGHT_ACISI = Math.toDegrees(KAMERA_EGIMI_RADYAN);
    }

    public static class NavXTestSabitleri {
        public static final long SIFIRLAMA_OTURMA_MS = 500;
        public static final double TEST_DONUS_CIKTISI = 0.3;
        public static final double BEKLENEN_MIN_DELTA_DERECE = 45.0;
        public static final double MAKS_MUTLAK_YAW_SICRAMA_DERECE = 180.0;
        public static final double DONUS_ASAMA_ZAMAN_ASIMI_SN = 5.0;
    }

}
