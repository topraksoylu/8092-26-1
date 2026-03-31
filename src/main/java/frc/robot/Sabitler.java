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
        public static final int ATICI_MOTOR_ID = 7; // CAN ID 6'dan 7'ye degistirildi (brick mode sorunu)
        public static final boolean ATICI_MOTOR_TERS = false;

        // Taret motoru (tek Spark Max)
        public static final int TARET_MOTOR_ID = 5;
        public static final boolean TARET_MOTOR_TERS = false;

        // Taret disli orani: 200T ring / 20T pinion = 10.0:1
        public static final double TARET_DISLI_ORANI = 200.0 / 20.0;

        // Poz tabanli taret takibi - DEVRE DISI (sadece manuel kontrol)
        public static final double TARET_POZ_KP = 0.0;            // derece hata -> motor hizi (0 = otomatik kapali)
        public static final double TARET_KI = 0.0;               // statik surunme hatasini giderir
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
        public static final int TARET_LIMIT_SWITCH_DIO = 0;
        // DIO okuma seviyesi: true=aktif high, false=aktif low.
        // Mevcut robot kurulumunda switch basili durum "low" olarak okunuyor.
        public static final boolean TARET_LIMIT_SWITCH_AKTIF_HIGH = true;
        // Homing sirasinda tareti limit switch'e dogru dondurmek icin hiz
        // Manuel hiz ile ayni (daha kontrollu)
        public static final double TARET_HOMING_HIZI = -0.08;
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
        // 2019 KOP AM14U/Toughbox Mini disli seti:
        // (50/14) * (48/16) = 10.71:1
        public static final double DISLI_ORANI = 10.71;

        // 10.71:1 disli oraniyla NEO azami hiz hesabi:
        // NEO Bos Hiz: 5676 RPM
        // Disli kutusundan sonra: 5676 / 10.71 = 530 RPM
        // Teker hizi: 530 / 60 * pi * 0.1524 = 4.23 m/s teorik
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

        // Yumusaklik + dusuk gecikme dengesi (stutter'i azaltirken cevabi hizli tutar)
        public static final double OTELEME_SINIRLAMA_ORANI = 3.0;  // Forward/Yanal acceleration
        public static final double DONUS_SINIRLAMA_ORANI = 4.0;    // Donus ivmesi

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

    /**
     * am-5780_CN "Launcher in a Box" fiziksel sabitleri.
     *
     * Kaynak: AndyMark Assembly Guide Rev.0 (12/18/2025), ürün: am-5780_CN
     *
     * Teker:     4" Stealth Wheel (am-2647_orange), Ø = 0.1016 m
     * Kayış:     Motor 24T (am-3403_half) → Mil 45T  →  oran = 45/24 = 1.875:1 azaltma
     * Motor:     NEO (CIM adaptörü am-0588_long ile de takılabilir)
     * Açı:       Yan plakalarda 22.5° adımlı 4 pozisyon — varsayılan: 45° (pozisyon 3)
     * Verimlilik:~0.55  (basınç + kayma kayıpları; saha ölçümüyle doğrulandı)
     *
     * Atış topu (2026 REBUILT game piece):
     *   Çap: 5.91 in (150 mm), Kütle: ~215 g
     */
    public static class AticiLauncherSabitleri {
        /** 4" Stealth Wheel çapı (m) */
        public static final double TEKER_CAPI_METRE    = 0.1016;
        /** 24T motor / 45T mil kayış dişlisi — motor:mil azaltma oranı */
        public static final double DISLI_ORANI         = 45.0 / 24.0;   // 1.875
        /**
         * Basınç + kayma verimliliği.
         * 0.55 → saha ölçümüyle doğrulandı (55% of theoretical surface speed).
         * Farklı sıkıştırma ayarında ±0.05 değişebilir; saha testinden sonra kalibre et.
         */
        public static final double VERIMLILIK          = 0.55;
        /** Atış açısı — 45° (yan plakaların 3. churro pozisyonu, en çok kullanılan) */
        public static final double ATIS_ACISI_DERECE   = 45.0;
        /**
         * Launcher çıkış noktası yüksekliği halıdan (m).
         * Taret pivot yüksekliği + launcher montaj yüksekliği.
         * Değeri robottan ölç ve güncelle.
         */
        public static final double LAUNCHER_YUKSEKLIK  = 0.70;
        /**
         * Hedef merkezi yüksekliği halıdan (m).
         * 2026 REBUILT MID Hub = 45 in = 1.143 m.
         */
        public static final double HEDEF_YUKSEKLIK     = 1.143;
        /** Minimum atış mesafesi (m) — bu altında top hedefe ulaşamaz */
        public static final double MIN_ATIS_MESAFESI   = 1.2;
        /** Maksimum güvenli atış mesafesi (m) — NEO ~5400 RPM sınırı */
        public static final double MAKS_ATIS_MESAFESI  = 5.5;
    }

    public static class ModulSabitleri {
        public static final double ALIM_HIZI = 0.75;
        public static final double DEPO_ATICI_YUKARI_TASIYICI_HIZI = 0.75;
        public static final double ATICI_HIZI = 0.90;
        public static final double YAKIN_ATIS_HIZ_CARPANI = 0.80;
        public static final double ORTA_ATIS_HIZ_CARPANI  = 0.90;
        public static final double UZAK_ATIS_HIZ_CARPANI  = 1.00;
        public static final double ATICI_MIN_RPM = 2600.0;
        public static final double ATICI_MAKS_RPM = 5400.0;
        public static final double ATICI_MAKS_CIKIS = 0.90;
        public static final double ATICI_HEDEF_RPM = 4000.0; // Velocity PID hedef hizi (NEO max: 5676 RPM)

        // 3 mesafe için atış RPM değerleri (Elastic Dashboard'dan değiştirilebilir)
        // SAHA TESTI ile kalibre edildi: 5200 RPM → 2.8m atış başarılı
        public static double YAKIN_ATIS_RPM = 3800.0;  // ~1.2 m
        public static double ORTA_ATIS_RPM  = 5200.0;  // ~2.8 m (test edilmiş değer)
        public static double UZAK_ATIS_RPM  = 5400.0;  // ~4.4 m (stabilite icin limitli)
        public static final double YAKIN_ATIS_HIZI = YAKIN_ATIS_RPM / 5676.0;
        public static final double ORTA_ATIS_HIZI  = ORTA_ATIS_RPM / 5676.0;
        public static final double UZAK_ATIS_HIZI  = UZAK_ATIS_RPM / 5676.0;
        /**
         * Mesafe→RPM kalibasyon tablosu.
         * SAHA TESTI ile güncellendi: 5200 RPM → 2.8m atış başarılı.
         * Not: NEO motor maksimum RPM = 5676
         */
        public static final double[] ATIS_MESAFE_TABLOSU_METRE =
            {1.2,  2.0,  2.8,  3.6,  4.4,  5.2};
        public static final double[] ATIS_RPM_TABLOSU =
            {3800, 4500, 5200, 5350, 5400, 5400};
        public static final double ATICI_KP = 0.0003;         // Velocity PID P kazanci
        public static final double ATICI_KFF = 1.0 / 5676.0; // Velocity feedforward: 1/NEO max RPM
        public static final double TARET_HIZI = 0.08;
    }

    /**
     * Limelight hedef görmediğinde sürücünün Elastic'ten seçebileceği
     * manuel atış mesafe ön ayarları.
     *
     * RPM değerleri SAHA TESTI ile kalibre edildi (5200 RPM → 2.8m başarılı):
     *   YAKIN  → ~1.2 m  → 3800 RPM
     *   ORTA   → ~2.8 m  → 5200 RPM
     *   UZAK   → ~4.4 m  → 5500 RPM (NEO max: 5676)
     */
    public enum ManuelAtisModu {
        YAKIN ("Yakın  (~1.2 m)",  3800.0),
        ORTA  ("Orta   (~2.8 m)",  5200.0),
        UZAK  ("Uzak   (~4.4 m)",  5400.0);

        public final String etiket;
        public final double rpm;

        ManuelAtisModu(String etiket, double rpm) {
            this.etiket = etiket;
            this.rpm    = rpm;
        }
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

        // Kamera montaji (robot merkezine gore)
        public static final double KAMERA_YUKSEKLIGI_METRE  = 0.5;    // yerden yukseklik (m)
        public static final double KAMERA_EGIMI_RADYAN      = Math.toRadians(30.0); // yukari bakis acisi
        public static final double KAMERA_YAN_OFFSET_METRE  = -0.26;  // saga = negatif (Limelight Y+ sola)

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
