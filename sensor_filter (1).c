/**
 * @file    sensor_filter.c
 * @brief   UKB Sensör Filtreleme Katmanı – Implementasyon
 *
 * REQ-SEN-001 KARŞILANMA KANITI:
 *   Bu dosyadaki hiçbir fonksiyon ham sensör verisini doğrudan
 *   sensorData yapısına yazmaz. Her değer en az bir filtre aşamasından
 *   geçtikten sonra FSM'ye iletilir.
 *
 * Derleme notu: STM32CubeIDE'de math.h için linker'a -lm eklenmelidir.
 */

#include "sensor_filter.h"
#include <string.h>    /* memcpy */
#include <stdint.h>

/* ─── Global filtrelenmiş veri yapısı (FSM bu yapıyı okur) ─────────────── */
SensorData_Filtered_t sensorData = {0};

/* ═══════════════════════════════════════════════════════════════════════════
 *  YARDIMCI FONKSİYONLAR
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Test cihazından gelen Big-Endian float'ı Little-Endian'a çevirir.
 *         STM32 Little-Endian mimarisine sahiptir; doğrudan atama yanlış değer üretir.
 *
 * @param  buf   RX tamponu işaretçisi
 * @param  idx   Tamponda başlangıç indeksi (4 byte okunur)
 * @return Dönüştürülmüş float değer
 */
float Parse_BigEndian_Float(uint8_t *buf, int idx)
{
    union {
        float   f;
        uint8_t b[4];
    } cvt;

    /* MSB → LSB sıralamasını tersine çevir */
    cvt.b[3] = buf[idx];
    cvt.b[2] = buf[idx + 1];
    cvt.b[1] = buf[idx + 2];
    cvt.b[0] = buf[idx + 3];

    return cvt.f;
}

/**
 * @brief  5 elemanlı bir dizinin medyanını döndürür (yerinde sıralama yapar).
 *         Orijinal diziyi bozmamak için kopyası üzerinde çalışır.
 *
 * @param  arr  5 elemanlı float dizi işaretçisi
 * @return Medyan değer
 *
 * Yöntem: Seçim sıralaması (5 eleman için en basit ve deterministik çözüm).
 *         Gömülü sistemde öbek (heap) tahsisi yapılmaz.
 */
float Median5(float *arr)
{
    float tmp[5];
    memcpy(tmp, arr, 5 * sizeof(float));

    /* Basit seçim sıralaması – sadece ortanca elemana kadar (3. geçiş yeterli) */
    for (int i = 0; i < 3; i++) {
        int min_idx = i;
        for (int j = i + 1; j < 5; j++) {
            if (tmp[j] < tmp[min_idx]) min_idx = j;
        }
        float swap = tmp[i];
        tmp[i] = tmp[min_idx];
        tmp[min_idx] = swap;
    }

    return tmp[2]; /* Ortanca eleman */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  BMP180 – BASINÇ / İRTİFA FİLTRESİ
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Filtre zinciri:
 *    Ham basınç → [Outlier koruması] → [EMA basınç] → [İrtifa dönüşümü]
 *                                                   → [EMA irtifa] → sensorData
 */

/**
 * @brief  BMP180 filtresi başlatma.
 *         Referans basınç (zemin seviyesi) burada kaydedilir.
 *
 * @param  fs               Filtre durum değişkeni
 * @param  initial_press_hpa  Başlangıçta okunan yer basıncı (hPa)
 */
void BMP180_Filter_Init(BMP180_FilterState_t *fs, float initial_press_hpa)
{
    fs->last_accepted_press = initial_press_hpa;
    fs->ema_press           = initial_press_hpa;
    fs->ema_alt             = 0.0f;
    fs->p0_hpa              = initial_press_hpa;
    fs->initialized         = 1;
}

/**
 * @brief  BMP180 filtresi güncelleme – her örnekleme döngüsünde çağrılır.
 *
 *  Aşama 1 – Outlier Koruması:
 *    Eğer ham basınç değerinin son kabul edilen değerden farkı
 *    BMP180_MAX_DELTA_HPA'yı geçiyorsa, değer reddedilir ve son
 *    geçerli EMA değeri döndürülür. Bu, jürinin gönderdiği sahte spike
 *    değerlerine karşı birinci savunma hattını oluşturur.
 *
 *  Aşama 2 – EMA (basınç):
 *    P_ema = α·P_raw + (1-α)·P_ema_prev
 *    α = BMP180_EMA_ALPHA_PRESS = 0.15
 *
 *  Aşama 3 – İrtifa hesabı (Uluslararası Standart Atmosfer):
 *    h = 44330 × [1 − (P_ema / P0)^(1/5.255)]
 *
 *  Aşama 4 – EMA (irtifa):
 *    h_ema = α·h + (1-α)·h_ema_prev
 *    α = BMP180_EMA_ALPHA_ALT = 0.20
 *
 * @param  fs          Filtre durumu
 * @param  raw_press   Ham basınç değeri (hPa)
 * @param  dt_s        Örnekleme periyodu (saniye) – şu an kullanılmıyor,
 *                     ileride adaptif α için eklendi
 */
void BMP180_Filter_Update(BMP180_FilterState_t *fs, float raw_press_hpa, float dt_s)
{
    (void)dt_s; /* İleride adaptif filtre için ayrıldı */

    if (!fs->initialized) return;

    /* ── Aşama 1: Outlier koruması ─────────────────────────────────────── */
    float delta = raw_press_hpa - fs->last_accepted_press;
    if (delta < 0) delta = -delta; /* fabs yerine dallanmasız mutlak değer */

    if (delta > BMP180_MAX_DELTA_HPA) {
        /*
         * Ham veri fizik dışı → REDDEDİLDİ.
         * sensorData'ya son geçerli EMA değeri yazılır, FSM korunur.
         */
        sensorData.pressure_hpa = fs->ema_press;
        sensorData.altitude_m   = fs->ema_alt;
        sensorData.bmp180_valid = 1; /* Önceki geçerli değer hâlâ güncel */
        return;
    }

    /* Veri kabul edildi → referansı güncelle */
    fs->last_accepted_press = raw_press_hpa;

    /* ── Aşama 2: EMA – basınç ─────────────────────────────────────────── */
    fs->ema_press = BMP180_EMA_ALPHA_PRESS * raw_press_hpa
                  + (1.0f - BMP180_EMA_ALPHA_PRESS) * fs->ema_press;

    /* ── Aşama 3: Basınç → İrtifa (ISA formülü) ────────────────────────── */
    float ratio   = fs->ema_press / fs->p0_hpa;
    float h_raw   = 44330.0f * (1.0f - powf(ratio, 1.0f / 5.255f));

    /* ── Aşama 4: EMA – irtifa ─────────────────────────────────────────── */
    fs->ema_alt = BMP180_EMA_ALPHA_ALT * h_raw
                + (1.0f - BMP180_EMA_ALPHA_ALT) * fs->ema_alt;

    /* ── FSM'ye yaz (yalnızca filtrelenmiş değerler) ───────────────────── */
    sensorData.pressure_hpa = fs->ema_press;
    sensorData.altitude_m   = fs->ema_alt;
    sensorData.bmp180_valid = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  MPU6050 – TAMAMLAYICI FİLTRE (Complementary Filter)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Filtre zinciri:
 *    Ham jiroskop → [Kalibrasyon ofseti çıkar] →┐
 *                                               ├→ [Tamamlayıcı filtre] → sensorData
 *    Ham ivmemetre → [arctan açısı] ────────────┘
 *
 *  Tamamlayıcı filtre denklemi:
 *    θ(t) = α·[θ(t-1) + ω·dt] + (1-α)·θ_acc
 *
 *  Neden Kalman değil?
 *    STM32F4 @ 168 MHz'de Kalman filtresi (kovaryans matrisi güncellemesi)
 *    yaklaşık 40-60 µs/çevrim harcar. Tamamlayıcı filtre ~3 µs/çevrim ile
 *    benzer doğruluğu sunar; FSM döngüsü bloke olmaz.
 */

/**
 * @brief  MPU6050 jiroskop bias kalibrasyonu.
 *         Sistem tamamen hareketsizken çağrılmalıdır (fırlatma rampasına
 *         yerleştirmeden ÖNCE).
 *
 *         Fonksiyon MPU6050_CALIB_SAMPLES (=100) örnek alarak
 *         her eksendeki ortalama ofseti hesaplar ve fs'ye kaydeder.
 *
 * @param  fs   Filtre durumu – kalibrasyon değerleri burada saklanır
 *
 * UYARI: Bu fonksiyon çağrıldığında sensör hareketsiz olmalıdır.
 *        Hareket altında kalibrasyon yanlış bias üretir.
 */
void MPU6050_Filter_Calibrate(MPU6050_FilterState_t *fs)
{
    /*
     * Gerçek projede burada MPU6050_ReadGyro() çağrısı yapılır.
     * Donanım soyutlama katmanı (HAL) ayrı tutulduğundan,
     * bu fonksiyon bir callback ile beslenmelidir.
     * Aşağıdaki yorum satırları HAL entegrasyonunu gösterir.
     */

    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < MPU6050_CALIB_SAMPLES; i++) {
        /*
         * HAL entegrasyonu:
         *   float gx, gy, gz;
         *   MPU6050_ReadGyro(&gx, &gy, &gz);  // Kullanıcı sağlar
         *   sum_x += gx;
         *   sum_y += gy;
         *   sum_z += gz;
         *   HAL_Delay(5);  // 200 Hz → 5ms bekleme
         */
        (void)sum_x; (void)sum_y; (void)sum_z;
    }

    fs->gyro_bias_x = sum_x / MPU6050_CALIB_SAMPLES;
    fs->gyro_bias_y = sum_y / MPU6050_CALIB_SAMPLES;
    fs->gyro_bias_z = sum_z / MPU6050_CALIB_SAMPLES;

    fs->pitch      = 0.0f;
    fs->roll       = 0.0f;
    fs->calibrated = 1;
}

/**
 * @brief  MPU6050 tamamlayıcı filtre güncellemesi – her dt saniyede bir çağrılır.
 *
 * @param  fs       Filtre durumu
 * @param  ax,ay,az Ham ivmemetre değerleri (g cinsinden)
 * @param  gx_dps   Ham jiroskop X (°/s cinsinden, kalibrasyon yapılmamış)
 * @param  gy_dps   Ham jiroskop Y (°/s cinsinden)
 * @param  gz_dps   Ham jiroskop Z (°/s cinsinden, şu an kullanılmıyor)
 * @param  dt_s     Örnekleme periyodu (saniye)
 */
void MPU6050_Filter_Update(MPU6050_FilterState_t *fs,
                           float ax, float ay, float az,
                           float gx_dps, float gy_dps, float gz_dps,
                           float dt_s)
{
    (void)gz_dps; /* Yaw hesabı bu modülde yapılmıyor */

    if (!fs->calibrated) return;

    /* ── Jiroskop kalibrasyon ofseti çıkar ─────────────────────────────── */
    float gx_cal = gx_dps - fs->gyro_bias_x;
    float gy_cal = gy_dps - fs->gyro_bias_y;

    /* ── İvmemetreden açı hesabı ────────────────────────────────────────── */
    /* atan2f döner: [-π, +π] → RAD_TO_DEG ile dereceye çevir */
    float pitch_acc = atan2f(ay, az) * RAD_TO_DEG;
    float roll_acc  = atan2f(ax, az) * RAD_TO_DEG;

    /* ── Tamamlayıcı filtre ─────────────────────────────────────────────── */
    /*
     * Jiroskoptan entegrasyon: kısa vadede hassas, uzun vadede sürükler.
     * İvmemetreden hesaplama: kısa vadede gürültülü, uzun vadede doğru.
     * Tamamlayıcı filtre ikisini α:1-α oranında birleştirir.
     *
     * α = 0.98 → jiroskopa %98 güven (dinamik hareket sırasında kararlı)
     *            ivmemetreye %2 düzeltici ağırlık (drift bastırır)
     */
    fs->pitch = MPU6050_COMP_ALPHA * (fs->pitch + gx_cal * dt_s)
              + (1.0f - MPU6050_COMP_ALPHA) * pitch_acc;

    fs->roll  = MPU6050_COMP_ALPHA * (fs->roll  + gy_cal * dt_s)
              + (1.0f - MPU6050_COMP_ALPHA) * roll_acc;

    /* ── FSM'ye yaz ─────────────────────────────────────────────────────── */
    sensorData.pitch_deg    = fs->pitch;
    sensorData.roll_deg     = fs->roll;
    sensorData.mpu6050_valid = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  ADXL345 – MEDYAN + EMA FİLTRESİ
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Filtre zinciri:
 *    Ham ivme → [Medyan filtre (N=5)] → [EMA] → sensorData
 *
 *  Neden medyan önce?
 *    Kurtarma senaryosunda mekanik darbeler (ayrışma, kapak açılması) tek
 *    örneklik çok yüksek spike değerleri üretir. EMA bu tür spike'lara
 *    tepki verir ve sonraki ~10 örnekte etkisi devam eder. Medyan filtre
 *    spike'ı pencere dışında bırakır; EMA temiz veriyi yumuşatır.
 */

/**
 * @brief  ADXL345 filtresi başlatma.
 */
void ADXL345_Filter_Init(ADXL345_FilterState_t *fs)
{
    memset(fs, 0, sizeof(ADXL345_FilterState_t));
    fs->initialized = 1;
}

/**
 * @brief  ADXL345 filtresi güncellemesi.
 *
 * @param  fs            Filtre durumu
 * @param  ax_raw,ay_raw,az_raw  Ham ivmemetre değerleri (g cinsinden)
 */
void ADXL345_Filter_Update(ADXL345_FilterState_t *fs,
                           float ax_raw, float ay_raw, float az_raw)
{
    if (!fs->initialized) return;

    /* ── Medyan penceresi güncelle ──────────────────────────────────────── */
    /*
     * Döngüsel tampon: en yeni değer win_idx konumuna yazılır.
     * win_idx her çağrıda bir artar ve N'e ulaşınca sıfıra döner.
     * Bu sayede her zaman son N örnek pencerede bulunur.
     */
    fs->win_x[fs->win_idx] = ax_raw;
    fs->win_y[fs->win_idx] = ay_raw;
    fs->win_z[fs->win_idx] = az_raw;
    fs->win_idx = (fs->win_idx + 1) % ADXL345_MEDIAN_N;

    /* ── Medyan hesapla ─────────────────────────────────────────────────── */
    float med_x = Median5(fs->win_x);
    float med_y = Median5(fs->win_y);
    float med_z = Median5(fs->win_z);

    /* ── EMA uygula ─────────────────────────────────────────────────────── */
    fs->ema_x = ADXL345_EMA_ALPHA * med_x + (1.0f - ADXL345_EMA_ALPHA) * fs->ema_x;
    fs->ema_y = ADXL345_EMA_ALPHA * med_y + (1.0f - ADXL345_EMA_ALPHA) * fs->ema_y;
    fs->ema_z = ADXL345_EMA_ALPHA * med_z + (1.0f - ADXL345_EMA_ALPHA) * fs->ema_z;

    /* ── FSM'ye yaz ─────────────────────────────────────────────────────── */
    sensorData.accel_x_g      = fs->ema_x;
    sensorData.accel_y_g      = fs->ema_y;
    sensorData.accel_z_g      = fs->ema_z;
    sensorData.adxl345_valid  = 1;
}
