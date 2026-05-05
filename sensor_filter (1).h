/**
 * @file    sensor_filter.h
 * @brief   UKB Kurtarma Sistemi – Sensör Filtreleme Katmanı
 *
 * Kapsam    : BMP180 (basınç/irtifa), MPU6050 (6-eksen IMU), ADXL345 (3-eksen ivme)
 * Gereksinim: REQ-SEN-001 – Ham sensör verisi FSM'ye doğrudan iletilmez;
 *             tüm değerler bu modülden geçirilir.
 *
 * Filtre Zinciri:
 *   BMP180  → Outlier koruması → EMA (basınç) → İrtifa hesabı → EMA (irtifa)
 *   MPU6050 → Jiroskop kalibrasyonu → Tamamlayıcı filtre (pitch/roll)
 *   ADXL345 → Medyan filtre (N=5) → EMA (her eksen)
 */

#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

#include <stdint.h>
#include <math.h>

/* ─── Sabitler ─────────────────────────────────────────────────────────── */

/* BMP180 – Outlier koruması: bir örnekleme döngüsünde izin verilen
 * maksimum basınç değişimi (hPa). Roketin gerçek çıkışı göz önüne alınarak
 * 50 Hz örnekleme için ~10 hPa/adım yeterlidir. */
#define BMP180_MAX_DELTA_HPA    10.0f

/* BMP180 – EMA yumuşatma katsayıları */
#define BMP180_EMA_ALPHA_PRESS  0.15f   /* Basınç yumuşatma */
#define BMP180_EMA_ALPHA_ALT    0.20f   /* İrtifa yumuşatma */

/* MPU6050 – Tamamlayıcı filtre: jiroskopa verilen güven oranı.
 * 1-0.98 = 0.02 → ivmemetreye %2 ağırlık (drift düzeltmesi için) */
#define MPU6050_COMP_ALPHA      0.98f

/* MPU6050 – Kalibrasyon için kullanılan örnek sayısı */
#define MPU6050_CALIB_SAMPLES   100

/* ADXL345 – Medyan filtre pencere boyutu (tek sayı olmalı) */
#define ADXL345_MEDIAN_N        5

/* ADXL345 – EMA yumuşatma katsayısı */
#define ADXL345_EMA_ALPHA       0.25f

/* Uluslararası Standart Atmosfer sabitleri */
#define ISA_P0_HPA              1013.25f  /* Deniz seviyesi referans basıncı */
#define RAD_TO_DEG              (180.0f / 3.14159265f)

/* ─── Veri Yapıları ─────────────────────────────────────────────────────── */

/**
 * @brief BMP180 filtresi iç durumu.
 *        Her değişken modül içinde korunur; dışarıdan doğrudan yazılmaz.
 */
typedef struct {
    float last_accepted_press;  /* Outlier kontrolünden geçen son basınç */
    float ema_press;            /* EMA çıkışı – basınç (hPa) */
    float ema_alt;              /* EMA çıkışı – irtifa (m) */
    float p0_hpa;               /* Referans yer basıncı (kalibrasyon anında alınır) */
    uint8_t initialized;        /* 0: ilk okuma yapılmadı, 1: hazır */
} BMP180_FilterState_t;

/**
 * @brief MPU6050 filtresi iç durumu.
 */
typedef struct {
    float gyro_bias_x;          /* Kalibrasyon ofseti – X ekseni (°/s) */
    float gyro_bias_y;          /* Kalibrasyon ofseti – Y ekseni (°/s) */
    float gyro_bias_z;          /* Kalibrasyon ofseti – Z ekseni (°/s) */
    float pitch;                /* Tamamlayıcı filtre çıkışı – pitch (°) */
    float roll;                 /* Tamamlayıcı filtre çıkışı – roll  (°) */
    uint8_t calibrated;         /* 0: kalibrasyon yapılmadı, 1: hazır */
} MPU6050_FilterState_t;

/**
 * @brief ADXL345 filtresi iç durumu.
 */
typedef struct {
    float win_x[ADXL345_MEDIAN_N];   /* Medyan penceresi – X */
    float win_y[ADXL345_MEDIAN_N];   /* Medyan penceresi – Y */
    float win_z[ADXL345_MEDIAN_N];   /* Medyan penceresi – Z */
    int   win_idx;                    /* Döngüsel yazma indeksi */
    float ema_x;                      /* EMA çıkışı – X (g) */
    float ema_y;                      /* EMA çıkışı – Y (g) */
    float ema_z;                      /* EMA çıkışı – Z (g) */
    uint8_t initialized;
} ADXL345_FilterState_t;

/**
 * @brief FSM'ye iletilen TEMİZ veri yapısı.
 *        Bu yapı yalnızca filtrelenmiş değerler içerir.
 *        Ham sensör okumalarına FSM'den hiçbir zaman erişilmez.
 */
typedef struct {
    /* BMP180 çıkışları */
    float altitude_m;           /* Filtrelenmiş irtifa (metre) */
    float pressure_hpa;         /* Filtrelenmiş basınç (hPa) */

    /* MPU6050 çıkışları */
    float pitch_deg;            /* Filtrelenmiş pitch açısı (derece) */
    float roll_deg;             /* Filtrelenmiş roll açısı (derece) */

    /* ADXL345 çıkışları */
    float accel_x_g;            /* Filtrelenmiş X ivmesi (g) */
    float accel_y_g;            /* Filtrelenmiş Y ivmesi (g) */
    float accel_z_g;            /* Filtrelenmiş Z ivmesi (g) */

    /* Geçerlilik bayrakları */
    uint8_t bmp180_valid;       /* 1: BMP180 verisi güncel ve güvenilir */
    uint8_t mpu6050_valid;      /* 1: MPU6050 verisi güncel ve güvenilir */
    uint8_t adxl345_valid;      /* 1: ADXL345 verisi güncel ve güvenilir */
} SensorData_Filtered_t;

/* ─── Global örnek (main.c'de tanımlanır, filter.c'de extern kullanılır) ── */
extern SensorData_Filtered_t sensorData;

/* ─── Fonksiyon Prototipleri ────────────────────────────────────────────── */

/* Başlatma */
void BMP180_Filter_Init(BMP180_FilterState_t *fs, float initial_press_hpa);
void MPU6050_Filter_Calibrate(MPU6050_FilterState_t *fs);
void ADXL345_Filter_Init(ADXL345_FilterState_t *fs);

/* Filtreleme – her örnekleme döngüsünde çağrılır */
void BMP180_Filter_Update(BMP180_FilterState_t *fs, float raw_press_hpa, float dt_s);
void MPU6050_Filter_Update(MPU6050_FilterState_t *fs,
                           float ax, float ay, float az,
                           float gx_dps, float gy_dps, float gz_dps,
                           float dt_s);
void ADXL345_Filter_Update(ADXL345_FilterState_t *fs,
                           float ax_raw, float ay_raw, float az_raw);

/* Yardımcı */
float Median5(float *arr);
float Parse_BigEndian_Float(uint8_t *buf, int idx);

#endif /* SENSOR_FILTER_H */
