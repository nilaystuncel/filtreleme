
#include "sensor_filter.h"
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  UÇUŞ DURUM BİT ALANI (Telemetri paketi)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Bellekte tam olarak 16 bit (2 byte) yer kaplar.
 *  Bit kaydırma operasyonları yerine okunabilir bit alanı kullanılır.
 */
typedef struct __attribute__((packed)) {
    uint16_t liftoff_detected      : 1;  /* Bit  0: Fırlatma algılandı */
    uint16_t motor_burnout         : 1;  /* Bit  1: Motor yanması bitti */
    uint16_t threshold_alt_passed  : 1;  /* Bit  2: Eşik irtifası geçildi */
    uint16_t tilt_or_accel_fault   : 1;  /* Bit  3: Eğim / ivme hatası */
    uint16_t descent_detected      : 1;  /* Bit  4: İniş algılandı */
    uint16_t drogue_chute_cmd      : 1;  /* Bit  5: Fren paraşütü komutu */
    uint16_t main_chute_alt_passed : 1;  /* Bit  6: Ana paraşüt irtifası */
    uint16_t main_chute_cmd        : 1;  /* Bit  7: Ana paraşüt komutu */
    uint16_t reserved              : 8;  /* Bit 8-15: İleride kullanım */
} FlightStatusBits_t;

typedef union {
    FlightStatusBits_t status;
    uint8_t            payload_bytes[2];
} TelemetryStatus_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  GLOBAL DEĞİŞKENLER
 * ═══════════════════════════════════════════════════════════════════════════ */

/* DMA tamponları */
#define RX_BUF_LEN  34
#define TX_BUF_LEN  34
static uint8_t uart_rx_buffer[RX_BUF_LEN];
static uint8_t uart_tx_buffer[TX_BUF_LEN];

/* Filtre durum yapıları */
static BMP180_FilterState_t  bmp180_fs;
static MPU6050_FilterState_t mpu6050_fs;
static ADXL345_FilterState_t adxl345_fs;

/* Telemetri */
static TelemetryStatus_t current_telemetry = {0};

/* SUT protokol sabitleri */
#define SUT_HEADER      0xAB
#define TELEM_HEADER    0xAA
#define TELEM_FOOTER_CR 0x0D
#define TELEM_FOOTER_LF 0x0A

/* Örnekleme periyodu (saniye) – MPU6050 tamamlayıcı filtresi için */
#define SAMPLE_DT_S     0.01f   /* 100 Hz döngü varsayımı */

/* ═══════════════════════════════════════════════════════════════════════════
 *  CHECKSUM
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint8_t Calculate_Checksum(uint8_t *buf, int len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum += buf[i];
    return sum;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUT PAKETİ İŞLEME
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Test cihazı SUT modunda 34-byte paketler gönderir.
 *  Paket düzeni (örnek – proje belgesine göre güncellenir):
 *    [0]    : 0xAB  (header)
 *    [1..4] : İrtifa (Big-Endian float, hPa)
 *    [5..8] : İvme X (Big-Endian float, g)
 *    [9..12]: İvme Y (Big-Endian float, g)
 *    [13..16]: İvme Z (Big-Endian float, g)
 *    ...    : Diğer alanlar
 *    [33]   : Checksum
 *
 *  ÖNEMLİ: Her ham değer Apply_Rocket_Filter / BMP180_Filter_Update
 *           üzerinden geçirilmeden sensorData'ya yazılmaz.
 */
static void Process_SUT_Packet(uint8_t *buf)
{
    /* ── Checksum doğrulama ─────────────────────────────────────────────── */
    uint8_t expected_cs = Calculate_Checksum(buf, RX_BUF_LEN - 1);
    if (buf[RX_BUF_LEN - 1] != expected_cs) {
        /*
         * Bozuk paket → tüm sensör verileri geçersiz sayılır.
         * sensorData'daki son geçerli filtrelenmiş değerler korunur.
         */
        sensorData.bmp180_valid  = 0;
        sensorData.mpu6050_valid = 0;
        sensorData.adxl345_valid = 0;
        return;
    }

    /* ── Big-Endian dönüşümü + BMP180 filtreleme ────────────────────────── */
    float raw_press = Parse_BigEndian_Float(buf, 1);   /* byte[1..4] */
    BMP180_Filter_Update(&bmp180_fs, raw_press, SAMPLE_DT_S);
    /*
     * BMP180_Filter_Update içinde:
     *   1. Outlier koruması
     *   2. EMA basınç
     *   3. İrtifa hesabı
     *   4. EMA irtifa
     *   5. sensorData.altitude_m ve sensorData.pressure_hpa güncellenir
     * → Ham raw_press değeri FSM'ye ulaşmaz.
     */

    /* ── Big-Endian dönüşümü + ADXL345 filtreleme ───────────────────────── */
    float raw_ax = Parse_BigEndian_Float(buf, 5);   /* byte[5..8]   */
    float raw_ay = Parse_BigEndian_Float(buf, 9);   /* byte[9..12]  */
    float raw_az = Parse_BigEndian_Float(buf, 13);  /* byte[13..16] */
    ADXL345_Filter_Update(&adxl345_fs, raw_ax, raw_ay, raw_az);
    /*
     * ADXL345_Filter_Update içinde:
     *   1. Medyan filtre (N=5)
     *   2. EMA
     *   3. sensorData.accel_x/y/z_g güncellenir
     * → Ham raw_ax/y/z değerleri FSM'ye ulaşmaz.
     */

    /* ── MPU6050 (gerçek sensörden okunuyorsa ayrıca güncellenir) ────────── */
    /*
     * MPU6050 DMA paketi ayrı bir UART/I2C kanalında gelebilir.
     * SUT testi sırasında jiroskop verisi aşağıdaki gibi beslenir:
     *
     *   float raw_gx = Parse_BigEndian_Float(buf, 17);
     *   float raw_gy = Parse_BigEndian_Float(buf, 21);
     *   float raw_gz = Parse_BigEndian_Float(buf, 25);
     *   MPU6050_Filter_Update(&mpu6050_fs,
     *                         raw_ax, raw_ay, raw_az,
     *                         raw_gx, raw_gy, raw_gz,
     *                         SAMPLE_DT_S);
     */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  HAL DMA CALLBACK – UART RX TAMAM
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  DMA tam bir paket aldığında otomatik çağrılır.
 *  Kesme içinde sadece filtreleme yapılır; FSM ana döngüye bırakılır.
 */
/* void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) */
void UART_RxComplete_Handler(void)  /* HAL callback olarak yeniden adlandırılır */
{
    if (uart_rx_buffer[0] == SUT_HEADER) {
        /* SUT paketi: Big-Endian dönüşüm + filtrele + sensorData'ya yaz */
        Process_SUT_Packet(uart_rx_buffer);
    }
    /* else: Diğer komut tipleri burada işlenir */

    /* DMA'yı sıfırla – bir sonraki paketi bekle */
    /* HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, RX_BUF_LEN); */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  UÇUŞ DURUM MAKİNESİ (FSM)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  FSM YALNIZCA sensorData'yı (filtrelenmiş) okur.
 *  Ham sensör değişkenlerine erişimi yoktur ve olmamalıdır.
 *
 *  Debouncing (Sıçrama Engelleme):
 *    Fırlatma algılama, irtifa eşiği ve paraşüt açma kararları
 *    ardışık N onay ile doğrulanır. Tek bir anlık değer yetmez.
 */

typedef enum {
    STATE_IDLE = 0,       /* Hazır bekliyor */
    STATE_LIFTOFF,        /* Fırlatma algılandı */
    STATE_BURNOUT,        /* Motor yanması bitti */
    STATE_COASTING,       /* Ballistic kısım */
    STATE_APOGEE,         /* Tepe noktası */
    STATE_DESCENT,        /* İniş */
    STATE_DROGUE_DEPLOY,  /* Fren paraşütü */
    STATE_MAIN_DEPLOY,    /* Ana paraşüt */
    STATE_LANDED          /* İniş tamamlandı */
} FlightState_t;

static FlightState_t current_state = STATE_IDLE;

/* Ardışık doğrulama sayaçları (debouncing) */
static int apogee_confirm_count   = 0;
static int liftoff_confirm_count  = 0;
#define APOGEE_CONFIRM_NEEDED    5   /* 5 ardışık negatif hız onayı */
#define LIFTOFF_CONFIRM_NEEDED   3   /* 3 ardışık ivme eşiği aşımı */

/* Eşik değerleri */
#define LIFTOFF_ACCEL_THRESHOLD  2.5f   /* g – fırlatma ivmesi */
#define APOGEE_ALT_MIN           50.0f  /* m – tepe tespiti için minimum irtifa */
#define DROGUE_ALT_THRESHOLD    300.0f  /* m – fren paraşütü açma irtifası */
#define MAIN_CHUTE_ALT          100.0f  /* m – ana paraşüt açma irtifası */

static float prev_altitude = 0.0f;

void RunFlightStateMachine(void)
{
    /*
     * sensorData buradan okunur.
     * Tüm değerler filtrelenmiştir; ham sensör verisine erişim yoktur.
     */
    if (!sensorData.bmp180_valid || !sensorData.adxl345_valid) {
        /* Geçersiz veri → güvenli durumda kal */
        return;
    }

    float alt   = sensorData.altitude_m;
    float az    = sensorData.accel_z_g;
    float pitch = sensorData.pitch_deg;
    float roll  = sensorData.roll_deg;

    /* Hız tahmini: basit türev (irtifa farkı / dt) */
    float velocity_est = (alt - prev_altitude) / SAMPLE_DT_S;
    prev_altitude = alt;

    switch (current_state) {

        case STATE_IDLE:
            /* Fırlatma tespiti – ardışık ivme doğrulaması (debouncing) */
            if (az > LIFTOFF_ACCEL_THRESHOLD) {
                liftoff_confirm_count++;
                if (liftoff_confirm_count >= LIFTOFF_CONFIRM_NEEDED) {
                    current_state = STATE_LIFTOFF;
                    current_telemetry.status.liftoff_detected = 1;
                    liftoff_confirm_count = 0;
                }
            } else {
                liftoff_confirm_count = 0; /* Sayacı sıfırla – tek spike yetmez */
            }
            break;

        case STATE_LIFTOFF:
            /* Motor yanması sona erdiğinde ivme 1g'ye düşer */
            if (az < 1.2f) {
                current_state = STATE_BURNOUT;
                current_telemetry.status.motor_burnout = 1;
            }
            /* Eğim hatası kontrolü */
            if (pitch > 30.0f || roll > 30.0f) {
                current_telemetry.status.tilt_or_accel_fault = 1;
            }
            break;

        case STATE_BURNOUT:
            current_state = STATE_COASTING;
            break;

        case STATE_COASTING:
            /* Tepe tespiti: ardışık 5 defa negatif hız → apogee */
            if (alt > APOGEE_ALT_MIN && velocity_est < 0.0f) {
                apogee_confirm_count++;
                if (apogee_confirm_count >= APOGEE_CONFIRM_NEEDED) {
                    current_state = STATE_APOGEE;
                    apogee_confirm_count = 0;
                }
            } else if (velocity_est >= 0.0f) {
                apogee_confirm_count = 0; /* Hız pozitife döndüyse sıfırla */
            }
            break;

        case STATE_APOGEE:
            current_state = STATE_DESCENT;
            current_telemetry.status.descent_detected = 1;
            break;

        case STATE_DESCENT:
            if (alt <= DROGUE_ALT_THRESHOLD) {
                current_state = STATE_DROGUE_DEPLOY;
                current_telemetry.status.drogue_chute_cmd = 1;
                /* Fren paraşütü tetikleme komutu buradan gönderilir */
                /* HAL_GPIO_WritePin(DROGUE_GPIO_Port, DROGUE_Pin, GPIO_PIN_SET); */
            }
            break;

        case STATE_DROGUE_DEPLOY:
            current_telemetry.status.threshold_alt_passed = 1;
            if (alt <= MAIN_CHUTE_ALT) {
                current_state = STATE_MAIN_DEPLOY;
                current_telemetry.status.main_chute_cmd = 1;
                /* Ana paraşüt tetikleme komutu */
                /* HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_SET); */
            }
            break;

        case STATE_MAIN_DEPLOY:
            if (az > 0.9f && az < 1.1f && alt < 10.0f) {
                /* 1g sabit ivme + düşük irtifa → iniş tamamlandı */
                current_state = STATE_LANDED;
            }
            break;

        case STATE_LANDED:
            /* Son durum – sistem güvenli modda */
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SİSTEM BAŞLATMA
 * ═══════════════════════════════════════════════════════════════════════════ */
void System_Init(void)
{
    /* ── Filtre başlatmaları ─────────────────────────────────────────────── */

    /* BMP180: İlk basınç okuması referans (zemin) olarak kullanılır.
     * Gerçek projede: float p0 = BMP180_ReadPressure(); */
    float initial_press = ISA_P0_HPA; /* Yer testi için standart atmosfer */
    BMP180_Filter_Init(&bmp180_fs, initial_press);

    /* MPU6050: Kalibrasyon – sensör hareketsizken çağrılmalı */
    MPU6050_Filter_Calibrate(&mpu6050_fs);

    /* ADXL345 */
    ADXL345_Filter_Init(&adxl345_fs);

    /* ── DMA dinlemeyi başlat ────────────────────────────────────────────── */
    /* HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, RX_BUF_LEN); */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  ANA DÖNGÜ (Super-Loop)
 * ═══════════════════════════════════════════════════════════════════════════ */
int main_ukb(void)  /* Gerçek projede: int main(void) */
{
    /* HAL_Init(); */
    /* SystemClock_Config(); */
    System_Init();

    uint32_t last_telemetry_tick = 0; /* HAL_GetTick() ile eşlenecek */

    while (1) {
        /*
         * FSM YALNIZCA FİLTRELENMİŞ TEMİZ VERİYLE ÇALIŞIR.
         * DMA callback'i arka planda sensorData'yı günceller.
         * RunFlightStateMachine() ham sensör değişkenlerine erişmez.
         */
        RunFlightStateMachine();

        /* ── 100ms'de bir telemetri paketi gönder ───────────────────────── */
        uint32_t now = 0; /* HAL_GetTick() */
        if (now - last_telemetry_tick >= 100) {

            uart_tx_buffer[0] = TELEM_HEADER;
            uart_tx_buffer[1] = current_telemetry.payload_bytes[0];
            uart_tx_buffer[2] = current_telemetry.payload_bytes[1];
            uart_tx_buffer[3] = Calculate_Checksum(uart_tx_buffer, 3);
            uart_tx_buffer[4] = TELEM_FOOTER_CR;
            uart_tx_buffer[5] = TELEM_FOOTER_LF;

            /* DMA TX – işlemciyi bloke etmez */
            /* HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 6); */

            last_telemetry_tick = now;
        }

        /* ── SİT testi: gerçek sensör okuma döngüsü ─────────────────────── */
        /*
         * SİT modunda (canlı sensörler varken):
         *
         *   float p   = BMP180_ReadPressure();
         *   BMP180_Filter_Update(&bmp180_fs, p, SAMPLE_DT_S);
         *
         *   float ax, ay, az, gx, gy, gz;
         *   MPU6050_ReadAll(&ax, &ay, &az, &gx, &gy, &gz);
         *   MPU6050_Filter_Update(&mpu6050_fs, ax, ay, az, gx, gy, gz, SAMPLE_DT_S);
         *
         *   ADXL345_ReadAll(&ax, &ay, &az);
         *   ADXL345_Filter_Update(&adxl345_fs, ax, ay, az);
         *
         * Her çağrı sensorData'yı günceller.
         * FSM yalnızca sensorData'dan okur.
         */
    }

    return 0;
}
