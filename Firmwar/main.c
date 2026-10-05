/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Adaptive Software-Defined Sonar Transmitter
  *
  * STM32F407G-DISC1 / STM32F407VGT6
  *
  * Prototype:
  *   5 potentiometers emulate environmental sensors:
  *   PA1 = Temperature
  *   PA2 = Depth
  *   PA3 = Turbidity
  *   PB0 = Salinity
  *   PB1 = Battery
  *
  * Output:
  *   PA4 = DAC1_OUT1
  *
  * Processing:
  *   ADC scan + DMA -> Environment quantization
  *   -> RAM adaptive LUT/cache -> waveform configuration
  *   -> 1024-point sine LUT -> adaptive Hann/Hamming/Blackman window -> DAC buffer
  *   -> TIM6 2 MHz trigger -> DAC DMA
  *
  * LUTs:
  *   1. 1024-point sine LUT for waveform synthesis
  *   2. RAM adaptive LUT/cache for calculated sonar configurations
  *
  * Waveform LEDs:
  *   LD3 = CW, LD4 = LFM, LD5 = Geometric, LD6 = Barker-13
  *
  * DMA used:
  *   ADC1 DMA + DAC1 DMA only. TIM6 is a trigger source, not a DMA source.
  *   USART DMA/USART application code is not used.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usb_host.h"

#include <math.h>
#include <stdint.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC_Init(void);
void MX_USB_HOST_Process(void);
void Error_Handler(void);

/* ============================================================================
 * APPLICATION CONFIGURATION
 * ========================================================================== */

#define PI                          3.14159265358979323846f

#define ADC_CHANNEL_COUNT           5U
#define ADC_FULL_SCALE              4095.0f

#define DAC_MID                     2048.0f
#define DAC_PEAK                    2047.0f
#define DAC_MAX                     4095U

#define SINE_LUT_SIZE               1024U
#define DAC_BUFFER_SIZE             32768U
#define DAC_HALF_SIZE               (DAC_BUFFER_SIZE / 2U)
#define DAC_FRAME_SAMPLES           DAC_HALF_SIZE

/* TIM6 = 84 MHz / (0+1) / (41+1) = 2 MHz */
#define SAMPLE_RATE_HZ              2000000.0f

/* Nyquist = 1 MHz. Adaptive carrier search: 100 kHz to 500 kHz. */
#define SEARCH_MIN_FREQUENCY_HZ     100000.0f
#define SEARCH_MAX_FREQUENCY_HZ     500000.0f

/* Real transmitter / amplifier usable band. */
#define TX_MIN_FREQUENCY_HZ         100000.0f
#define TX_MAX_FREQUENCY_HZ         500000.0f
#define COARSE_STEP_HZ              50000.0f
#define FINE_STEP_HZ                5000.0f
#define FINE_SEARCH_HALF_SPAN_HZ    25000.0f

/* Candidate-frequency cost weights. */
#define COST_W_TWO_WAY_TL           1.0f
#define COST_W_ENERGY               8.0f
#define COST_W_HARDWARE             1000000.0f

/* Battery safety */
#define LOW_BATTERY_V               10.5f

/* Adaptive window-selection thresholds. */
#define TURBIDITY_HIGH_THRESHOLD    0.70f
#define BATTERY_HIGH_THRESHOLD      11.80f

/* ============================================================================
 * MISSION / SONAR-EQUATION PARAMETERS
 *
 * These are the values from the current sonar design example.
 * They are mission parameters, not sensor measurements.
 * ========================================================================== */
#define TARGET_RANGE_M              80.0f
#define REQUIRED_RANGE_RES_M        0.05f
#define REQUIRED_SNR_DB             10.0f
#define NOISE_LEVEL_DB              50.0f
#define DIRECTIVITY_INDEX_DB        10.0f
#define TARGET_STRENGTH_DB          10.0f
#define REQUIRED_TB                  100.0f
#define WATER_PH                      8.1f

/*
 * Source-level -> normalized DAC calibration.
 *
 * IMPORTANT: These two points are the illustrative calibration values used
 * in the project technical design (0.60 -> 120 dB, 0.80 -> 125 dB).
 * Replace them with measured transducer + amplifier calibration data before
 * using this as a physical source-level controller.
 */
#define CAL_SL_LOW_DB               120.0f
#define CAL_AMP_LOW                 0.60f
#define CAL_SL_HIGH_DB              125.0f
#define CAL_AMP_HIGH                0.80f

/* STM32F407G-DISC1 onboard waveform LEDs */
#define WAVEFORM_LED_PORT           GPIOD

/* Recalculate adaptive parameters every 100 ms */
#define UPDATE_PERIOD_MS             100U

/* ============================================================================
 * TYPES
 * ========================================================================== */

typedef enum
{
    WAVE_CW = 0,
    WAVE_LFM,
    WAVE_GEOMETRIC,
    WAVE_BARKER13

} WaveformType;

typedef enum
{
    WINDOW_HANN = 0,
    WINDOW_HAMMING,
    WINDOW_BLACKMAN
} WindowType;

typedef struct
{
    float temperature_C;
    float salinity_ppt;
    float depth_m;
    float turbidity;
    float battery_V;

} Environment_t;

typedef struct
{
    float sound_speed;
    float center_frequency;
    float bandwidth;
    float f_start;
    float f_end;
    float pulse_duration;
    float amplitude;
    float transmission_loss;
    float two_way_loss;
    float required_source_level;
    float range_resolution;
    float time_bandwidth;
    float optimization_cost;
    WindowType window;
    WaveformType waveform;

} SonarConfig_t;

/* ============================================================================
 * GLOBAL APPLICATION DATA
 * ========================================================================== */

volatile uint16_t adc_buffer[ADC_CHANNEL_COUNT];

uint16_t dac_buffer[DAC_BUFFER_SIZE];

/* Continuous circular DAC DMA ping-pong state. */
volatile uint8_t dac_half_free[2] = {0U, 0U};
volatile uint8_t pending_config_valid = 0U;
static SonarConfig_t pending_sonar_config;

float sine_lut[SINE_LUT_SIZE];

Environment_t env;
SonarConfig_t sonar;

static uint32_t last_update_ms = 0U;

/* Barker-13 phase code */
static const int8_t barker13[13] =
{
     1,  1,  1,  1,  1,
    -1, -1,
     1,  1,
    -1,
     1,
    -1,
     1
};

/* ============================================================================
 * RAM-ONLY ADAPTIVE LUT / CACHE
 *
 * No SD card / FatFs is used in this version.
 *
 * Flow:
 *   sensors -> quantized state key -> RAM cache lookup
 *      -> HIT  : use cached sonar configuration
 *      -> MISS : calculate configuration -> store in RAM cache
 *
 * The cache is intentionally small and fixed-size so it is suitable for
 * STM32 RAM. When full, the least-recently-used entry is replaced.
 * ========================================================================== */
#define RAM_LUT_SIZE                32U

/* Sensor quantization used to create a repeatable state key. */
#define TEMP_KEY_STEP_C             0.5f
#define DEPTH_KEY_STEP_M            1.0f
#define TURB_KEY_STEP               0.01f
#define SALINITY_KEY_STEP_PPT       0.1f
#define BATTERY_KEY_STEP_V          0.1f

typedef struct
{
    uint16_t temp_key;
    uint16_t depth_key;
    uint16_t turbidity_key;
    uint16_t salinity_key;
    uint16_t battery_key;

    SonarConfig_t config;

    uint32_t last_used;
    uint8_t valid;
} RAM_LUT_Record;

static RAM_LUT_Record ram_lut[RAM_LUT_SIZE];
static uint32_t ram_lut_time = 0U;
static uint32_t ram_lut_hits = 0U;
static uint32_t ram_lut_misses = 0U;

/* --------------------------------------------------------------------------
 * Utility helpers
 * ------------------------------------------------------------------------ */
static uint16_t QuantizeKey(float value, float step)
{
    if (value <= 0.0f)
        return 0U;

    return (uint16_t)(value / step + 0.5f);
}

static void MakeLUTKey(uint16_t *temp_key,
                       uint16_t *depth_key,
                       uint16_t *turb_key,
                       uint16_t *sal_key,
                       uint16_t *battery_key)
{
    *temp_key =
        QuantizeKey(env.temperature_C, TEMP_KEY_STEP_C);

    *depth_key =
        QuantizeKey(env.depth_m, DEPTH_KEY_STEP_M);

    *turb_key =
        QuantizeKey(env.turbidity, TURB_KEY_STEP);

    *sal_key =
        QuantizeKey(env.salinity_ppt, SALINITY_KEY_STEP_PPT);

    *battery_key =
        QuantizeKey(env.battery_V, BATTERY_KEY_STEP_V);
}

static uint8_t LUTKeyMatches(const RAM_LUT_Record *r,
                             uint16_t temp_key,
                             uint16_t depth_key,
                             uint16_t turb_key,
                             uint16_t sal_key,
                             uint16_t battery_key)
{
    return (r->valid != 0U &&
            r->temp_key == temp_key &&
            r->depth_key == depth_key &&
            r->turbidity_key == turb_key &&
            r->salinity_key == sal_key &&
            r->battery_key == battery_key);
}

/* --------------------------------------------------------------------------
 * Mackenzie sound-speed model
 *
 * T = temperature in deg C
 * S = salinity in ppt
 * D = depth in m
 * ------------------------------------------------------------------------ */
static float MackenzieSoundSpeed(float T, float S, float D)
{
    float c;

    c = 1448.96f
      + 4.591f * T
      - 0.05304f * T * T
      + 0.0002374f * T * T * T
      + 1.340f * (S - 35.0f)
      + 0.01630f * D
      + 0.000000167f * D * D;

    return c;
}

/* --------------------------------------------------------------------------
 * Prototype attenuation estimate.
 *
 * This is a firmware adaptation factor, not a calibrated hydro-acoustic
 * measurement. It is used only to make the adaptive configuration respond
 * to depth, turbidity and operating frequency.
 * ------------------------------------------------------------------------ */
static float FrancoisGarrisonAlpha(float frequency_hz,
                                     float T,
                                     float S,
                                     float D,
                                     float pH,
                                     float c)
{
    float f = frequency_hz / 1000.0f;
    float Tk = T + 273.15f;
    float A1, P1, f1, alpha1;
    float A2, P2, f2, alpha2;
    float A3, P3, alpha3;

    if (f < 0.001f) f = 0.001f;
    if (S < 0.1f) S = 0.1f;
    if (c < 1000.0f) c = 1500.0f;
    if (Tk < 250.0f) Tk = 250.0f;

    A1 = (8.86f / c) * powf(10.0f, 0.78f * pH - 5.0f);
    P1 = 1.0f;
    f1 = 2.8f * sqrtf(S / 35.0f) *
         powf(10.0f, 4.0f - 1245.0f / Tk);
    alpha1 = (A1 * P1 * f1 * f * f) /
             (f1 * f1 + f * f);

    A2 = 21.44f * (S / c) * (1.0f + 0.025f * T);
    P2 = 1.0f - 1.37e-4f * D + 6.2e-9f * D * D;
    f2 = (8.17f * powf(10.0f, 8.0f - 1990.0f / Tk)) /
         (1.0f + 0.0018f * (S - 35.0f));
    alpha2 = (A2 * P2 * f2 * f * f) /
             (f2 * f2 + f * f);

    if (T <= 20.0f)
    {
        A3 = 4.937e-4f - 2.590e-5f * T
           + 9.11e-7f * T * T - 1.5e-8f * T * T * T;
    }
    else
    {
        A3 = 3.964e-4f - 1.146e-5f * T
           + 1.45e-7f * T * T - 6.5e-10f * T * T * T;
    }

    P3 = 1.0f - 3.83e-5f * D + 4.9e-10f * D * D;
    alpha3 = A3 * P3 * f * f;

    return alpha1 + alpha2 + alpha3;
}

static float EstimateTransmissionLoss(float frequency_hz,
                                       float range_m,
                                       float T,
                                       float S,
                                       float D,
                                       float pH,
                                       float sound_speed)
{
    float spreading_loss;
    float absorption_db_per_km;
    float range_km;
    float absorption_loss;

    if (range_m < 1.0f)
        range_m = 1.0f;

    spreading_loss = 20.0f * log10f(range_m);
    range_km = range_m / 1000.0f;

    absorption_db_per_km =
        FrancoisGarrisonAlpha(frequency_hz,
                               T, S, D, pH, sound_speed);

    absorption_loss =
        absorption_db_per_km * range_km;

    return spreading_loss + absorption_loss;
}

static float CalculateRequiredSourceLevel(float two_way_loss_db);

static float EvaluateFrequencyCost(float fc,
                                   float bandwidth,
                                   float sound_speed,
                                   float *tl_out,
                                   float *two_way_out,
                                   float *sl_out)
{
    float f_start = fc - 0.5f * bandwidth;
    float f_end   = fc + 0.5f * bandwidth;
    float tl, two_way, sl;
    float energy_penalty;
    float normalized_frequency;

    if (fc < TX_MIN_FREQUENCY_HZ || fc > TX_MAX_FREQUENCY_HZ)
        return COST_W_HARDWARE;

    if (f_start < TX_MIN_FREQUENCY_HZ ||
        f_end > TX_MAX_FREQUENCY_HZ)
        return COST_W_HARDWARE;

    if (f_end >= 0.5f * SAMPLE_RATE_HZ)
        return COST_W_HARDWARE;

    tl = EstimateTransmissionLoss(fc,
                                   TARGET_RANGE_M,
                                   env.temperature_C,
                                   env.salinity_ppt,
                                   env.depth_m,
                                   WATER_PH,
                                   sound_speed);

    two_way = 2.0f * tl;
    sl = CalculateRequiredSourceLevel(two_way);

    normalized_frequency =
        fc / TX_MAX_FREQUENCY_HZ;

    energy_penalty =
        COST_W_ENERGY * normalized_frequency;

    if (tl_out != NULL) *tl_out = tl;
    if (two_way_out != NULL) *two_way_out = two_way;
    if (sl_out != NULL) *sl_out = sl;

    return COST_W_TWO_WAY_TL * two_way +
           energy_penalty;
}

static float SearchBestFrequency(float bandwidth,
                                 float sound_speed)
{
    float best_fc = SEARCH_MIN_FREQUENCY_HZ;
    float best_cost = COST_W_HARDWARE;
    float coarse_best = best_fc;
    float cost;

    for (float fc = SEARCH_MIN_FREQUENCY_HZ;
         fc <= SEARCH_MAX_FREQUENCY_HZ;
         fc += COARSE_STEP_HZ)
    {
        cost = EvaluateFrequencyCost(fc,
                                     bandwidth,
                                     sound_speed,
                                     NULL, NULL, NULL);

        if (cost < best_cost)
        {
            best_cost = cost;
            coarse_best = fc;
        }
    }

    best_cost = COST_W_HARDWARE;

    for (float fc = coarse_best - FINE_SEARCH_HALF_SPAN_HZ;
         fc <= coarse_best + FINE_SEARCH_HALF_SPAN_HZ;
         fc += FINE_STEP_HZ)
    {
        cost = EvaluateFrequencyCost(fc,
                                     bandwidth,
                                     sound_speed,
                                     NULL, NULL, NULL);

        if (cost < best_cost)
        {
            best_cost = cost;
            best_fc = fc;
        }
    }

    return best_fc;
}

/* --------------------------------------------------------------------------
 * Active sonar equation.
 *
 * SNR = SL - 2TL - (NL - DI) + TS
 *
 * If two_way_loss = 2TL, then:
 * SL_required = SNR_required + two_way_loss + NL - DI - TS
 * ------------------------------------------------------------------------ */
static float CalculateRequiredSourceLevel(float two_way_loss_db)
{
    return REQUIRED_SNR_DB
         + two_way_loss_db
         + NOISE_LEVEL_DB
         - DIRECTIVITY_INDEX_DB
         - TARGET_STRENGTH_DB;
}

/* --------------------------------------------------------------------------
 * Convert required acoustic source level to normalized DAC amplitude.
 *
 * This is intentionally a calibration function rather than a direct
 * physics conversion. Acoustic dB cannot be converted to DAC voltage
 * without measured amplifier/transducer calibration.
 * ------------------------------------------------------------------------ */
static float SourceLevelToAmplitude(float source_level_db)
{
    float amplitude;

    if (source_level_db <= CAL_SL_LOW_DB)
        return CAL_AMP_LOW;

    if (source_level_db >= CAL_SL_HIGH_DB)
        return 1.0f;

    amplitude =
        CAL_AMP_LOW
        + (source_level_db - CAL_SL_LOW_DB)
          * (CAL_AMP_HIGH - CAL_AMP_LOW)
          / (CAL_SL_HIGH_DB - CAL_SL_LOW_DB);

    if (amplitude < 0.0f) amplitude = 0.0f;
    if (amplitude > 1.0f) amplitude = 1.0f;

    return amplitude;
}

/* --------------------------------------------------------------------------
 * Calculate a new sonar configuration for a state that is not in RAM.
 *
 * The sensor values determine the configuration; the LUT is therefore a
 * cache of previously calculated states rather than a fixed 48-state table.
 * ------------------------------------------------------------------------ */
static WindowType SelectWindow(float turbidity,
                               float battery_V,
                               WaveformType waveform);

static void CalculateAdaptiveConfig(SonarConfig_t *cfg)
{
    float sound_speed;
    float bandwidth;
    float pulse_duration;
    float fc;
    float min_freq;
    float max_freq;
    float transmission_loss;
    float two_way_loss;
    float required_source_level;
    float amplitude;
    float cost;

    sound_speed =
        MackenzieSoundSpeed(env.temperature_C,
                            env.salinity_ppt,
                            env.depth_m);

    bandwidth =
        sound_speed /
        (2.0f * REQUIRED_RANGE_RES_M);

    pulse_duration =
        REQUIRED_TB / bandwidth;

    if (pulse_duration > 0.008f)
        pulse_duration = 0.008f;

    /* Low-battery safety: force the lowest supported carrier. */
    if (env.battery_V < LOW_BATTERY_V)
    {
        fc = TX_MIN_FREQUENCY_HZ;

        min_freq = fc - 0.5f * bandwidth;
        max_freq = fc + 0.5f * bandwidth;

        if (min_freq < TX_MIN_FREQUENCY_HZ)
            min_freq = TX_MIN_FREQUENCY_HZ;

        if (max_freq > TX_MAX_FREQUENCY_HZ)
            max_freq = TX_MAX_FREQUENCY_HZ;

        cfg->sound_speed = sound_speed;
        cfg->center_frequency = fc;
        cfg->bandwidth = max_freq - min_freq;
        cfg->f_start = min_freq;
        cfg->f_end = max_freq;
        cfg->pulse_duration = 0.001f;
        cfg->amplitude = 0.30f;
        cfg->transmission_loss = 0.0f;
        cfg->two_way_loss = 0.0f;
        cfg->required_source_level = 0.0f;
        cfg->range_resolution = REQUIRED_RANGE_RES_M;
        cfg->time_bandwidth = REQUIRED_TB;
        cfg->optimization_cost = 0.0f;
        cfg->waveform = WAVE_CW;
        cfg->window = WINDOW_HAMMING;
        return;
    }

    /* ================================================================
     * ADAPTIVE FREQUENCY: 100 kHz -> 500 kHz
     *
     * Coarse: 50 kHz steps
     * Fine:   +/-25 kHz around coarse winner, 5 kHz steps
     * Cost:   two-way transmission loss + frequency energy penalty
     * ================================================================ */
    fc = SearchBestFrequency(bandwidth, sound_speed);

    min_freq = fc - 0.5f * bandwidth;
    max_freq = fc + 0.5f * bandwidth;

    if (min_freq < TX_MIN_FREQUENCY_HZ)
        min_freq = TX_MIN_FREQUENCY_HZ;

    if (max_freq > TX_MAX_FREQUENCY_HZ)
        max_freq = TX_MAX_FREQUENCY_HZ;

    bandwidth = max_freq - min_freq;

    if (bandwidth < 1.0f)
        bandwidth = 1.0f;

    pulse_duration =
        REQUIRED_TB / bandwidth;

    if (pulse_duration > 0.008f)
        pulse_duration = 0.008f;

    cost =
        EvaluateFrequencyCost(fc,
                              bandwidth,
                              sound_speed,
                              &transmission_loss,
                              &two_way_loss,
                              &required_source_level);

    amplitude =
        SourceLevelToAmplitude(required_source_level);

    /* Keep the existing waveform adaptation from the gap-free code. */
    {
        float depth_n = env.depth_m / 200.0f;
        float turb_n = env.turbidity;
        float clarity;

        if (depth_n < 0.0f) depth_n = 0.0f;
        if (depth_n > 1.0f) depth_n = 1.0f;
        if (turb_n < 0.0f) turb_n = 0.0f;
        if (turb_n > 1.0f) turb_n = 1.0f;

        clarity =
            1.0f -
            (0.65f * turb_n +
             0.35f * depth_n);

        if (clarity >= 0.75f)
            cfg->waveform = WAVE_CW;
        else if (clarity >= 0.50f)
            cfg->waveform = WAVE_LFM;
        else if (clarity >= 0.25f)
            cfg->waveform = WAVE_GEOMETRIC;
        else
            cfg->waveform = WAVE_BARKER13;
    }

    /*
     * Adaptive window selection:
     *   High turbidity -> Blackman for stronger sidelobe suppression.
     *   CW + lower battery -> Hamming for the low-energy/simple-CW case.
     *   Otherwise -> Hann as the balanced default.
     *
     * These thresholds are engineering parameters and can be tuned.
     */
    cfg->window = SelectWindow(env.turbidity,
                               env.battery_V,
                               cfg->waveform);

    cfg->sound_speed = sound_speed;
    cfg->center_frequency = fc;
    cfg->bandwidth = bandwidth;
    cfg->f_start = min_freq;
    cfg->f_end = max_freq;
    cfg->pulse_duration = pulse_duration;
    cfg->amplitude = amplitude;
    cfg->transmission_loss = transmission_loss;
    cfg->two_way_loss = two_way_loss;
    cfg->required_source_level = required_source_level;
    cfg->range_resolution = REQUIRED_RANGE_RES_M;
    cfg->time_bandwidth = REQUIRED_TB;
    cfg->optimization_cost = cost;
}

/* --------------------------------------------------------------------------
 * Find the configuration in RAM. Returns 1 on hit, 0 on miss.
 * ------------------------------------------------------------------------ */
static uint8_t RAM_LUT_Find(uint16_t temp_key,
                            uint16_t depth_key,
                            uint16_t turb_key,
                            uint16_t sal_key,
                            uint16_t battery_key,
                            SonarConfig_t *cfg)
{
    for (uint32_t i = 0U; i < RAM_LUT_SIZE; i++)
    {
        if (LUTKeyMatches(&ram_lut[i],
                          temp_key,
                          depth_key,
                          turb_key,
                          sal_key,
                          battery_key))
        {
            ram_lut[i].last_used = ++ram_lut_time;
            *cfg = ram_lut[i].config;
            ram_lut_hits++;
            return 1U;
        }
    }

    ram_lut_misses++;
    return 0U;
}

/* --------------------------------------------------------------------------
 * Store a new configuration. If the cache is full, replace the LRU entry.
 * ------------------------------------------------------------------------ */
static void RAM_LUT_Store(uint16_t temp_key,
                          uint16_t depth_key,
                          uint16_t turb_key,
                          uint16_t sal_key,
                          uint16_t battery_key,
                          const SonarConfig_t *cfg)
{
    uint32_t selected = 0U;
    uint32_t oldest = 0xFFFFFFFFUL;

    for (uint32_t i = 0U; i < RAM_LUT_SIZE; i++)
    {
        if (ram_lut[i].valid == 0U)
        {
            selected = i;
            oldest = 0U;
            break;
        }

        if (ram_lut[i].last_used < oldest)
        {
            oldest = ram_lut[i].last_used;
            selected = i;
        }
    }

    ram_lut[selected].temp_key = temp_key;
    ram_lut[selected].depth_key = depth_key;
    ram_lut[selected].turbidity_key = turb_key;
    ram_lut[selected].salinity_key = sal_key;
    ram_lut[selected].battery_key = battery_key;
    ram_lut[selected].config = *cfg;
    ram_lut[selected].last_used = ++ram_lut_time;
    ram_lut[selected].valid = 1U;
}

/* --------------------------------------------------------------------------
 * Main adaptive lookup function.
 * ------------------------------------------------------------------------ */
static void ApplyAdaptiveLUT(void)
{
    uint16_t temp_key;
    uint16_t depth_key;
    uint16_t turb_key;
    uint16_t sal_key;
    uint16_t battery_key;

    MakeLUTKey(&temp_key,
               &depth_key,
               &turb_key,
               &sal_key,
               &battery_key);

    /* Fast path: configuration already exists in RAM. */
    if (RAM_LUT_Find(temp_key,
                     depth_key,
                     turb_key,
                     sal_key,
                     battery_key,
                     &sonar) != 0U)
    {
        return;
    }

    /* Cache miss: calculate from the current environmental state. */
    CalculateAdaptiveConfig(&sonar);

    /* Save the newly calculated configuration in RAM. */
    RAM_LUT_Store(temp_key,
                  depth_key,
                  turb_key,
                  sal_key,
                  battery_key,
                  &sonar);
}

/* ============================================================================
 * FUNCTION PROTOTYPES / BASIC WAVEFORM HELPERS
 * ========================================================================== */

static void InitSineLUT(void);
static void ReadEnvironment(void);
static int BarkerSign(uint32_t n, uint32_t N);
static float WindowFunction(WindowType type, uint32_t n, uint32_t N);
static void GenerateWaveformHalf(uint16_t *buffer, const SonarConfig_t *cfg);
static void GenerateWaveform(void);
static void StartSonarOutput(void);
static void UpdateWaveformLEDs(void);
static void ServiceWaveformBuffer(void);

/* ============================================================================
 * MAIN
 * ========================================================================== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USB_HOST_Init();
    MX_ADC1_Init();
    MX_TIM6_Init();
    MX_DAC_Init();

    InitSineLUT();

    if (HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)adc_buffer,
                          ADC_CHANNEL_COUNT) != HAL_OK)
    {
        Error_Handler();
    }

    ReadEnvironment();
    ApplyAdaptiveLUT();
    UpdateWaveformLEDs();

    /* Fill both DMA halves before the DAC starts. */
    GenerateWaveform();
    StartSonarOutput();

    last_update_ms = HAL_GetTick();

    while (1)
    {
        MX_USB_HOST_Process();

        /* Refill whichever DMA half has just finished. */
        ServiceWaveformBuffer();

        /* Recalculate the adaptive state slowly; DAC/TIM6 never stop. */
        if ((HAL_GetTick() - last_update_ms) >= UPDATE_PERIOD_MS)
        {
            last_update_ms = HAL_GetTick();

            ReadEnvironment();
            ApplyAdaptiveLUT();
            UpdateWaveformLEDs();

            /* New configuration is used on the next safe DMA-half refill. */
            pending_sonar_config = sonar;
            pending_config_valid = 1U;
        }
    }
}


static void InitSineLUT(void)
{
    for (uint32_t i = 0U; i < SINE_LUT_SIZE; i++)
    {
        sine_lut[i] = sinf((2.0f * PI * (float)i) /
                           (float)SINE_LUT_SIZE);
    }
}

static void ReadEnvironment(void)
{
    env.temperature_C =
        5.0f + ((float)adc_buffer[0] / ADC_FULL_SCALE) * 17.0f;

    env.depth_m =
        ((float)adc_buffer[1] / ADC_FULL_SCALE) * 200.0f;

    env.turbidity =
        ((float)adc_buffer[2] / ADC_FULL_SCALE);

    env.salinity_ppt =
        30.0f + ((float)adc_buffer[3] / ADC_FULL_SCALE) * 5.0f;

    env.battery_V =
        9.0f + ((float)adc_buffer[4] / ADC_FULL_SCALE) * 7.8f;
}

static int BarkerSign(uint32_t n, uint32_t N)
{
    uint32_t chip;

    if (N == 0U)
        return 1;

    chip = (n * 13U) / N;
    if (chip >= 13U)
        chip = 12U;

    return barker13[chip];
}

/*
 * Adaptive window-selection decision layer.
 *
 * Decision rules:
 *   turbidity >= 0.70              -> Blackman
 *   waveform == CW and battery < 11.80 V -> Hamming
 *   otherwise                      -> Hann
 */
static WindowType SelectWindow(float turbidity,
                               float battery_V,
                               WaveformType waveform)
{
    if (turbidity >= TURBIDITY_HIGH_THRESHOLD)
        return WINDOW_BLACKMAN;

    if ((waveform == WAVE_CW) &&
        (battery_V < BATTERY_HIGH_THRESHOLD))
        return WINDOW_HAMMING;

    return WINDOW_HANN;
}

/*
 * Mathematical window functions.
 *
 * x[n] = 2*pi*n/(N-1)
 *
 * Hann:
 *     w[n] = 0.50 - 0.50*cos(x)
 *
 * Hamming:
 *     w[n] = 0.54 - 0.46*cos(x)
 *
 * Blackman:
 *     w[n] = 0.42 - 0.50*cos(x) + 0.08*cos(2*x)
 */
static float WindowFunction(WindowType type, uint32_t n, uint32_t N)
{
    float x;

    if (N <= 1U)
        return 1.0f;

    x = (2.0f * PI * (float)n) / (float)(N - 1U);

    switch (type)
    {
        case WINDOW_HAMMING:
            return 0.54f - 0.46f * cosf(x);

        case WINDOW_BLACKMAN:
            return 0.42f
                 - 0.50f * cosf(x)
                 + 0.08f * cosf(2.0f * x);

        case WINDOW_HANN:
        default:
            return 0.50f - 0.50f * cosf(x);
    }
}

static void GenerateWaveformHalf(uint16_t *buffer, const SonarConfig_t *cfg)
{
    uint32_t N;
    uint32_t phase = 0U;
    float freq = cfg->f_start;
    float geometric_ratio = 1.0f;

    N = (uint32_t)(cfg->pulse_duration * SAMPLE_RATE_HZ);

    if (N < 2U) N = 2U;
    if (N > DAC_FRAME_SAMPLES) N = DAC_FRAME_SAMPLES;

    if (cfg->waveform == WAVE_GEOMETRIC && cfg->f_start > 0.0f)
    {
        geometric_ratio =
            expf(logf(cfg->f_end / cfg->f_start) /
                 (float)(N - 1U));
    }

    for (uint32_t n = 0U; n < DAC_FRAME_SAMPLES; n++)
    {
        float sample = 0.0f;

        if (n < N)
        {
            float window;
            uint32_t phase_inc_u32;

            if (cfg->waveform == WAVE_LFM)
            {
                freq = cfg->f_start +
                       cfg->bandwidth *
                       ((float)n / (float)(N - 1U));
            }
            else if (cfg->waveform == WAVE_GEOMETRIC)
            {
                if (n > 0U)
                    freq *= geometric_ratio;
            }
            else
            {
                freq = cfg->center_frequency;
            }

            /*
             * DDS phase accumulator.
             * 32-bit phase gives fine frequency resolution while the upper
             * 10 bits address the 1024-point sine LUT.
             */
            phase_inc_u32 =
                (uint32_t)((freq / SAMPLE_RATE_HZ) * 4294967296.0f);

            phase += phase_inc_u32;

            sample = sine_lut[(phase >> 22) &
                              (SINE_LUT_SIZE - 1U)];

            if (cfg->waveform == WAVE_BARKER13)
                sample *= (float)BarkerSign(n, N);

            /* Apply the window selected by the adaptive decision layer. */
            window = WindowFunction(cfg->window, n, N);

            sample *= window;
            sample *= cfg->amplitude;
        }

        {
            float dac_value =
                DAC_MID + sample * DAC_PEAK;

            if (dac_value < 0.0f)
                dac_value = 0.0f;

            if (dac_value > DAC_MAX)
                dac_value = DAC_MAX;

            buffer[n] = (uint16_t)dac_value;
        }
    }
}

/* Compatibility function: fills both ping-pong halves. */
static void GenerateWaveform(void)
{
    GenerateWaveformHalf(&dac_buffer[0], &sonar);
    GenerateWaveformHalf(&dac_buffer[DAC_HALF_SIZE], &sonar);
}

/*
 * Apply the new adaptive configuration only before generating a fresh
 * frame. The currently playing DMA half is never modified.
 */
static void CommitPendingSonarConfigAtFrameBoundary(void)
{
    if (pending_config_valid != 0U)
    {
        sonar = pending_sonar_config;
        pending_config_valid = 0U;
    }
}

/*
 * DMA callbacks MUST remain short.
 * They only mark the finished half as available. Waveform generation is
 * performed from the main loop, preventing a long ISR from interrupting
 * the DAC/DMA timing.
 */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    if (hdac->Instance == DAC)
        dac_half_free[0] = 1U;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    if (hdac->Instance == DAC)
        dac_half_free[1] = 1U;
}

static void ServiceWaveformBuffer(void)
{
    if (dac_half_free[0] != 0U)
    {
        dac_half_free[0] = 0U;

        CommitPendingSonarConfigAtFrameBoundary();
        GenerateWaveformHalf(&dac_buffer[0], &sonar);
    }

    if (dac_half_free[1] != 0U)
    {
        dac_half_free[1] = 0U;

        CommitPendingSonarConfigAtFrameBoundary();
        GenerateWaveformHalf(&dac_buffer[DAC_HALF_SIZE], &sonar);
    }
}

/* ============================================================================
 * WAVEFORM LED INDICATOR
 *
 * STM32F407G-DISC1 onboard LEDs:
 *   LD3 = Orange -> CW
 *   LD4 = Green  -> LFM
 *   LD5 = Red    -> Geometric
 *   LD6 = Blue   -> Barker-13
 *
 * Only one waveform LED is ON at a time.
 * ========================================================================== */

static void UpdateWaveformLEDs(void)
{
    /* Turn all waveform LEDs OFF first. */
    HAL_GPIO_WritePin(WAVEFORM_LED_PORT,
                      LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin,
                      GPIO_PIN_RESET);

    switch (sonar.waveform)
    {
        case WAVE_CW:
            HAL_GPIO_WritePin(WAVEFORM_LED_PORT, LD3_Pin, GPIO_PIN_SET);
            break;

        case WAVE_LFM:
            HAL_GPIO_WritePin(WAVEFORM_LED_PORT, LD4_Pin, GPIO_PIN_SET);
            break;

        case WAVE_GEOMETRIC:
            HAL_GPIO_WritePin(WAVEFORM_LED_PORT, LD5_Pin, GPIO_PIN_SET);
            break;

        case WAVE_BARKER13:
            HAL_GPIO_WritePin(WAVEFORM_LED_PORT, LD6_Pin, GPIO_PIN_SET);
            break;

        default:
            /* Keep all waveform LEDs OFF for an invalid state. */
            break;
    }
}

/* ============================================================================
 * START DAC + TIMER
 * ========================================================================== */

static void StartSonarOutput(void)
{
    if (HAL_DAC_Start_DMA(&hdac,
                          DAC_CHANNEL_1,
                          (uint32_t *)dac_buffer,
                          DAC_BUFFER_SIZE,
                          DAC_ALIGN_12B_R) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * Start TIM6 after DAC DMA is armed.
     */
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 * SYSTEM CLOCK
 * ========================================================================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState =
        RCC_HSE_ON;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV4;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 * ADC1
 * ========================================================================== */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;

    hadc1.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV4;

    hadc1.Init.Resolution =
        ADC_RESOLUTION_12B;

    hadc1.Init.ScanConvMode =
        ENABLE;

    hadc1.Init.ContinuousConvMode =
        ENABLE;

    hadc1.Init.DiscontinuousConvMode =
        DISABLE;

    hadc1.Init.ExternalTrigConvEdge =
        ADC_EXTERNALTRIGCONVEDGE_NONE;

    hadc1.Init.ExternalTrigConv =
        ADC_SOFTWARE_START;

    hadc1.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;

    hadc1.Init.NbrOfConversion =
        ADC_CHANNEL_COUNT;

    hadc1.Init.DMAContinuousRequests =
        ENABLE;

    hadc1.Init.EOCSelection =
        ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        Error_Handler();

    /* PA1 = Temperature */
    sConfig.Channel =
        ADC_CHANNEL_1;

    sConfig.Rank =
        1;

    sConfig.SamplingTime =
        ADC_SAMPLETIME_480CYCLES;

    if (HAL_ADC_ConfigChannel(
            &hadc1,
            &sConfig) != HAL_OK)
        Error_Handler();

    /* PA2 = Depth */
    sConfig.Channel =
        ADC_CHANNEL_2;

    sConfig.Rank =
        2;

    if (HAL_ADC_ConfigChannel(
            &hadc1,
            &sConfig) != HAL_OK)
        Error_Handler();

    /* PA3 = Turbidity */
    sConfig.Channel =
        ADC_CHANNEL_3;

    sConfig.Rank =
        3;

    if (HAL_ADC_ConfigChannel(
            &hadc1,
            &sConfig) != HAL_OK)
        Error_Handler();

    /* PB0 = Salinity */
    sConfig.Channel =
        ADC_CHANNEL_8;

    sConfig.Rank =
        4;

    if (HAL_ADC_ConfigChannel(
            &hadc1,
            &sConfig) != HAL_OK)
        Error_Handler();

    /* PB1 = Battery */
    sConfig.Channel =
        ADC_CHANNEL_9;

    sConfig.Rank =
        5;

    if (HAL_ADC_ConfigChannel(
            &hadc1,
            &sConfig) != HAL_OK)
        Error_Handler();
}

/* ============================================================================
 * DAC1
 * ========================================================================== */

static void MX_DAC_Init(void)
{
    DAC_ChannelConfTypeDef sConfig = {0};

    hdac.Instance = DAC;

    if (HAL_DAC_Init(&hdac) != HAL_OK)
        Error_Handler();

    sConfig.DAC_Trigger =
        DAC_TRIGGER_T6_TRGO;

    sConfig.DAC_OutputBuffer =
        DAC_OUTPUTBUFFER_ENABLE;

    if (HAL_DAC_ConfigChannel(
            &hdac,
            &sConfig,
            DAC_CHANNEL_1) != HAL_OK)
        Error_Handler();
}

/* ============================================================================
 * TIM6
 *
 * TIM6 timer clock = 84 MHz
 *
 * Fs = 84 MHz / (PSC+1) / (ARR+1)
 *    = 84 MHz / 1 / 42
 *    = 2 MHz
 * ========================================================================== */

static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim6.Instance = TIM6;

    htim6.Init.Prescaler =
        0;

    htim6.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    htim6.Init.Period =
        41;

    htim6.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_UPDATE;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim6,
            &sMasterConfig) != HAL_OK)
        Error_Handler();
}

/* ============================================================================
 * I2C1
 * ========================================================================== */

static void MX_I2C1_Init(void)
{
    hi2c1.Instance =
        I2C1;

    hi2c1.Init.ClockSpeed =
        100000;

    hi2c1.Init.DutyCycle =
        I2C_DUTYCYCLE_2;

    hi2c1.Init.OwnAddress1 =
        0;

    hi2c1.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;

    hi2c1.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;

    hi2c1.Init.OwnAddress2 =
        0;

    hi2c1.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;

    hi2c1.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();
}

/* ============================================================================
 * SPI1
 * ========================================================================== */

static void MX_SPI1_Init(void)
{
    hspi1.Instance =
        SPI1;

    hspi1.Init.Mode =
        SPI_MODE_MASTER;

    hspi1.Init.Direction =
        SPI_DIRECTION_2LINES;

    hspi1.Init.DataSize =
        SPI_DATASIZE_8BIT;

    hspi1.Init.CLKPolarity =
        SPI_POLARITY_LOW;

    hspi1.Init.CLKPhase =
        SPI_PHASE_1EDGE;

    hspi1.Init.NSS =
        SPI_NSS_SOFT;

    hspi1.Init.BaudRatePrescaler =
        SPI_BAUDRATEPRESCALER_2;

    hspi1.Init.FirstBit =
        SPI_FIRSTBIT_MSB;

    hspi1.Init.TIMode =
        SPI_TIMODE_DISABLE;

    hspi1.Init.CRCCalculation =
        SPI_CRCCALCULATION_DISABLE;

    hspi1.Init.CRCPolynomial =
        10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        Error_Handler();
}

/* ============================================================================
 * DMA
 *
 * CubeMX normally generates the exact stream/channel assignments in
 * stm32f4xx_hal_msp.c. Keep those assignments generated by CubeMX.
 * ========================================================================== */

static void MX_DMA_Init(void)
{
    /* DMA controller clocks: ADC1 = DMA2, DAC1 = DMA1 */
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* ADC1 DMA interrupt */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    /* DAC1 DMA interrupt */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/* ============================================================================
 * GPIO
 *
 * Keep the board GPIO generated by CubeMX.
 * ========================================================================== */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    HAL_GPIO_WritePin(
        CS_I2C_SPI_GPIO_Port,
        CS_I2C_SPI_Pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        OTG_FS_PowerSwitchOn_GPIO_Port,
        OTG_FS_PowerSwitchOn_Pin,
        GPIO_PIN_SET);

    HAL_GPIO_WritePin(
        GPIOD,
        LD4_Pin |
        LD3_Pin |
        LD5_Pin |
        LD6_Pin |
        Audio_RST_Pin,
        GPIO_PIN_RESET);

    GPIO_InitStruct.Pin =
        CS_I2C_SPI_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        CS_I2C_SPI_GPIO_Port,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        OTG_FS_PowerSwitchOn_Pin;

    HAL_GPIO_Init(
        OTG_FS_PowerSwitchOn_GPIO_Port,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        PDM_OUT_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_PP;

    GPIO_InitStruct.Alternate =
        GPIO_AF5_SPI2;

    HAL_GPIO_Init(
        PDM_OUT_GPIO_Port,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        B1_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_EVT_RISING;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        B1_GPIO_Port,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        BOOT1_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        BOOT1_GPIO_Port,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        CLK_IN_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_PP;

    GPIO_InitStruct.Alternate =
        GPIO_AF5_SPI2;

    HAL_GPIO_Init(
        CLK_IN_GPIO_Port,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        LD4_Pin |
        LD3_Pin |
        LD5_Pin |
        LD6_Pin |
        Audio_RST_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOD,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        I2S3_SCK_Pin |
        I2S3_SD_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_PP;

    GPIO_InitStruct.Alternate =
        GPIO_AF6_SPI3;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        OTG_FS_OverCurrent_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        OTG_FS_OverCurrent_GPIO_Port,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        MEMS_INT2_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_EVT_RISING;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        MEMS_INT2_GPIO_Port,
        &GPIO_InitStruct);
}

/* ============================================================================
 * ERROR HANDLER
 * ========================================================================== */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}

#endif
