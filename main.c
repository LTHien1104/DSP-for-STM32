/* USER CODE BEGIN Header */
/*
 * NUCLEO-F401RE + Pulse Sensor (3.3V)
 * ADC1 Ch0 (PA0) sample @500 Hz by TIM2 TRGO + DMA circular
 * Beat detection with hysteresis + refractory
 * LED LD2 (PA5) toggles on each beat
 * SWV Timeline: g_adc_smoothed (uint16), g_bpm (int), g_bpm_fft (int)
 * SWV ITM Console: printf on ITM port 0
 */
/* USER CODE END Header */
#include "main.h"
#include <stdint.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "core_cm4.h"   // ITM_SendChar()
#include <stdio.h>
#include <math.h>
#include "arm_math.h"   // CMSIS-DSP (RFFT, vector ops)
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FS_HZ               500U    // sample rate ADC
#define ADC_BUF_LEN         128     // DMA circular buffer length

/* Beat detection (12-bit ADC) */
#define THRESH_HIGH         2200
#define THRESH_LOW          1800
#define REFRACT_MS          320     // tránh >200 BPM (≈300 ms)

/* Giới hạn BPM hợp lý + smoothing */
#define BPM_MIN             40
#define BPM_MAX             180

/* ---------- FFT params (Fourier) ---------- */
#define DS_FACTOR      5U                 // 500 Hz -> 100 Hz
#define FS_FFT         (FS_HZ/DS_FACTOR)  // 100 Hz
#define FFT_N          1024U              // 10.24 s cửa sổ
#define HR_FMIN_HZ     0.8f               // 48 BPM
#define HR_FMAX_HZ     3.0f               // 180 BPM
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Biến vẽ & kết quả */
volatile uint16_t g_adc_smoothed = 0;   // time-domain signal (để vẽ)
volatile int      g_bpm          = 0;   // BPM (thời gian)
volatile int      g_bpm_fft      = 0;   // BPM ước lượng từ FFT (phổ)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static uint16_t adcBuf[ADC_BUF_LEN];
static volatile uint8_t  dmaHalf = 0, dmaFull = 0;
static volatile uint32_t sampleCounter = 0;

/* --- FFT buffers/state --- */
static float32_t fft_in[FFT_N];          // khung mẫu (sau downsample)
static float32_t fft_win[FFT_N];         // Hann window
static float32_t fft_mag[FFT_N/2];       // |X[k]|
static arm_rfft_fast_instance_f32 rfft;  // RFFT handle
static uint16_t ds_count = 0;
static uint32_t fft_idx  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void process_block(uint16_t *blk, size_t n);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* --- Retarget printf to SWV ITM port 0 --- */
int __io_putchar(int ch) { ITM_SendChar((uint32_t)ch); return ch; }
int _write(int file, char *ptr, int len) {
  (void)file; for (int i=0;i<len;i++) ITM_SendChar((uint32_t)ptr[i]); return len;
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  /* Start peripherals */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, ADC_BUF_LEN);
  HAL_TIM_Base_Start(&htim2);

  /* SWV console info */
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\n[SWV] Pulse demo started. fs=%lu Hz\r\n", (unsigned long)FS_HZ);
  printf("[SWV] Refract=%d ms, BPM range %d..%d\r\n", REFRACT_MS, BPM_MIN, BPM_MAX);

  /* ---------- Init RFFT & window ---------- */
  arm_rfft_fast_init_f32(&rfft, FFT_N);
  for (uint32_t n=0; n<FFT_N; n++) {
    fft_win[n] = 0.5f * (1.0f - arm_cos_f32(2.0f*PI * (float32_t)n / (float32_t)(FFT_N-1)));
  }
  printf("[SWV] FFT ready: N=%u, fs_fft=%lu Hz, df=%.3f Hz\r\n",
         FFT_N, (unsigned long)FS_FFT, (float)FS_FFT/(float)FFT_N);

  while (1)
  {
    if (dmaHalf) { dmaHalf = 0; process_block(&adcBuf[0], ADC_BUF_LEN/2); }
    if (dmaFull) { dmaFull = 0; process_block(&adcBuf[ADC_BUF_LEN/2], ADC_BUF_LEN/2); }
  }
}

/* ------------------------- Callbacks -------------------------- */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){ if (hadc->Instance==ADC1) dmaHalf=1; }
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){ if (hadc->Instance==ADC1) dmaFull=1; }

/* ------------------------- Processing ------------------------- */
static void process_block(uint16_t *blk, size_t n)
{
  /* moving-average cho hiển thị mượt */
  #define MA_WIN 7
  static uint16_t maBuf[MA_WIN];
  static uint32_t maSum=0;
  static uint8_t  maIdx=0, maCnt=0;

  static uint8_t  above=0;
  static uint32_t lastBeat=0;

  /* EMA cho BPM */
  static float bpm_ema = 0.0f;

  static uint32_t lastLog = 0;

  for (size_t i=0;i<n;i++)
  {
    uint16_t x = blk[i];

    /* moving average */
    maSum -= maBuf[maIdx];
    maBuf[maIdx] = x;
    maSum += maBuf[maIdx];
    maIdx = (maIdx + 1) % MA_WIN;
    if (maCnt < MA_WIN) maCnt++;
    uint16_t x_sm = (uint16_t)(maSum / maCnt);
    g_adc_smoothed = x_sm;
    sampleCounter++;

    /* ---- Beat detection (time-domain) ---- */
    uint32_t now = HAL_GetTick();
    if (!above && x_sm >= THRESH_HIGH && (now - lastBeat) > REFRACT_MS)
    {
      above = 1;
      if (lastBeat != 0) {
        uint32_t ibi = now - lastBeat;        // ms
        if (ibi > 0) {
          int bpm_inst = (int)(60000U / ibi);
          if (bpm_inst >= BPM_MIN && bpm_inst <= BPM_MAX) {
            if (bpm_ema == 0.0f) bpm_ema = (float)bpm_inst;
            bpm_ema = 0.7f*bpm_ema + 0.3f*(float)bpm_inst;
            g_bpm = (int)(bpm_ema + 0.5f);
            printf("[SWV] beat  BPM=%d  IBI=%lums  ADC=%u  t=%lums\r\n",
                   g_bpm, (unsigned long)ibi, (unsigned)x_sm, (unsigned long)now);
          }
        }
      } else {
        printf("[SWV] first beat detected\r\n");
      }
      lastBeat = now;
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
    else if (above && x_sm <= THRESH_LOW) { above = 0; }

    /* ---- Downsample và gom khung cho FFT ---- */
    if (++ds_count >= DS_FACTOR) {
      ds_count = 0;
      fft_in[fft_idx++] = (float32_t)x_sm;   // hoặc dùng x raw
      if (fft_idx >= FFT_N) {
        fft_idx = 0;

        /* 1) Bỏ DC (trừ trung bình) */
        float32_t mean;
        arm_mean_f32(fft_in, FFT_N, &mean);
        arm_offset_f32(fft_in, -mean, fft_in, FFT_N);

        /* 2) Áp cửa sổ Hann */
        arm_mult_f32(fft_in, fft_win, fft_in, FFT_N);

        /* 3) RFFT */
        arm_rfft_fast_f32(&rfft, fft_in, fft_in, 0);

        /* 4) Độ lớn phổ cho N/2 bin dương */
        arm_cmplx_mag_f32(fft_in, fft_mag, FFT_N/2);

        /* 5) Tìm đỉnh 0.8–3.0 Hz (48–180 BPM) */
        uint32_t k_min = (uint32_t)ceilf(HR_FMIN_HZ * (float)FFT_N / (float)FS_FFT);
        uint32_t k_max = (uint32_t)floorf(HR_FMAX_HZ * (float)FFT_N / (float)FS_FFT);
        if (k_min < 1) k_min = 1;
        if (k_max > (FFT_N/2 - 2)) k_max = FFT_N/2 - 2; // để còn k-1,k+1

        float32_t maxv = 0.0f; uint32_t k_star = k_min;
        for (uint32_t k=k_min; k<=k_max; k++) {
          if (fft_mag[k] > maxv) { maxv = fft_mag[k]; k_star = k; }
        }

        /* Nội suy parabol quanh đỉnh để cải thiện phân giải */
        float32_t y0=fft_mag[k_star-1], y1=fft_mag[k_star], y2=fft_mag[k_star+1];
        float32_t denom = 2.0f*(y0 - 2.0f*y1 + y2);
        float32_t delta = (denom != 0.0f) ? ((y0 - y2)/denom) : 0.0f; // -0.5..0.5
        float32_t k_hat = (float32_t)k_star + delta;

        float32_t f_hat = k_hat * (float32_t)FS_FFT / (float32_t)FFT_N;
        int bpm_fft = (int)(f_hat * 60.0f + 0.5f);

        /* Giới hạn hợp lý trước khi xuất/hiển thị */
        if (bpm_fft >= BPM_MIN && bpm_fft <= BPM_MAX) {
          g_bpm_fft = bpm_fft;
          printf("[SWV] FFT BPM=%d  (k=%.2f, f=%.3f Hz)\r\n",
                 g_bpm_fft, (double)k_hat, (double)f_hat);
        } else {
          // printf("[SWV] FFT out-of-range: %d BPM\r\n", bpm_fft);
        }
      }
    }

    /* log định kỳ */
    if (now - lastLog >= 500) {
      lastLog = now;
      printf("[SWV] adc=%u  bpm=%d  bpm_fft=%d\r\n",
             (unsigned)g_adc_smoothed, g_bpm, g_bpm_fft);
    }
  }
}

/* -------------------- Peripheral init funcs ------------------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  htim2.Instance = TIM2;
  htim2.Init.Prescaler         = 8399;                 // 84MHz -> 10kHz
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 19;                   // 10kHz/(19+1)=500Hz
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE; // TRGO on update
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK){
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance                      = ADC1;
  hadc1.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution               = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode             = DISABLE;
  hadc1.Init.ContinuousConvMode       = DISABLE;
  hadc1.Init.DiscontinuousConvMode    = DISABLE;
  hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv         = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion          = 1;
  hadc1.Init.DMAContinuousRequests    = ENABLE;
  hadc1.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  sConfig.Channel      = ADC_CHANNEL_0;          // PA0
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* ADC1 uses DMA2 Stream0 Channel0 on F401RE */
  hdma_adc1.Instance = DMA2_Stream0;
  hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode                = DMA_CIRCULAR;
  hdma_adc1.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) { Error_Handler(); }

  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  /* DMA interrupt init (handler ở stm32f4xx_it.c) */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* LD2: PA5 */
  GPIO_InitStruct.Pin   = GPIO_PIN_5;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ------------------------- Error Handler ---------------------- */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
