/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifndef APP_ENABLE_AHT10
#define APP_ENABLE_AHT10 1
#endif
#include <stdio.h>
#include "sgp30.h"
#if APP_ENABLE_AHT10
#include "aht10.h"
#endif
#ifndef APP_ENABLE_OLED
#define APP_ENABLE_OLED 1
#endif
#if APP_ENABLE_OLED
#include "ssd1306.h"
#endif

/* ESP8266-01S on USART3 (PB10 TX / PB11 RX); USART1 (PA9/PA10) = debug printf. */
#ifndef APP_ENABLE_ESP8266
#define APP_ENABLE_ESP8266 1
#endif
#ifndef APP_DEBUG_UART
#define APP_DEBUG_UART 0
#endif
#include "esp8266_http.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* APP_ENABLE_AHT10: set in USER CODE Includes (0 = SGP30-only build). */

/* When no sensors: retry/watch intervals (ms) — shorter spam on UART. */
#define APP_RUN_RETRY_FIRST_MS 15000U
#define APP_RUN_RETRY_PERIOD_MS 30000U
#define APP_I2C_WATCH_FIRST_MS 30000U
#define APP_I2C_WATCH_PERIOD_MS 60000U
#define APP_UPLOAD_WINDOW_MS 60000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t g_aht10_online = 0U;
static uint8_t g_sgp30_online = 0U;
static uint8_t g_aht10_ready = 0U;
static uint8_t g_sgp30_ready = 0U;
#if APP_ENABLE_OLED
static uint8_t g_oled_ok = 0U;
#endif
#if APP_ENABLE_ESP8266
static uint8_t g_esp8266_ok = 0U;
#endif
typedef struct
{
  uint32_t window_start_ms;
  uint16_t samples;
  float t_last;
  float rh_last;
  uint16_t co2_last;
  uint16_t tvoc_last;
  float t_min;
  float t_max;
  float t_sum;
  float rh_min;
  float rh_max;
  float rh_sum;
  uint32_t co2_sum;
  uint32_t tvoc_sum;
  uint16_t co2_min;
  uint16_t co2_max;
  uint16_t tvoc_min;
  uint16_t tvoc_max;
} App_MinuteStats;

#if APP_ENABLE_ESP8266
static App_MinuteStats g_upload_stats = {0};
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
#if APP_DEBUG_UART
  uint8_t c = (uint8_t)ch;
  (void)HAL_UART_Transmit(&huart1, &c, 1U, 3U);
#else
  (void)ch;
#endif
  return ch;
}

#if APP_ENABLE_ESP8266
static void App_MinuteStatsReset(App_MinuteStats *s, uint32_t now_ms)
{
  if (s == NULL)
  {
    return;
  }

  s->window_start_ms = now_ms;
  s->samples = 0U;
  s->t_last = 0.0f;
  s->rh_last = 0.0f;
  s->co2_last = 0U;
  s->tvoc_last = 0U;
  s->t_min = 0.0f;
  s->t_max = 0.0f;
  s->t_sum = 0.0f;
  s->rh_min = 0.0f;
  s->rh_max = 0.0f;
  s->rh_sum = 0.0f;
  s->co2_sum = 0U;
  s->tvoc_sum = 0U;
  s->co2_min = 0U;
  s->co2_max = 0U;
  s->tvoc_min = 0U;
  s->tvoc_max = 0U;
}

static void App_MinuteStatsAdd(App_MinuteStats *s, uint32_t now_ms, float t, float rh, uint16_t co2, uint16_t tvoc)
{
  if (s == NULL)
  {
    return;
  }

  if (s->window_start_ms == 0U)
  {
    App_MinuteStatsReset(s, now_ms);
  }

  s->t_last = t;
  s->rh_last = rh;
  s->co2_last = co2;
  s->tvoc_last = tvoc;

  if (s->samples == 0U)
  {
    s->t_min = t;
    s->t_max = t;
    s->rh_min = rh;
    s->rh_max = rh;
    s->co2_min = co2;
    s->co2_max = co2;
    s->tvoc_min = tvoc;
    s->tvoc_max = tvoc;
  }
  else
  {
    if (t < s->t_min) { s->t_min = t; }
    if (t > s->t_max) { s->t_max = t; }
    if (rh < s->rh_min) { s->rh_min = rh; }
    if (rh > s->rh_max) { s->rh_max = rh; }
    if (co2 < s->co2_min) { s->co2_min = co2; }
    if (co2 > s->co2_max) { s->co2_max = co2; }
    if (tvoc < s->tvoc_min) { s->tvoc_min = tvoc; }
    if (tvoc > s->tvoc_max) { s->tvoc_max = tvoc; }
  }

  s->t_sum += t;
  s->rh_sum += rh;
  s->co2_sum += (uint32_t)co2;
  s->tvoc_sum += (uint32_t)tvoc;
  s->samples++;
}

static void App_MinuteStatsToUpload(const App_MinuteStats *s, Esp8266_UploadStats *out)
{
  if ((s == NULL) || (out == NULL) || (s->samples == 0U))
  {
    return;
  }

  out->t = s->t_last;
  out->rh = s->rh_last;
  out->co2 = s->co2_last;
  out->tvoc = s->tvoc_last;
  out->t_min = s->t_min;
  out->t_max = s->t_max;
  out->t_avg = s->t_sum / (float)s->samples;
  out->rh_min = s->rh_min;
  out->rh_max = s->rh_max;
  out->rh_avg = s->rh_sum / (float)s->samples;
  out->co2_min = s->co2_min;
  out->co2_max = s->co2_max;
  out->co2_avg = (uint16_t)((s->co2_sum + (uint32_t)(s->samples / 2U)) / (uint32_t)s->samples);
  out->tvoc_min = s->tvoc_min;
  out->tvoc_max = s->tvoc_max;
  out->tvoc_avg = (uint16_t)((s->tvoc_sum + (uint32_t)(s->samples / 2U)) / (uint32_t)s->samples);
  out->samples = s->samples;
}
#endif

/* Release stuck I2C bus: HAL returns HAL_BUSY when SR2 BUSY never clears. */
static void I2C1_UnstickBus(void)
{
  uint32_t i;
  GPIO_InitTypeDef g = {0};

  (void)HAL_I2C_DeInit(&hi2c1);

  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(2);
  __HAL_RCC_I2C1_RELEASE_RESET();

  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(1);

  for (i = 0U; i < 9U; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    for (volatile uint32_t w = 0U; w < 2000U; w++) { }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    for (volatile uint32_t w = 0U; w < 2000U; w++) { }
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  HAL_Delay(1);

  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

  MX_I2C1_Init();
  hi2c1.Lock = HAL_UNLOCKED;
}

/* F1 HAL can return HAL_ERROR from Master_Transmit without __HAL_UNLOCK / READY — next call is HAL_BUSY. */
static void I2C1_ResetAfterHalStall(void)
{
  (void)HAL_I2C_DeInit(&hi2c1);
  MX_I2C1_Init();
  /* HAL Master_* can leave Lock stuck; DeInit unlocks, this is a safety net. */
  hi2c1.Lock = HAL_UNLOCKED;
}

/* HAL_BUSY on Master_* often means I2C BUSY bit stuck; unstick bit-bangs SCL. Other errors: soft re-init. */
static void I2C1_RecoverFromI2cFailure(HAL_StatusTypeDef st)
{
  if (st == HAL_BUSY)
  {
    I2C1_UnstickBus();
  }
  else
  {
    I2C1_ResetAfterHalStall();
  }
}

static void App_PrintLastResetCause(void)
{
  uint32_t csr = RCC->CSR;

  printf("[INIT] RCC_CSR=0x%08lx", (unsigned long)csr);
  if ((csr & RCC_CSR_LPWRRSTF) != 0U)
  {
    printf(" LPWRRST");
  }
  if ((csr & RCC_CSR_WWDGRSTF) != 0U)
  {
    printf(" WWDG");
  }
  if ((csr & RCC_CSR_IWDGRSTF) != 0U)
  {
    printf(" IWDG");
  }
  if ((csr & RCC_CSR_SFTRSTF) != 0U)
  {
    printf(" SFTRST");
  }
  if ((csr & RCC_CSR_PORRSTF) != 0U)
  {
    printf(" POR");
  }
  if ((csr & RCC_CSR_PINRSTF) != 0U)
  {
    printf(" PIN");
  }
  if ((csr & (RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_IWDGRSTF |
              RCC_CSR_SFTRSTF | RCC_CSR_PORRSTF | RCC_CSR_PINRSTF)) == 0U)
  {
    printf(" (no reset flags)");
  }
  printf(" (dup boot banner => often 2nd reset here)\r\n");
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

static const char *App_HalStatusStr(HAL_StatusTypeDef s)
{
  if (s == HAL_OK)
  {
    return "HAL_OK";
  }
  if (s == HAL_ERROR)
  {
    return "HAL_ERROR";
  }
  if (s == HAL_BUSY)
  {
    return "HAL_BUSY";
  }
  if (s == HAL_TIMEOUT)
  {
    return "HAL_TIMEOUT";
  }
  return "HAL_?";
}

static void App_PrintI2cDecodeRaw(uint32_t state, uint32_t lock, uint32_t err)
{
  printf(" i2c_state=0x%lx", (unsigned long)state);
  if (state == HAL_I2C_STATE_READY)
  {
    printf("[READY]");
  }
  else if (state == HAL_I2C_STATE_RESET)
  {
    printf("[RESET]");
  }
  else if (state == HAL_I2C_STATE_ERROR)
  {
    printf("[ERROR]");
  }
  else
  {
    printf("[not READY]");
  }

  printf(" lock=%lu%s", (unsigned long)lock, (lock == HAL_LOCKED) ? "[LOCKED]" : "");

  printf(" err=0x%lx:", (unsigned long)err);
  if (err == 0U)
  {
    printf("none");
  }
  else
  {
    if ((err & HAL_I2C_ERROR_TIMEOUT) != 0U)
    {
      printf(" TIMEOUT");
    }
    if ((err & HAL_I2C_ERROR_AF) != 0U)
    {
      printf(" AF");
    }
    if ((err & HAL_I2C_ERROR_BERR) != 0U)
    {
      printf(" BERR");
    }
    if ((err & HAL_I2C_ERROR_ARLO) != 0U)
    {
      printf(" ARLO");
    }
    if ((err & HAL_I2C_ERROR_OVR) != 0U)
    {
      printf(" OVR");
    }
  }
  printf("\r\n");
}

static void App_PrintI2cDecode(const I2C_HandleTypeDef *hi2c)
{
  App_PrintI2cDecodeRaw((uint32_t)hi2c->State, (uint32_t)hi2c->Lock, hi2c->ErrorCode);
}

#if APP_ENABLE_OLED
static void App_OledShowStatus(const char *line1, const char *line2, const char *line3)
{
  SSD1306_DisplayOn();
  SSD1306_Clear();
  SSD1306_DrawStr5x7(0, 0, "F103 AIR");
  SSD1306_DrawStr5x7(0, 2, line1);
  SSD1306_DrawStr5x7(0, 4, line2);
  SSD1306_DrawStr5x7(0, 6, line3);
  (void)SSD1306_Update(&hi2c1);
}

/* aht_disp: 0=no/off, 1=OK show T/RH, 2=on bus but read error */
static void App_OledRedraw(float Tc, float Rh, uint8_t aht_disp, uint16_t eco2, uint16_t tvoc, uint8_t sgp_disp)
{
  char ln[22];

  SSD1306_DisplayOn();
  SSD1306_Clear();
  SSD1306_DrawStr5x7(0, 0, "F103 AIR");
  if (aht_disp == 1U)
  {
    (void)snprintf(ln, sizeof(ln), "T %4.1fC", Tc);
    SSD1306_DrawStr5x7(0, 1, ln);
    (void)snprintf(ln, sizeof(ln), "RH %4.1f%%", Rh);
    SSD1306_DrawStr5x7(0, 2, ln);
  }
  else if (aht_disp == 2U)
  {
    SSD1306_DrawStr5x7(0, 1, "AHT ERR");
    SSD1306_DrawStr5x7(0, 2, "--------");
  }
  else
  {
    SSD1306_DrawStr5x7(0, 1, "AHT ----");
    SSD1306_DrawStr5x7(0, 2, "--------");
  }

  if (sgp_disp == 1U)
  {
    (void)snprintf(ln, sizeof(ln), "CO2 %4u", (unsigned)eco2);
    SSD1306_DrawStr5x7(0, 4, ln);
    SSD1306_DrawStr5x7(78, 4, "ppm");
    (void)snprintf(ln, sizeof(ln), "TVOC %4u", (unsigned)tvoc);
    SSD1306_DrawStr5x7(0, 5, ln);
    SSD1306_DrawStr5x7(90, 5, "ppb");
  }
  else if (sgp_disp == 2U)
  {
    SSD1306_DrawStr5x7(0, 4, "SGP ERR");
    SSD1306_DrawStr5x7(0, 5, "--------");
  }
  else
  {
    SSD1306_DrawStr5x7(0, 4, "SGP ----");
    SSD1306_DrawStr5x7(0, 5, "--------");
  }

  (void)SSD1306_Update(&hi2c1);
}
#endif /* APP_ENABLE_OLED */

#if APP_ENABLE_AHT10
/* Quick ACK sweep: if nothing responds, no sensor data can ever arrive over I2C. */
static void I2C1_PrintBusProbe(void)
{
  uint32_t a;
  unsigned found = 0U;
  uint8_t sgp_seen_58 = 0U;
  uint8_t sgp_seen_59 = 0U;
  g_aht10_online = 0U;
  g_sgp30_online = 0U;

  printf("[INIT] I2C1 scan (AHT10=0x38; SGP30=0x58, alt 0x59 e.g. SGP40/some clones):\r\n");
  for (a = 1U; a < 127U; a++)
  {
    /* 5ms was marginal on breadboards; 15ms reduces false "no device". */
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1U), 2U, 15U) == HAL_OK)
    {
      printf("[INIT]   0x%02lx ACK\r\n", (unsigned long)a);
      if (a == 0x38U)
      {
        g_aht10_online = 1U;
      }
      if (a == 0x58U)
      {
        sgp_seen_58 = 1U;
      }
      if (a == 0x59U)
      {
        sgp_seen_59 = 1U;
      }
      found++;
    }
  }
  /* Prefer 0x58 (datasheet SGP30); else accept 0x59 and point driver there. */
  if (sgp_seen_58 != 0U)
  {
    g_sgp30_online = 1U;
    SGP30_SetAddr7(0x58U);
  }
  else if (sgp_seen_59 != 0U)
  {
    g_sgp30_online = 1U;
    SGP30_SetAddr7(0x59U);
    printf("[INIT] Using I2C addr 0x59 for VOC driver (SGP30 is 0x58). If reads fail, chip may be SGP40/41 (different protocol).\r\n");
  }
  if (found == 0U)
  {
    printf("[INIT]   no ACK -> bus sees no slave; fix wiring GND 3V3 PB8=SCL PB9=SDA pull-ups\r\n");
  }
  I2C1_ResetAfterHalStall();
}
#endif /* APP_ENABLE_AHT10 */

#if APP_ENABLE_AHT10
static HAL_StatusTypeDef App_TryInitAht10(const char *tag)
{
  HAL_StatusTypeDef st;
  unsigned prep;
  HAL_StatusTypeDef ready_st = HAL_ERROR;

  /* ASAIR modules may need tens of ms after 3V3 / after heavy I2C traffic from scan + SGP30 probe. */
  HAL_Delay(80);

  for (prep = 0U; prep < 4U; prep++)
  {
    if (prep > 0U)
    {
      HAL_Delay(50);
    }
    ready_st = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(0x38U << 1U), 10U, 40U);
    if (ready_st == HAL_OK)
    {
      printf("[%s] AHT10 probe[%u]: IsDeviceReady(0x38)=HAL_OK |", tag, (unsigned)(prep + 1U));
      App_PrintI2cDecode(&hi2c1);
      break;
    }
    printf("[%s] AHT10 probe[%u]: IsDeviceReady(0x38)=%s |", tag, (unsigned)(prep + 1U), App_HalStatusStr(ready_st));
    App_PrintI2cDecode(&hi2c1);
  }
  if (prep >= 4U)
  {
    g_aht10_online = 0U;
    g_aht10_ready = 0U;
    printf("[%s] AHT10 probe summary: last_ready=%s |", tag, App_HalStatusStr(ready_st));
    App_PrintI2cDecode(&hi2c1);
    printf("[%s] AHT10: no ACK at 7-bit 0x38 — same I2C as SGP30: SDA=PB9 SCL=PB8, 3.3V, GND, short wires / 4k7 pull-ups.\r\n", tag);
    return HAL_ERROR;
  }

  g_aht10_online = 1U;
  st = AHT10_Init(&hi2c1);
  if (st == HAL_OK)
  {
    g_aht10_ready = 1U;
    printf("[%s] AHT10 init ok.\r\n", tag);
  }
  else
  {
    g_aht10_ready = 0U;
    printf("[%s] AHT10 init failed: %s |", tag, App_HalStatusStr(st));
    App_PrintI2cDecode(&hi2c1);
    I2C1_RecoverFromI2cFailure(st);
  }

  return st;
}
#endif /* APP_ENABLE_AHT10 */

/* Multi-speed + dual-address + serial-ID verify + optional reset (no external pull-ups needed if MCU pull-up + module OK). */
static HAL_StatusTypeDef App_TryInitSgp30(const char *tag)
{
  static const uint32_t k_speed_hz[] = {25000U, 50000U, 100000U, 10000U};
  static const uint8_t k_addr7[] = {0x58U, 0x59U};
  unsigned si;
  unsigned ai;

  g_sgp30_online = 0U;
  g_sgp30_ready = 0U;

  for (si = 0U; si < (unsigned)(sizeof(k_speed_hz) / sizeof(k_speed_hz[0])); si++)
  {
    I2C1_SetClock(k_speed_hz[si]);
    printf("[%s] SGP30: try I2C %lu Hz + addr 0x58/0x59\r\n", tag, (unsigned long)k_speed_hz[si]);
    HAL_Delay(20);

    for (ai = 0U; ai < (unsigned)(sizeof(k_addr7) / sizeof(k_addr7[0])); ai++)
    {
      uint8_t a7 = k_addr7[ai];
      HAL_StatusTypeDef st;

      SGP30_SetAddr7(a7);
      if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)((uint16_t)a7 << 1U), 8U, 40U) != HAL_OK)
      {
        continue;
      }

      printf("[%s] ACK 7-bit=0x%02x\r\n", tag, (unsigned)a7);

      {
        uint16_t fs = 0U;
        st = SGP30_ReadFeatureSet(&hi2c1, &fs);
        if (st == HAL_OK)
        {
          uint8_t ptype = (uint8_t)((fs & 0xF000U) >> 12);
          printf("[%s] FeatureSet word=0x%04x product_type=%u (datasheet SGP30=>0)\r\n",
                 tag, (unsigned)fs, (unsigned)ptype);
          if (ptype != 0U)
          {
            printf("[%s] WARN: product_type!=0 — chip may not be SGP30; expect CRC mismatch on Sensirion framing.\r\n", tag);
          }
        }
        else
        {
          printf("[%s] FeatureSet read/CRC fail (same class of fault as IAQ if bus marginal or wrong chip).\r\n", tag);
        }
      }

      {
        uint8_t sid[9];
        st = SGP30_ReadSerialId(&hi2c1, sid);
      }
      if (st == HAL_OK)
      {
        printf("[%s] GetSerialId CRC OK\r\n", tag);
      }
      else
      {
        printf("[%s] GetSerialId skip/err (still try iaq_init)\r\n", tag);
      }

      st = SGP30_Init(&hi2c1);
      if (st == HAL_OK)
      {
        g_sgp30_online = 1U;
        g_sgp30_ready = 1U;
        I2C1_SetClock(50000U);
        printf("[%s] SGP30 iaq_init OK @0x%02x (running I2C at 50 kHz)\r\n", tag, (unsigned)a7);
        return HAL_OK;
      }

      printf("[%s] iaq_init fail: %s |", tag, App_HalStatusStr(st));
      App_PrintI2cDecode(&hi2c1);
      I2C1_RecoverFromI2cFailure(st);
    }

    printf("[%s] general-call reset + delay\r\n", tag);
    (void)SGP30_GeneralCallReset(&hi2c1);
    HAL_Delay(100);
  }

  return HAL_ERROR;
}

/* When no sensor driver is active: periodic I2C ACK sweep (no spam every 2 s). */
static void App_I2c1_LogBusStatusIfIdle(uint32_t tick_ms)
{
  static uint32_t next_watch_ms = 0U;
  unsigned found;
  unsigned i;
#if APP_ENABLE_AHT10
  static const uint8_t watch_addr[] = {0x38U, 0x58U, 0x59U};
#else
  static const uint8_t watch_addr[] = {0x58U, 0x59U};
#endif

#if APP_ENABLE_AHT10
  if ((g_aht10_ready != 0U) || (g_sgp30_ready != 0U))
#else
  if (g_sgp30_ready != 0U)
#endif
  {
    return;
  }

  if (next_watch_ms == 0U)
  {
    next_watch_ms = tick_ms + APP_I2C_WATCH_FIRST_MS;
    return;
  }
  if (tick_ms < next_watch_ms)
  {
    return;
  }
  next_watch_ms = tick_ms + APP_I2C_WATCH_PERIOD_MS;

  /* Fast probe only sensor addresses — full 1..126 sweep leaves HAL TIMEOUT flags and blocks ~seconds. */
#if APP_ENABLE_AHT10
  printf("[I2C1] watch: probe 0x38/0x58/0x59 (idle, ~60s period)\r\n");
#else
  printf("[I2C1] watch: probe SGP30 0x58 / alt 0x59 (idle, ~60s period)\r\n");
#endif
  found = 0U;
  for (i = 0U; i < (unsigned)(sizeof(watch_addr) / sizeof(watch_addr[0])); i++)
  {
    uint8_t a = watch_addr[i];
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)((uint16_t)a << 1U), 3U, 20U) == HAL_OK)
    {
      printf("[I2C1]   ACK 0x%02x\r\n", (unsigned)a);
      found++;
    }
  }
  if (found == 0U)
  {
    printf("[I2C1]   no ACK (check PB8=SCL PB9=SDA GND 3V3 sensor power)\r\n");
  }

  I2C1_ResetAfterHalStall();
  printf("[I2C1]   i2c after watch reset:");
  App_PrintI2cDecode(&hi2c1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemCoreClockUpdate();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
#if APP_DEBUG_UART
  {
    static const uint8_t early[] = "\r\n[EARLY] USART1 PA9 TX alive before I2C/OLED/ESP init\r\n";
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)early, sizeof(early) - 1U, 50U);
  }
#endif
  MX_I2C1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  /* Onboard LED is active-low: SET = off, RESET = on. Code only toggles every 2s (not PWM "breathing"). */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  setbuf(stdout, NULL);
  App_PrintLastResetCause();
  /* Brief settle: supply / USB-UART reconnect; reduces split/corrupt first line after debugger reset. */
  HAL_Delay(120);
  printf("\r\n======== F103 app boot ========\r\n");
  printf("[INIT] HAL_Init + SystemClock + MX_GPIO/I2C1/USART1 done (CubeMX).\r\n");
  printf("[INIT] printf -> USART1 115200 8N1, stdout unbuffered.\r\n");

#if APP_ENABLE_AHT10
  printf("[INIT] I2C1 bus recovery (unstick) starting before OLED/sensor init...\r\n");
  I2C1_UnstickBus();
  printf("[INIT] I2C1 bus recovery done,");
  App_PrintI2cDecode(&hi2c1);
#endif

#if APP_ENABLE_OLED
  HAL_Delay(50);
  if (SSD1306_Init(&hi2c1, SSD1306_I2C_ADDR7_DEFAULT) == HAL_OK)
  {
    g_oled_ok = 1U;
    App_OledShowStatus("OLED OK", "INIT", "WAIT SENSOR");
    printf("[INIT] OLED SSD1306/1315 @0x3C (shared I2C1)\r\n");
  }
  else if (SSD1306_Init(&hi2c1, 0x3DU) == HAL_OK)
  {
    g_oled_ok = 1U;
    App_OledShowStatus("OLED OK", "INIT", "WAIT SENSOR");
    printf("[INIT] OLED SSD1306/1315 @0x3D (shared I2C1)\r\n");
  }
  else
  {
    printf("[INIT] OLED init failed (probe 0x3C/0x3D; JMD096 SSD1315)\r\n");
  }
#endif
#if APP_DEBUG_UART
  {
    static const uint8_t alive[] = "APP alive (USART1 raw TX)\r\n";
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)alive, sizeof(alive) - 1U, 50U);
  }
  printf("[INIT] Raw UART line above = USART1 TX OK (bypass printf).\r\n");
#endif

#if APP_ENABLE_AHT10
  I2C1_PrintBusProbe();

  printf("[INIT] AHT10 driver init (probe 0x38, stronger than full scan)...\r\n");
  g_aht10_ready = 0U;
  if (App_TryInitAht10("INIT") != HAL_OK)
  {
    printf("[INIT] AHT10 not on bus yet; main loop will retry.\r\n");
  }
  printf("[INIT] Build: AHT10 + SGP30 | PB8=SCL PB9=SDA (I2C1 remap)\r\n");
#else
  printf("[INIT] Build: SGP30-only | PB8=SCL PB9=SDA (I2C1 remap) internal pull-up enabled\r\n");
  printf("[INIT] First attempt WITHOUT bit-bang unstick (same idea as many demo wiring).\r\n");
#endif

  HAL_Delay(200);
  printf("[INIT] SGP30 redundant bring-up (speed ladder + Serial ID + iaq_init + addr 0x58/0x59)...\r\n");
  g_sgp30_ready = 0U;
  if (App_TryInitSgp30("INIT") != HAL_OK)
  {
    printf("[INIT] SGP30 failed — optional bus unstick + second pass...\r\n");
    I2C1_UnstickBus();
    HAL_Delay(100);
    if (App_TryInitSgp30("INIT-R") != HAL_OK)
    {
      printf("[INIT] SGP30 still missing; main loop will retry.\r\n");
    }
  }

#if APP_ENABLE_AHT10
  printf("[HINT] If you see TIMEOUT on I2C: PB8=SCL PB9=SDA, common GND, 3.3V, 4.7k pull-ups, sensor power.\r\n");
#else
  printf("[HINT] I2C: PB8=SCL PB9=SDA, common GND, 3.3V sensor supply; external pull-ups optional if unreliable.\r\n");
#endif

#if APP_ENABLE_ESP8266
  MX_USART3_UART_Init();
  printf("[INIT] USART3 115200 AT -> ESP8266 (PB10 TX->ESP RX, PB11 RX<-ESP TX)\r\n");
#if APP_ENABLE_OLED
  if (g_oled_ok != 0U)
  {
    App_OledShowStatus("SENSOR READY", "ESP INIT", "WAIT");
  }
#endif
  if (Esp8266_Setup() == HAL_OK)
  {
    g_esp8266_ok = 1U;
    printf("[INIT] ESP8266 WiFi connected\r\n");
#if APP_ENABLE_OLED
    if (g_oled_ok != 0U)
    {
      App_OledShowStatus("SENSOR READY", "ESP OK", "RUN");
    }
#endif
  }
  else
  {
    printf("[INIT] ESP8266 setup failed (wiring / SSID / password in esp8266_http.h)\r\n");
#if APP_ENABLE_OLED
    if (g_oled_ok != 0U)
    {
      App_OledShowStatus("SENSOR READY", "ESP ERR", "RUN LOCAL");
    }
#endif
  }
#endif

  printf("[INIT] All startup steps finished; entering main loop (sensor ~2s, heartbeat 5s).\r\n");
  printf("================================\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    {
      static uint32_t next_hb_ms = 0U;
      uint32_t now = HAL_GetTick();
      if (next_hb_ms == 0U)
      {
        next_hb_ms = now + 5000U;
      }
      else if (now >= next_hb_ms)
      {
        next_hb_ms = now + 5000U;
        printf("[heartbeat] tick=%lu ms\r\n", (unsigned long)now);
#if APP_DEBUG_UART
        {
          static const uint8_t hb_raw[] = "[HB-RAW] 5s OK\r\n";
          (void)HAL_UART_Transmit(&huart1, (uint8_t *)hb_raw, sizeof(hb_raw) - 1U, 50U);
        }
#endif
      }
    }

    /* Hot-plug / late power-up: retry targeted probes (quiet when bus empty). */
    {
      static uint32_t next_retry_ms = 0U;
      uint32_t now_r = HAL_GetTick();

#if APP_ENABLE_AHT10
      if ((g_aht10_ready == 0U) || (g_sgp30_ready == 0U))
#else
      if (g_sgp30_ready == 0U)
#endif
      {
        if (next_retry_ms == 0U)
        {
          next_retry_ms = now_r + APP_RUN_RETRY_FIRST_MS;
        }
        else if (now_r >= next_retry_ms)
        {
          next_retry_ms = now_r + APP_RUN_RETRY_PERIOD_MS;
          printf("[RUN] retry:");
#if APP_ENABLE_AHT10
          if (g_aht10_ready == 0U)
          {
            printf(" AHT10");
          }
#endif
          if (g_sgp30_ready == 0U)
          {
            printf(" SGP30");
          }
          printf("\r\n");
#if APP_ENABLE_AHT10
          if (g_aht10_ready == 0U)
          {
            (void)App_TryInitAht10("RUN");
          }
#endif
          if (g_sgp30_ready == 0U)
          {
            (void)App_TryInitSgp30("RUN");
          }
        }
      }
    }

    float temperature = 0.0f;
    float humidity = 0.0f;
    uint16_t eco2_ppm = 0U;
    uint16_t tvoc_ppb = 0U;
    uint32_t i2c_snap_state = 0U;
    uint32_t i2c_snap_lock = 0U;
    uint32_t i2c_snap_err = 0U;
    uint8_t did_aht = 0U;
    uint8_t did_sgp = 0U;

    HAL_StatusTypeDef aht_status = HAL_ERROR;
    HAL_StatusTypeDef sgp_status = HAL_ERROR;

#if APP_ENABLE_AHT10
    if (g_aht10_ready != 0U)
    {
      did_aht = 1U;
      aht_status = AHT10_Read(&hi2c1, &temperature, &humidity);
      if (aht_status != HAL_OK)
      {
        i2c_snap_state = (uint32_t)HAL_I2C_GetState(&hi2c1);
        i2c_snap_lock = (uint32_t)hi2c1.Lock;
        i2c_snap_err |= HAL_I2C_GetError(&hi2c1);
        I2C1_RecoverFromI2cFailure(aht_status);
      }
    }
#endif

    if (g_sgp30_ready != 0U)
    {
      did_sgp = 1U;
      sgp_status = SGP30_ReadAirQuality(&hi2c1, &eco2_ppm, &tvoc_ppb);
      if (sgp_status != HAL_OK)
      {
        uint8_t sgp_diag = SGP30_GetLastDiag();

        i2c_snap_state = (uint32_t)HAL_I2C_GetState(&hi2c1);
        i2c_snap_lock = (uint32_t)hi2c1.Lock;
        i2c_snap_err |= HAL_I2C_GetError(&hi2c1);
        /* CRC-only: I2C RX completed — bus timing OK; DeInit/reinit often resets clock to 100 kHz and hides the real issue. */
        if ((sgp_diag & (SGP30_DIAG_TX_FAIL | SGP30_DIAG_RX_FAIL)) != 0U)
        {
          I2C1_RecoverFromI2cFailure(sgp_status);
        }
        /* SGP30 must see iaq_init again after any I2C DeInit; harmless after CRC-only soft failure too. */
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)((uint16_t)SGP30_GetAddr7() << 1U), 2U, 25U) == HAL_OK)
        {
          (void)SGP30_Init(&hi2c1);
        }
      }
    }

#if APP_ENABLE_AHT10
    if ((did_aht == 0U) && (did_sgp == 0U))
    {
      App_I2c1_LogBusStatusIfIdle(HAL_GetTick());
    }
    else if ((did_aht != 0U) && (did_sgp != 0U) && (aht_status == HAL_OK) && (sgp_status == HAL_OK))
    {
      printf("T=%.2fC RH=%.2f%% eCO2=%uppm TVOC=%uppb\r\n",
             temperature, humidity, (unsigned)eco2_ppm, (unsigned)tvoc_ppb);
    }
    else if ((did_aht != 0U) && (did_sgp != 0U) && (aht_status == HAL_OK))
    {
      printf("T=%.2fC RH=%.2f%% SGP30 err=%d\r\n",
             temperature, humidity, (int)sgp_status);
    }
    else if ((did_aht != 0U) && (did_sgp != 0U) && (sgp_status == HAL_OK))
    {
      printf("AHT10 err=%d eCO2=%uppm TVOC=%uppb\r\n",
             (int)aht_status, (unsigned)eco2_ppm, (unsigned)tvoc_ppb);
    }
    else if ((did_aht == 0U) && (did_sgp != 0U))
    {
      if (sgp_status == HAL_OK)
      {
        printf("SGP30 only: eCO2=%uppm TVOC=%uppb\r\n",
               (unsigned)eco2_ppm, (unsigned)tvoc_ppb);
      }
      else
      {
        uint8_t sgp_diag = SGP30_GetLastDiag();
        printf("SGP30 %s diag=0x%02x(", App_HalStatusStr(sgp_status), (unsigned)sgp_diag);
        if (sgp_diag == 0U) { printf("none"); }
        else
        {
          if ((sgp_diag & (1U << 0)) != 0U) { printf(" TX"); }
          if ((sgp_diag & (1U << 1)) != 0U) { printf(" RX"); }
          if ((sgp_diag & (1U << 2)) != 0U) { printf(" CRC"); }
        }
        printf(") |");
        App_PrintI2cDecodeRaw(i2c_snap_state, i2c_snap_lock, i2c_snap_err);
      }
    }
    else if ((did_aht != 0U) && (did_sgp == 0U))
    {
      if (aht_status == HAL_OK)
      {
        printf("AHT10 only: T=%.2fC RH=%.2f%%\r\n", temperature, humidity);
      }
      else
      {
        printf("AHT10 %s |", App_HalStatusStr(aht_status));
        App_PrintI2cDecodeRaw(i2c_snap_state, i2c_snap_lock, i2c_snap_err);
      }
    }
    else
    {
      printf("AHT10 %s SGP30 %s |", App_HalStatusStr(aht_status), App_HalStatusStr(sgp_status));
      App_PrintI2cDecodeRaw(i2c_snap_state, i2c_snap_lock, i2c_snap_err);
    }
#else
    if (did_sgp == 0U)
    {
      App_I2c1_LogBusStatusIfIdle(HAL_GetTick());
    }
    else if (sgp_status == HAL_OK)
    {
      printf("SGP30: eCO2=%uppm TVOC=%uppb\r\n",
             (unsigned)eco2_ppm, (unsigned)tvoc_ppb);
    }
    else
    {
      uint8_t sgp_diag = SGP30_GetLastDiag();
      printf("SGP30 %s diag=0x%02x(", App_HalStatusStr(sgp_status), (unsigned)sgp_diag);
      if (sgp_diag == 0U) { printf("none"); }
      else
      {
        if ((sgp_diag & (1U << 0)) != 0U) { printf(" TX"); }
        if ((sgp_diag & (1U << 1)) != 0U) { printf(" RX"); }
        if ((sgp_diag & (1U << 2)) != 0U) { printf(" CRC"); }
      }
      printf(") |");
      App_PrintI2cDecodeRaw(i2c_snap_state, i2c_snap_lock, i2c_snap_err);
    }
#endif

#if APP_ENABLE_OLED
    if (g_oled_ok != 0U)
    {
      uint8_t aht_disp = 0U;
      uint8_t sgp_disp = 0U;
#if APP_ENABLE_AHT10
      if ((g_aht10_ready != 0U) && (did_aht != 0U))
      {
        aht_disp = (aht_status == HAL_OK) ? 1U : 2U;
      }
#endif
      if ((g_sgp30_ready != 0U) && (did_sgp != 0U))
      {
        sgp_disp = (sgp_status == HAL_OK) ? 1U : 2U;
      }
      App_OledRedraw(temperature, humidity, aht_disp, eco2_ppm, tvoc_ppb, sgp_disp);
    }
#endif

#if APP_ENABLE_ESP8266
    if (g_esp8266_ok != 0U)
    {
      Esp8266_UploadStats upload = {0};
      uint32_t now_esp = HAL_GetTick();
      uint8_t have_sample = 0U;

#if APP_ENABLE_AHT10
      if ((g_aht10_ready != 0U) && (g_sgp30_ready != 0U) && (aht_status == HAL_OK) && (sgp_status == HAL_OK))
      {
        have_sample = 1U;
      }
#else
      if ((g_sgp30_ready != 0U) && (sgp_status == HAL_OK))
      {
        have_sample = 1U;
      }
#endif
      if (have_sample != 0U)
      {
        App_MinuteStatsAdd(&g_upload_stats, now_esp, temperature, humidity, eco2_ppm, tvoc_ppb);
      }

      if ((g_upload_stats.window_start_ms != 0U) &&
          ((now_esp - g_upload_stats.window_start_ms) >= APP_UPLOAD_WINDOW_MS) &&
          (g_upload_stats.samples != 0U))
      {
        App_MinuteStatsToUpload(&g_upload_stats, &upload);
        printf("[ESP8266] minute samples=%u Tavg=%.2f Tmin=%.2f Tmax=%.2f RHavg=%.2f CO2avg=%u TVOCavg=%u\r\n",
               (unsigned)upload.samples,
               upload.t_avg, upload.t_min, upload.t_max,
               upload.rh_avg, (unsigned)upload.co2_avg, (unsigned)upload.tvoc_avg);
        if (Esp8266_PostSensorJson(&upload) != HAL_OK)
        {
          printf("[ESP8266] HTTP POST failed\r\n");
        }
        else
        {
          printf("[ESP8266] HTTP POST ok\r\n");
        }
        App_MinuteStatsReset(&g_upload_stats, now_esp);
      }
    }
#endif

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(2000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
