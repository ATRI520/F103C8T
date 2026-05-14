#include "aht10.h"
#include "i2c.h"

#define AHT10_ADDR_7BIT          (0x38U)
#define AHT10_ADDR               (AHT10_ADDR_7BIT << 1)
#define AHT10_CMD_INIT           (0xE1U)
#define AHT20_CMD_INIT           (0xBEU)
#define AHT10_CMD_SOFT_RESET     (0xBAU)
#define AHT10_CMD_TRIGGER        (0xACU)
#define AHT10_CALIBRATED_MASK    (0x08U)
#define AHT10_BUSY_MASK          (0x80U)
#define AHT10_I2C_TIMEOUT_MS     (150U)

static void AHT10_RestoreI2cSpeed(I2C_HandleTypeDef *hi2c, uint32_t prev_hz)
{
  if (hi2c->Instance == I2C1)
  {
    I2C1_SetClock(prev_hz);
  }
}

HAL_StatusTypeDef AHT10_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status;
  uint8_t soft_reset[1] = {AHT10_CMD_SOFT_RESET};
  uint8_t init_aht10[3] = {AHT10_CMD_INIT, 0x08U, 0x00U};
  uint8_t init_aht20[3] = {AHT20_CMD_INIT, 0x08U, 0x00U};
  uint32_t attempt;
  uint32_t prev_hz;

  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  prev_hz = hi2c->Init.ClockSpeed;
  if (hi2c->Instance == I2C1)
  {
    /* Slow edges help mixed bus with SGP30; AF on init often clears at 10 kHz. */
    I2C1_SetClock(10000U);
  }

  for (attempt = 0U; attempt < 3U; attempt++)
  {
    if (attempt > 0U)
    {
      HAL_Delay(30);
    }

    status = HAL_I2C_Master_Transmit(hi2c, AHT10_ADDR, soft_reset, sizeof(soft_reset), AHT10_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      continue;
    }
    HAL_Delay(80);

    status = HAL_I2C_Master_Transmit(hi2c, AHT10_ADDR, init_aht10, sizeof(init_aht10), AHT10_I2C_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      AHT10_RestoreI2cSpeed(hi2c, prev_hz);
      return HAL_OK;
    }

    /* Many boards sold as "AHT10" use AHT20/AHT21 (same 0x38); init uses 0xBE not 0xE1. */
    status = HAL_I2C_Master_Transmit(hi2c, AHT10_ADDR, init_aht20, sizeof(init_aht20), AHT10_I2C_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      AHT10_RestoreI2cSpeed(hi2c, prev_hz);
      return HAL_OK;
    }
  }

  AHT10_RestoreI2cSpeed(hi2c, prev_hz);
  return HAL_ERROR;
}

HAL_StatusTypeDef AHT10_Read(I2C_HandleTypeDef *hi2c, float *temperature_c, float *humidity_rh)
{
  HAL_StatusTypeDef status;
  uint8_t trigger_cmd[3] = {AHT10_CMD_TRIGGER, 0x33U, 0x00U};
  uint8_t data[6];
  uint32_t raw_h;
  uint32_t raw_t;

  if ((hi2c == NULL) || (temperature_c == NULL) || (humidity_rh == NULL))
  {
    return HAL_ERROR;
  }

  status = HAL_I2C_Master_Transmit(hi2c, AHT10_ADDR, trigger_cmd, sizeof(trigger_cmd), AHT10_I2C_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  /* Datasheet: measurement time typ. 75 ms; poll until busy clears */
  HAL_Delay(80);

  for (uint32_t attempt = 0U; attempt < 30U; attempt++)
  {
    status = HAL_I2C_Master_Receive(hi2c, AHT10_ADDR, data, sizeof(data), AHT10_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      return status;
    }
    if ((data[0] & AHT10_BUSY_MASK) == 0U)
    {
      break;
    }
    HAL_Delay(10);
  }

  if ((data[0] & AHT10_BUSY_MASK) != 0U)
  {
    return HAL_TIMEOUT;
  }

  /* Some modules clear busy before calibration bit is set; still parse if plausible */
  if ((data[0] & AHT10_CALIBRATED_MASK) == 0U)
  {
    /* allow read to proceed — raw values may still be valid after power-up */
  }

  raw_h = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);
  raw_t = (((uint32_t)data[3] & 0x0FU) << 16) | ((uint32_t)data[4] << 8) | data[5];

  *humidity_rh = ((float)raw_h * 100.0f) / 1048576.0f;
  *temperature_c = ((float)raw_t * 200.0f) / 1048576.0f - 50.0f;

  return HAL_OK;
}
