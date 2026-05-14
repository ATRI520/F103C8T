/* 1 = accept MSB/LSB after successful I2C RX without Sensirion CRC (marginal bus / debug). Set 0 for production. */
#define SGP30_SKIP_CRC 1

#include "sgp30.h"
#include "i2c.h"

#define SGP30_ADDR_7BIT_DEFAULT (0x58U)

/* 7-bit I2C address; HAL expects (addr7 << 1). */
static uint8_t s_sgp30_addr7 = SGP30_ADDR_7BIT_DEFAULT;

#define SGP30_ADDR_HAL          ((uint16_t)((uint16_t)s_sgp30_addr7 << 1U))
#define SGP30_CMD_IAQ_INIT_MSB  (0x20U)
#define SGP30_CMD_IAQ_INIT_LSB  (0x03U)
#define SGP30_CMD_IAQ_MEAS_MSB  (0x20U)
#define SGP30_CMD_IAQ_MEAS_LSB  (0x08U)
#define SGP30_CMD_GET_SERIAL_MSB (0x36U)
#define SGP30_CMD_GET_SERIAL_LSB (0x82U)
#define SGP30_CMD_GET_FEATURE_MSB (0x20U)
#define SGP30_CMD_GET_FEATURE_LSB (0x2FU)
#define SGP30_I2C_TIMEOUT_MS    (150U)

static uint8_t s_sgp30_last_diag = 0U;

void SGP30_SetAddr7(uint8_t addr7)
{
  /* SGP30=0x58; 0x59 used by SGP40/SGP41 and some mislabeled modules. */
  if ((addr7 == 0x58U) || (addr7 == 0x59U))
  {
    s_sgp30_addr7 = addr7;
  }
}

uint8_t SGP30_GetAddr7(void)
{
  return s_sgp30_addr7;
}

#if !SGP30_SKIP_CRC
static uint8_t SGP30_CRC8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0xFFU;
  uint8_t i;
  uint8_t bit;

  for (i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1) ^ 0x31U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}
#endif /* !SGP30_SKIP_CRC */

HAL_StatusTypeDef SGP30_GeneralCallReset(I2C_HandleTypeDef *hi2c)
{
  uint8_t rst = 0x06U;

  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  /* I2C general call + reset byte (datasheet Table 11); resets all devices that implement it. */
  return HAL_I2C_Master_Transmit(hi2c, 0x00U, &rst, 1U, SGP30_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef SGP30_ReadSerialId(I2C_HandleTypeDef *hi2c, uint8_t id9[9])
{
  uint8_t cmd[2] = {SGP30_CMD_GET_SERIAL_MSB, SGP30_CMD_GET_SERIAL_LSB};
  HAL_StatusTypeDef st;

  if ((hi2c == NULL) || (id9 == NULL))
  {
    return HAL_ERROR;
  }

  st = HAL_I2C_Master_Transmit(hi2c, SGP30_ADDR_HAL, cmd, sizeof(cmd), SGP30_I2C_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

  /* Sensirion driver: SGP30_CMD_GET_SERIAL_ID_DURATION_US = 500 (~0.5 ms); allow margin. */
  HAL_Delay(2);

  st = HAL_I2C_Master_Receive(hi2c, SGP30_ADDR_HAL, id9, 9U, SGP30_I2C_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

#if !SGP30_SKIP_CRC
  if ((SGP30_CRC8(&id9[0], 2U) != id9[2]) || (SGP30_CRC8(&id9[3], 2U) != id9[5]) ||
      (SGP30_CRC8(&id9[6], 2U) != id9[8]))
  {
    return HAL_ERROR;
  }
#endif

  return HAL_OK;
}

HAL_StatusTypeDef SGP30_ReadFeatureSet(I2C_HandleTypeDef *hi2c, uint16_t *feature_word)
{
  uint8_t cmd[2] = {SGP30_CMD_GET_FEATURE_MSB, SGP30_CMD_GET_FEATURE_LSB};
  uint8_t data[3];
  HAL_StatusTypeDef st;

  if ((hi2c == NULL) || (feature_word == NULL))
  {
    return HAL_ERROR;
  }

  st = HAL_I2C_Master_Transmit(hi2c, SGP30_ADDR_HAL, cmd, sizeof(cmd), SGP30_I2C_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

  /* Sensirion: SGP30_CMD_GET_FEATURESET_DURATION_US = 10000 */
  HAL_Delay(10);

  st = HAL_I2C_Master_Receive(hi2c, SGP30_ADDR_HAL, data, sizeof(data), SGP30_I2C_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

#if !SGP30_SKIP_CRC
  if (SGP30_CRC8(&data[0], 2U) != data[2])
  {
    return HAL_ERROR;
  }
#endif

  *feature_word = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
  return HAL_OK;
}

HAL_StatusTypeDef SGP30_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t cmd[2] = {SGP30_CMD_IAQ_INIT_MSB, SGP30_CMD_IAQ_INIT_LSB};

  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Master_Transmit(hi2c, SGP30_ADDR_HAL, cmd, sizeof(cmd), SGP30_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef SGP30_ReadAirQuality(I2C_HandleTypeDef *hi2c, uint16_t *eco2_ppm, uint16_t *tvoc_ppb)
{
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef last_error = HAL_ERROR;
  uint8_t cmd[2] = {SGP30_CMD_IAQ_MEAS_MSB, SGP30_CMD_IAQ_MEAS_LSB};
  uint8_t data[6];
  uint32_t attempt;
  /* Sensirion embedded-sgp: SGP30_CMD_IAQ_MEASURE_DURATION_US = 12000 (Table 10 max 12 ms). */
  static const uint8_t k_measure_delay_ms[3] = {15U, 30U, 50U};

  if ((hi2c == NULL) || (eco2_ppm == NULL) || (tvoc_ppb == NULL))
  {
    return HAL_ERROR;
  }

  s_sgp30_last_diag = 0U;
  I2C1_SetClock(50000U);

  for (attempt = 0U; attempt < 3U; attempt++)
  {
    /* Diagnose only this attempt — old code OR'd bits across retries so 0x05 looked like TX+CRC in one xfer. */
    s_sgp30_last_diag = 0U;

    if (attempt > 0U)
    {
      HAL_Delay(5);
    }

    if (attempt == 0U)
    {
      I2C1_SetClock(50000U);
    }
    else if (attempt == 1U)
    {
      I2C1_SetClock(25000U);
    }
    else
    {
      I2C1_SetClock(10000U);
    }

    status = HAL_I2C_Master_Transmit(hi2c, SGP30_ADDR_HAL, cmd, sizeof(cmd), SGP30_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      s_sgp30_last_diag = SGP30_DIAG_TX_FAIL;
      last_error = status;
      continue;
    }

    HAL_Delay((uint32_t)k_measure_delay_ms[attempt]);

    status = HAL_I2C_Master_Receive(hi2c, SGP30_ADDR_HAL, data, sizeof(data), SGP30_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      s_sgp30_last_diag = SGP30_DIAG_RX_FAIL;
      last_error = status;
      continue;
    }

#if !SGP30_SKIP_CRC
    if ((SGP30_CRC8(&data[0], 2U) != data[2]) || (SGP30_CRC8(&data[3], 2U) != data[5]))
    {
      s_sgp30_last_diag = SGP30_DIAG_CRC_FAIL;
      last_error = HAL_ERROR;
      continue;
    }
#endif

    *eco2_ppm = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    *tvoc_ppb = (uint16_t)(((uint16_t)data[3] << 8) | data[4]);
    I2C1_SetClock(50000U);
    return HAL_OK;
  }

  I2C1_SetClock(50000U);
  return last_error;
}

uint8_t SGP30_GetLastDiag(void)
{
  return s_sgp30_last_diag;
}
