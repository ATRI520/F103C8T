#ifndef __SGP30_H__
#define __SGP30_H__

#include "main.h"

/* Last error bits from SGP30_GetLastDiag() — CRC-only means bytes RX'd but checksum mismatch. */
#define SGP30_DIAG_TX_FAIL   (1U << 0)
#define SGP30_DIAG_RX_FAIL   (1U << 1)
#define SGP30_DIAG_CRC_FAIL  (1U << 2)

/* Default 0x58 (SGP30). Some boards respond at 0x59 (often SGP40/41); set after I2C scan. */
void SGP30_SetAddr7(uint8_t addr7);
uint8_t SGP30_GetAddr7(void);

HAL_StatusTypeDef SGP30_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef SGP30_ReadAirQuality(I2C_HandleTypeDef *hi2c, uint16_t *eco2_ppm, uint16_t *tvoc_ppb);
uint8_t SGP30_GetLastDiag(void);

/* Optional robust bring-up: datasheet serial ID cmd 0x3682; general-call reset per Table 11 */
HAL_StatusTypeDef SGP30_ReadSerialId(I2C_HandleTypeDef *hi2c, uint8_t id9[9]);
HAL_StatusTypeDef SGP30_GeneralCallReset(I2C_HandleTypeDef *hi2c);

/* Cmd 0x202F: one data word + CRC; decode per Sensirion embedded-sgp (product_type = bits 15:12). */
HAL_StatusTypeDef SGP30_ReadFeatureSet(I2C_HandleTypeDef *hi2c, uint16_t *feature_word);

#endif /* __SGP30_H__ */
