#ifndef __AHT10_H__
#define __AHT10_H__

#include "main.h"

HAL_StatusTypeDef AHT10_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef AHT10_Read(I2C_HandleTypeDef *hi2c, float *temperature_c, float *humidity_rh);

#endif /* __AHT10_H__ */
