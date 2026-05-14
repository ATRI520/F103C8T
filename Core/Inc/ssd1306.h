#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "main.h"

#define SSD1306_WIDTH        128U
#define SSD1306_HEIGHT       64U
#define SSD1306_PAGES         8U
#define SSD1306_BUFSIZE    1024U

/* Most 0.96" modules use 0x3C; some use 0x3D (solder jumper). */
#define SSD1306_I2C_ADDR7_DEFAULT  (0x3CU)

HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr7);
void SSD1306_DisplayOn(void);
void SSD1306_SetContrast(uint8_t level);
void SSD1306_Clear(void);
void SSD1306_Fill(uint8_t on);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t on);
/* Draw 5x7 glyph in one page row; x in 0..127, page in 0..7. */
void SSD1306_DrawChar5x7(uint8_t x, uint8_t page, char c);
void SSD1306_DrawStr5x7(uint8_t x, uint8_t page, const char *s);
HAL_StatusTypeDef SSD1306_Update(I2C_HandleTypeDef *hi2c);

#endif /* __SSD1306_H__ */
