#include "ssd1306.h"
#include <string.h>

#define SSD1306_CTRL_CMD   (0x00U)
#define SSD1306_CTRL_DATA  (0x40U)
#define SSD1306_I2C_TIMEOUT_MS  (200U)

static I2C_HandleTypeDef *s_hi2c;
static uint16_t s_dev_addr_hal;
static uint8_t s_framebuffer[SSD1306_BUFSIZE];

/* 5 columns per glyph, bits 0..6 = rows within one SSD1306 page (row0 = LSB). */
static const uint8_t s_font5x7_digit[10][5] = {
  {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
  {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
  {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
  {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
  {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
  {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
  {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
  {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
  {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
  {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU},
};

static const uint8_t s_font5x7_space[5] = {0, 0, 0, 0, 0};
static const uint8_t s_font5x7_minus[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
static const uint8_t s_font5x7_dot[5]  = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U};
static const uint8_t s_font5x7_pct[5]  = {0x26U, 0x49U, 0x49U, 0x29U, 0x11U};
static const uint8_t s_font5x7_C[5]    = {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U};
static const uint8_t s_font5x7_H[5]    = {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU};
static const uint8_t s_font5x7_e[5]    = {0x38U, 0x54U, 0x54U, 0x54U, 0x18U};
static const uint8_t s_font5x7_p[5]    = {0x7CU, 0x12U, 0x12U, 0x12U, 0x0CU};
static const uint8_t s_font5x7_m[5]   = {0x44U, 0x6DU, 0x55U, 0x55U, 0x44U};
static const uint8_t s_font5x7_E[5]    = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
static const uint8_t s_font5x7_r[5]    = {0x7CU, 0x08U, 0x04U, 0x04U, 0x08U};
static const uint8_t s_font5x7_o[5]    = {0x38U, 0x44U, 0x44U, 0x44U, 0x38U};
static const uint8_t s_font5x7_T[5]    = {0x01U, 0x7FU, 0x01U, 0x01U, 0x01U};
static const uint8_t s_font5x7_colon[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
static const uint8_t s_font5x7_slash[5] = {0x40U, 0x30U, 0x08U, 0x06U, 0x01U};
static const uint8_t s_font5x7_A[5]    = {0x7CU, 0x12U, 0x11U, 0x12U, 0x7CU};
static const uint8_t s_font5x7_B[5]    = {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U};
static const uint8_t s_font5x7_D[5]    = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU};
static const uint8_t s_font5x7_K[5]    = {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U};
static const uint8_t s_font5x7_P[5]    = {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U};
static const uint8_t s_font5x7_R[5]    = {0x7EU, 0x11U, 0x11U, 0x11U, 0x0EU};
static const uint8_t s_font5x7_S[5]    = {0x46U, 0x49U, 0x49U, 0x49U, 0x31U};
static const uint8_t s_font5x7_O[5]    = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
static const uint8_t s_font5x7_V[5]    = {0x1EU, 0x21U, 0x21U, 0x12U, 0x0CU};
static const uint8_t s_font5x7_W[5]    = {0x7FU, 0x20U, 0x18U, 0x20U, 0x7FU};
static const uint8_t s_font5x7_N[5]    = {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU};
static const uint8_t s_font5x7_I[5]    = {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U};
static const uint8_t s_font5x7_F[5]    = {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U};
static const uint8_t s_font5x7_U[5]    = {0x3EU, 0x41U, 0x41U, 0x22U, 0x1CU};
static const uint8_t s_font5x7_G[5]    = {0x3EU, 0x41U, 0x49U, 0x49U, 0x3AU};
static const uint8_t s_font5x7_L[5]    = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U};
static const uint8_t s_font5x7_b[5]    = {0x7FU, 0x48U, 0x44U, 0x44U, 0x38U};
static const uint8_t s_font5x7_y[5]    = {0x08U, 0x14U, 0x62U, 0x02U, 0x02U};
static const uint8_t s_font5x7_k[5]   = {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U};

static HAL_StatusTypeDef SSD1306_WriteRaw(I2C_HandleTypeDef *hi2c, const uint8_t *buf, uint16_t len)
{
  if ((hi2c == NULL) || (buf == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }
  return HAL_I2C_Master_Transmit(hi2c, s_dev_addr_hal, buf, len, SSD1306_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef SSD1306_WriteCommands(I2C_HandleTypeDef *hi2c, const uint8_t *cmds, uint16_t ncmd)
{
  uint8_t tmp[40];
  uint16_t i;

  if ((hi2c == NULL) || (cmds == NULL) || (ncmd == 0U) || (ncmd > (sizeof(tmp) - 1U)))
  {
    return HAL_ERROR;
  }
  tmp[0] = SSD1306_CTRL_CMD;
  for (i = 0U; i < ncmd; i++)
  {
    tmp[1U + i] = cmds[i];
  }
  return SSD1306_WriteRaw(hi2c, tmp, (uint16_t)(1U + ncmd));
}

HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr7)
{
  static const uint8_t init_seq[] = {
    0xAE,
    0xD5, 0x80,
    0xA8, 0x3F,
    0xD3, 0x00,
    0x40,
    0x8D, 0x14,
    0x20, 0x00,
    0xA1,
    0xC8,
    0xDA, 0x12,
    0x81, 0xCF,
    0xD9, 0xF1,
    0xDB, 0x40,
    0xA4,
    0xA6,
    0xAF,
  };
  HAL_StatusTypeDef st;

  if ((hi2c == NULL) || ((i2c_addr7 != 0x3CU) && (i2c_addr7 != 0x3DU)))
  {
    return HAL_ERROR;
  }

  s_hi2c = hi2c;
  s_dev_addr_hal = (uint16_t)((uint16_t)i2c_addr7 << 1U);
  HAL_Delay(50);
  st = SSD1306_WriteCommands(hi2c, init_seq, (uint16_t)sizeof(init_seq));
  if (st != HAL_OK)
  {
    return st;
  }
  SSD1306_Clear();
  return SSD1306_Update(hi2c);
}

void SSD1306_DisplayOn(void)
{
  static const uint8_t c = 0xAFU;

  if (s_hi2c != NULL)
  {
    (void)SSD1306_WriteCommands(s_hi2c, &c, 1U);
  }
}

void SSD1306_SetContrast(uint8_t level)
{
  uint8_t c[2] = {0x81U, level};
  if (s_hi2c != NULL)
  {
    (void)SSD1306_WriteCommands(s_hi2c, c, 2U);
  }
}

void SSD1306_Clear(void)
{
  (void)memset(s_framebuffer, 0x00, sizeof(s_framebuffer));
}

void SSD1306_Fill(uint8_t on)
{
  (void)memset(s_framebuffer, on ? 0xFFU : 0x00U, sizeof(s_framebuffer));
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t on)
{
  uint8_t page;
  uint8_t mask;

  if ((x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT))
  {
    return;
  }
  page = (uint8_t)(y >> 3);
  mask = (uint8_t)(1U << (y & 7U));
  if (on != 0U)
  {
    s_framebuffer[(uint16_t)page * SSD1306_WIDTH + x] |= mask;
  }
  else
  {
    s_framebuffer[(uint16_t)page * SSD1306_WIDTH + x] = (uint8_t)(s_framebuffer[(uint16_t)page * SSD1306_WIDTH + x] & (uint8_t)(~mask));
  }
}

static void SSD1306_DrawGlyph5x7(uint8_t x, uint8_t page, const uint8_t g[5])
{
  uint16_t base;
  uint8_t col;

  if (page >= SSD1306_PAGES)
  {
    return;
  }
  if ((uint16_t)x + 5U > SSD1306_WIDTH)
  {
    return;
  }
  base = (uint16_t)page * SSD1306_WIDTH + x;
  for (col = 0U; col < 5U; col++)
  {
    s_framebuffer[base + col] = g[col];
  }
}

void SSD1306_DrawChar5x7(uint8_t x, uint8_t page, char c)
{
  if (c >= '0' && c <= '9')
  {
    SSD1306_DrawGlyph5x7(x, page, s_font5x7_digit[(unsigned)(c - '0')]);
    return;
  }
  switch (c)
  {
    case ' ':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_space);
      return;
    case '-':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_minus);
      return;
    case '.':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_dot);
      return;
    case '%':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_pct);
      return;
    case 'C':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_C);
      return;
    case 'H':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_H);
      return;
    case 'e':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_e);
      return;
    case 'p':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_p);
      return;
    case 'm':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_m);
      return;
    case 'E':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_E);
      return;
    case 'r':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_r);
      return;
    case 'o':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_o);
      return;
    case 'T':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_T);
      return;
    case ':':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_colon);
      return;
    case '/':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_slash);
      return;
    case 'A':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_A);
      return;
    case 'B':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_B);
      return;
    case 'D':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_D);
      return;
    case 'K':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_K);
      return;
    case 'P':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_P);
      return;
    case 'R':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_R);
      return;
    case 'S':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_S);
      return;
    case 'O':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_O);
      return;
    case 'V':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_V);
      return;
    case 'W':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_W);
      return;
    case 'N':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_N);
      return;
    case 'I':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_I);
      return;
    case 'F':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_F);
      return;
    case 'U':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_U);
      return;
    case 'G':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_G);
      return;
    case 'L':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_L);
      return;
    case 'b':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_b);
      return;
    case 'y':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_y);
      return;
    case 'k':
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_k);
      return;
    default:
      SSD1306_DrawGlyph5x7(x, page, s_font5x7_space);
      return;
  }
}

void SSD1306_DrawStr5x7(uint8_t x, uint8_t page, const char *s)
{
  if (s == NULL)
  {
    return;
  }
  while ((*s != '\0') && (x + 6U <= SSD1306_WIDTH))
  {
    SSD1306_DrawChar5x7(x, page, *s);
    x = (uint8_t)(x + 6U);
    s++;
  }
}

HAL_StatusTypeDef SSD1306_Update(I2C_HandleTypeDef *hi2c)
{
  static uint8_t pkt[129];
  const uint8_t setup[] = {
    0x20, 0x00,
    0x21, 0x00, 0x7F,
    0x22, 0x00, 0x07,
  };
  HAL_StatusTypeDef st;
  uint8_t page;

  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  st = SSD1306_WriteCommands(hi2c, setup, (uint16_t)sizeof(setup));
  if (st != HAL_OK)
  {
    return st;
  }

  pkt[0] = SSD1306_CTRL_DATA;
  for (page = 0U; page < SSD1306_PAGES; page++)
  {
    (void)memcpy(&pkt[1], &s_framebuffer[(uint16_t)page * SSD1306_WIDTH], SSD1306_WIDTH);
    st = SSD1306_WriteRaw(hi2c, pkt, (uint16_t)(1U + SSD1306_WIDTH));
    if (st != HAL_OK)
    {
      return st;
    }
  }
  return HAL_OK;
}
