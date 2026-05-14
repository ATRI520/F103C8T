#include "esp8266_http.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#if APP_ENABLE_ESP8266

#define ESP8266_RX_TIMEOUT_MS   (5000U)
#define ESP8266_CWJAP_TIMEOUT_MS (25000U)
#define ESP8266_BUF_SZ          (768U)
#define ESP8266_AT_TIMEOUT_MS   (1500U)

static char g_esp8266_diag_line1[22] = "IDLE";
static char g_esp8266_diag_line2[22] = "";
static char g_esp8266_diag_line3[22] = "";
static char g_esp8266_diag_line4[22] = "";
static char g_esp8266_diag_line5[22] = "";
static char g_esp8266_diag_line6[22] = "";
static uint16_t g_esp8266_last_rx_bytes = 0U;

const char *Esp8266_GetLastDiagLine1(void)
{
  return g_esp8266_diag_line1;
}

const char *Esp8266_GetLastDiagLine2(void)
{
  return g_esp8266_diag_line2;
}

const char *Esp8266_GetLastDiagLine3(void)
{
  return g_esp8266_diag_line3;
}

const char *Esp8266_GetLastDiagLine4(void)
{
  return g_esp8266_diag_line4;
}

const char *Esp8266_GetLastDiagLine5(void)
{
  return g_esp8266_diag_line5;
}

const char *Esp8266_GetLastDiagLine6(void)
{
  return g_esp8266_diag_line6;
}

static void Esp8266_SetDiag6(const char *line1, const char *line2, const char *line3,
                             const char *line4, const char *line5, const char *line6)
{
  (void)snprintf(g_esp8266_diag_line1, sizeof(g_esp8266_diag_line1), "%s", (line1 != NULL) ? line1 : "");
  (void)snprintf(g_esp8266_diag_line2, sizeof(g_esp8266_diag_line2), "%s", (line2 != NULL) ? line2 : "");
  (void)snprintf(g_esp8266_diag_line3, sizeof(g_esp8266_diag_line3), "%s", (line3 != NULL) ? line3 : "");
  (void)snprintf(g_esp8266_diag_line4, sizeof(g_esp8266_diag_line4), "%s", (line4 != NULL) ? line4 : "");
  (void)snprintf(g_esp8266_diag_line5, sizeof(g_esp8266_diag_line5), "%s", (line5 != NULL) ? line5 : "");
  (void)snprintf(g_esp8266_diag_line6, sizeof(g_esp8266_diag_line6), "%s", (line6 != NULL) ? line6 : "");
}

static void Esp8266_SummarizeRx(const char *rx, char *out, size_t out_sz)
{
  size_t oi = 0U;
  int last_was_space = 0;

  if ((out == NULL) || (out_sz == 0U))
  {
    return;
  }

  if ((rx == NULL) || (rx[0] == '\0'))
  {
    (void)snprintf(out, out_sz, "RX:none");
    return;
  }

  if (out_sz > 4U)
  {
    out[oi++] = 'R';
    out[oi++] = 'X';
    out[oi++] = ':';
  }

  while ((*rx != '\0') && (oi < (out_sz - 1U)))
  {
    char ch = *rx++;

    if ((ch == '\r') || (ch == '\n') || (ch == '\t'))
    {
      ch = ' ';
    }

    if (((unsigned char)ch < 32U) || ((unsigned char)ch > 126U))
    {
      continue;
    }

    if (ch == ' ')
    {
      if ((oi <= 3U) || (last_was_space != 0))
      {
        continue;
      }
      last_was_space = 1;
    }
    else
    {
      last_was_space = 0;
    }

    out[oi++] = ch;
  }

  out[oi] = '\0';
  if (oi <= 3U)
  {
    (void)snprintf(out, out_sz, "RX:(bin)");
  }
}

#if ESP8266_DEBUG_PRINT
static void Esp8266_EscapeForLog(const char *src, char *dst, size_t dst_sz)
{
  size_t di = 0U;

  if ((dst == NULL) || (dst_sz == 0U))
  {
    return;
  }
  if (src == NULL)
  {
    (void)snprintf(dst, dst_sz, "(null)");
    return;
  }

  while ((*src != '\0') && (di < (dst_sz - 1U)))
  {
    char ch = *src++;

    if ((ch == '\r') && (di + 2U < dst_sz))
    {
      dst[di++] = '\\';
      dst[di++] = 'r';
    }
    else if ((ch == '\n') && (di + 2U < dst_sz))
    {
      dst[di++] = '\\';
      dst[di++] = 'n';
    }
    else if ((ch == '\t') && (di + 2U < dst_sz))
    {
      dst[di++] = '\\';
      dst[di++] = 't';
    }
    else if (((unsigned char)ch >= 32U) && ((unsigned char)ch <= 126U))
    {
      dst[di++] = ch;
    }
    else
    {
      dst[di++] = '.';
    }
  }

  dst[di] = '\0';
}

static void Esp8266_DebugResponse(const char *tag, const char *rx)
{
  char esc[160];
  Esp8266_EscapeForLog(((rx != NULL) && (rx[0] != '\0')) ? rx : "(timeout/no response)", esc, sizeof(esc));
  printf("[ESP8266] %s -> %s\r\n", tag, esc);
}
#else
static void Esp8266_DebugResponse(const char *tag, const char *rx)
{
  (void)tag;
  (void)rx;
}
#endif

static void Esp8266_DrainRx(uint32_t drain_ms)
{
  uint8_t b;
  uint32_t t0 = HAL_GetTick();

  while ((HAL_GetTick() - t0) < drain_ms)
  {
    (void)HAL_UART_Receive(&huart2, &b, 1U, 5U);
  }
}

static HAL_StatusTypeDef Esp8266_ReadUntilIdle(char *buf, size_t buf_sz, uint32_t timeout_ms)
{
  size_t i = 0U;
  uint32_t t_end = HAL_GetTick() + timeout_ms;
  uint8_t ch;

  if ((buf == NULL) || (buf_sz < 2U))
  {
    return HAL_ERROR;
  }
  buf[0] = '\0';
  g_esp8266_last_rx_bytes = 0U;

  while (HAL_GetTick() < t_end)
  {
    if (HAL_UART_Receive(&huart2, &ch, 1U, 40U) == HAL_OK)
    {
      if (g_esp8266_last_rx_bytes < 65535U)
      {
        g_esp8266_last_rx_bytes++;
      }
      if (i < buf_sz - 1U)
      {
        buf[i] = (char)ch;
        i++;
        buf[i] = '\0';
      }
      if (strstr(buf, "\r\nOK\r\n") != NULL)
      {
        break;
      }
      if (strstr(buf, "\r\nERROR\r\n") != NULL || strstr(buf, "ERROR\r\n") != NULL)
      {
        break;
      }
      if (strstr(buf, "SEND OK") != NULL)
      {
        break;
      }
      if (strstr(buf, "FAIL") != NULL)
      {
        break;
      }
    }
  }
  return HAL_OK;
}

static int Esp8266_ResponseHasOk(const char *buf)
{
  if ((buf == NULL) || (buf[0] == '\0'))
  {
    return 0;
  }
  if (strstr(buf, "ERROR\r\n") != NULL || strstr(buf, "\r\nERROR\r\n") != NULL)
  {
    return 0;
  }
  if (strstr(buf, "FAIL") != NULL)
  {
    return 0;
  }
  return strstr(buf, "OK") != NULL;
}

static HAL_StatusTypeDef Esp8266_SendLine(const char *tag, const char *line, uint32_t wait_ms)
{
  static char rx[ESP8266_BUF_SZ];
  char line1[22];
  char line2[22];
  char line3[22];
  char line4[22];
  char line5[22];
  HAL_StatusTypeDef st;
  size_t tx_len;

  Esp8266_DrainRx(30U);
  tx_len = strlen(line);
  (void)snprintf(line1, sizeof(line1), "%s TX", (tag != NULL) ? tag : "?");
  (void)snprintf(line2, sizeof(line2), "TX:%uB wait:%lums",
                 (unsigned)tx_len, (unsigned long)wait_ms);
  Esp8266_SetDiag6(line1, line2, "RX:waiting...", "PA2->ESP RX",
                   "PA3<-ESP TX", "GND common");
#if ESP8266_DEBUG_PRINT
  printf("[ESP8266] TX %s\r\n", tag);
#endif
  if (HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 2000U) != HAL_OK)
  {
    (void)snprintf(line1, sizeof(line1), "%s TX ERR", (tag != NULL) ? tag : "?");
    Esp8266_SetDiag6(line1, "HAL_UART_TX_FAIL", "check huart2 init", "PA2/PA3 wiring",
                     "GND common", "ESP EN high");
#if ESP8266_DEBUG_PRINT
    printf("[ESP8266] TX %s uart error\r\n", tag);
#endif
    return HAL_ERROR;
  }
  (void)memset(rx, 0, sizeof(rx));
  (void)Esp8266_ReadUntilIdle(rx, sizeof(rx), wait_ms);
  Esp8266_SummarizeRx(rx, line2, sizeof(line2));
  (void)snprintf(line3, sizeof(line3), "RX bytes:%u", (unsigned)g_esp8266_last_rx_bytes);
  Esp8266_DebugResponse(tag, rx);
  st = Esp8266_ResponseHasOk(rx) ? HAL_OK : HAL_ERROR;
  if (st == HAL_OK)
  {
    (void)snprintf(line1, sizeof(line1), "%s OK", (tag != NULL) ? tag : "?");
  }
  else
  {
    (void)snprintf(line1, sizeof(line1), "%s FAIL", (tag != NULL) ? tag : "?");
  }
  (void)snprintf(line4, sizeof(line4), "wait:%lums", (unsigned long)wait_ms);
  (void)snprintf(line5, sizeof(line5), "TX:%uB ok", (unsigned)tx_len);
  Esp8266_SetDiag6(line1, line2, line3, line4, line5, "GND/EN/RST/BAUD");
  return st;
}

static HAL_StatusTypeDef Esp8266_WaitChar(char want, uint32_t timeout_ms)
{
  uint32_t t_end = HAL_GetTick() + timeout_ms;
  uint8_t ch;

  while (HAL_GetTick() < t_end)
  {
    if (HAL_UART_Receive(&huart2, &ch, 1U, 40U) == HAL_OK)
    {
      if ((char)ch == want)
      {
        return HAL_OK;
      }
    }
  }
  return HAL_ERROR;
}

static void Esp8266_SoftReset(void)
{
  Esp8266_SetDiag6("AT RESET", "AT+RST", "wait 2800ms", "ESP rebooting",
                   "boot msg normal", "retry after boot");
  Esp8266_DrainRx(50U);
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)"AT+RST\r\n", 8U, 2000U);
  HAL_Delay(2800);
  Esp8266_DrainRx(500U);
}

static HAL_StatusTypeDef Esp8266_SyncAt(void)
{
  uint32_t attempt;
  char line2[22];

  for (attempt = 1U; attempt <= 3U; attempt++)
  {
    (void)snprintf(line2, sizeof(line2), "TRY %lu/3", (unsigned long)attempt);
    Esp8266_SetDiag6("AT SYNC", line2, "send AT", "wait for OK", "PA2/PA3 active", "GND common");
    if (Esp8266_SendLine("AT", "AT\r\n", ESP8266_AT_TIMEOUT_MS) == HAL_OK)
    {
      return HAL_OK;
    }
    HAL_Delay(120);
  }

  Esp8266_SoftReset();

  for (attempt = 1U; attempt <= 2U; attempt++)
  {
    (void)snprintf(line2, sizeof(line2), "RST TRY %lu/2", (unsigned long)attempt);
    Esp8266_SetDiag6("AT SYNC", line2, "send AT", "after reset", "ESP boot done?", "baud still 115200");
    if (Esp8266_SendLine("AT", "AT\r\n", ESP8266_AT_TIMEOUT_MS) == HAL_OK)
    {
      return HAL_OK;
    }
    HAL_Delay(150);
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef Esp8266_Setup(void)
{
  char cmd[96];

  HAL_Delay(400);
  Esp8266_DrainRx(300U);
  Esp8266_SetDiag6("BOOT", "WAIT 400ms", "drain boot junk", "USART2 115200",
                   "ESP EN=3V3", "GND common");
  printf("[ESP8266] setup start: USART2 PA2->ESP_RX PA3<-ESP_TX, debug on USART1 PA9/PA10\r\n");
  if (Esp8266_SyncAt() != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Esp8266_SendLine("ATE0", "ATE0\r\n", 800U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Esp8266_SendLine("CWMODE", "AT+CWMODE=1\r\n", 2000U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  (void)snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ESP8266_WIFI_SSID, ESP8266_WIFI_PASS);
  if (Esp8266_SendLine("CWJAP", cmd, ESP8266_CWJAP_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Esp8266_SendLine("CIPMUX", "AT+CIPMUX=0\r\n", 2000U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  printf("[ESP8266] setup ok\r\n");
  return HAL_OK;
}

HAL_StatusTypeDef Esp8266_PostSensorJson(const Esp8266_UploadStats *stats)
{
  char json[384];
  char http[ESP8266_BUF_SZ];
  char cmd[80];
  int jl;
  int hl;

  if (stats == NULL)
  {
    return HAL_ERROR;
  }

  jl = (int)snprintf(json, sizeof(json),
                     "{\"t\":%.2f,\"rh\":%.2f,\"co2\":%u,\"tvoc\":%u,"
                     "\"samples\":%u,"
                     "\"tMin\":%.2f,\"tMax\":%.2f,\"tAvg\":%.2f,"
                     "\"rhMin\":%.2f,\"rhMax\":%.2f,\"rhAvg\":%.2f,"
                     "\"co2Min\":%u,\"co2Max\":%u,\"co2Avg\":%u,"
                     "\"tvocMin\":%u,\"tvocMax\":%u,\"tvocAvg\":%u}",
                     (double)stats->t, (double)stats->rh,
                     (unsigned)stats->co2, (unsigned)stats->tvoc,
                     (unsigned)stats->samples,
                     (double)stats->t_min, (double)stats->t_max, (double)stats->t_avg,
                     (double)stats->rh_min, (double)stats->rh_max, (double)stats->rh_avg,
                     (unsigned)stats->co2_min, (unsigned)stats->co2_max, (unsigned)stats->co2_avg,
                     (unsigned)stats->tvoc_min, (unsigned)stats->tvoc_max, (unsigned)stats->tvoc_avg);
  if ((jl <= 0) || (jl >= (int)sizeof(json)))
  {
    return HAL_ERROR;
  }

  hl = (int)snprintf(http, sizeof(http),
                     "POST %s HTTP/1.1\r\n"
                     "Host: %s\r\n"
                     "Content-Type: application/json\r\n"
                     "Content-Length: %d\r\n"
                     "\r\n"
                     "%s",
                     ESP8266_HTTP_PATH, ESP8266_TCP_HOST, jl, json);
  if ((hl <= 0) || (hl >= (int)sizeof(http)))
  {
    return HAL_ERROR;
  }

  (void)snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",
                 ESP8266_TCP_HOST, (unsigned)ESP8266_TCP_PORT);
  printf("[ESP8266] POST JSON %s\r\n", json);
  if (Esp8266_SendLine("CIPSTART", cmd, ESP8266_RX_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  (void)snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", hl);
  Esp8266_DrainRx(50U);
#if ESP8266_DEBUG_PRINT
  printf("[ESP8266] TX CIPSEND len=%d\r\n", hl);
#endif
  if (HAL_UART_Transmit(&huart2, (uint8_t *)cmd, (uint16_t)strlen(cmd), 2000U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Esp8266_WaitChar('>', 4000U) != HAL_OK)
  {
#if ESP8266_DEBUG_PRINT
    printf("[ESP8266] CIPSEND prompt timeout\r\n");
#endif
    return HAL_ERROR;
  }
#if ESP8266_DEBUG_PRINT
  printf("[ESP8266] sending HTTP POST payload\r\n");
#endif
  if (HAL_UART_Transmit(&huart2, (uint8_t *)http, (uint16_t)hl, 5000U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  {
    static char rx[ESP8266_BUF_SZ];

    (void)memset(rx, 0, sizeof(rx));
    (void)Esp8266_ReadUntilIdle(rx, sizeof(rx), 8000U);
    Esp8266_DebugResponse("HTTP SEND", rx);
    if (strstr(rx, "SEND OK") == NULL)
    {
      (void)Esp8266_SendLine("CIPCLOSE", "AT+CIPCLOSE\r\n", 3000U);
      return HAL_ERROR;
    }
  }

  (void)Esp8266_SendLine("CIPCLOSE", "AT+CIPCLOSE\r\n", 3000U);
  return HAL_OK;
}

#else /* !APP_ENABLE_ESP8266 */

HAL_StatusTypeDef Esp8266_Setup(void)
{
  return HAL_ERROR;
}

HAL_StatusTypeDef Esp8266_PostSensorJson(const Esp8266_UploadStats *stats)
{
  (void)stats;
  return HAL_ERROR;
}

const char *Esp8266_GetLastDiagLine1(void)
{
  return "ESP OFF";
}

const char *Esp8266_GetLastDiagLine2(void)
{
  return "";
}

const char *Esp8266_GetLastDiagLine3(void)
{
  return "";
}

const char *Esp8266_GetLastDiagLine4(void)
{
  return "";
}

const char *Esp8266_GetLastDiagLine5(void)
{
  return "";
}

const char *Esp8266_GetLastDiagLine6(void)
{
  return "";
}

#endif /* APP_ENABLE_ESP8266 */
