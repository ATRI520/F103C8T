#include "esp8266_http.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#if APP_ENABLE_ESP8266

#define ESP8266_RX_TIMEOUT_MS    (5000U)
#define ESP8266_CWJAP_TIMEOUT_MS (25000U)
#define ESP8266_BUF_SZ           (768U)
#define ESP8266_JSON_BUF_SZ      (384U)
/* First AT after power-on: echo "AT" then "OK"; 500 ms is often too short if ESP is busy. */
#define ESP8266_AT_FIRST_MS      (2500U)
#define ESP8266_AT_OTHER_MS      (800U)

#if ESP8266_DEBUG_PRINT
static void Esp8266_DebugResponse(const char *tag, const char *rx)
{
  printf("[ESP8266] %s -> %s\r\n", tag, ((rx != NULL) && (rx[0] != '\0')) ? rx : "(timeout/no response)");
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

  while (HAL_GetTick() < t_end)
  {
    if (HAL_UART_Receive(&huart2, &ch, 1U, 40U) == HAL_OK)
    {
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
      /* Do not use strstr(..., "FAIL") — matches "FAILED", "FAILSAFE", etc. */
      if (strstr(buf, "\r\nFAIL\r\n") != NULL || strstr(buf, "\nFAIL\n") != NULL)
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
  if (strstr(buf, "\r\nFAIL\r\n") != NULL || strstr(buf, "\nFAIL\n") != NULL)
  {
    return 0;
  }
  /* Accept OK after echo: "AT\r\n\r\nOK\r\n" or trailing "OK\r\n". */
  return (strstr(buf, "\r\nOK\r\n") != NULL) || (strstr(buf, "\r\nOK") != NULL) ||
         (strstr(buf, "OK\r\n") != NULL);
}

static int Esp8266_ResponseHasSendSuccess(const char *buf)
{
  if ((buf == NULL) || (buf[0] == '\0'))
  {
    return 0;
  }
  if (strstr(buf, "ERROR") != NULL)
  {
    return 0;
  }
  if (strstr(buf, "\r\nFAIL\r\n") != NULL || strstr(buf, "\nFAIL\n") != NULL)
  {
    return 0;
  }
  return (strstr(buf, "SEND OK") != NULL) ||
         (strstr(buf, "200 OK") != NULL) ||
         (strstr(buf, "Recv ") != NULL);
}

static HAL_StatusTypeDef Esp8266_SendLine(const char *tag, const char *line, uint32_t wait_ms)
{
  static char rx[ESP8266_BUF_SZ];
  HAL_StatusTypeDef st;

  Esp8266_DrainRx(30U);
#if ESP8266_DEBUG_PRINT
  printf("[ESP8266] TX %s\r\n", tag);
#endif
  if (HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)strlen(line), 2000U) != HAL_OK)
  {
#if ESP8266_DEBUG_PRINT
    printf("[ESP8266] TX %s uart error\r\n", tag);
#endif
    return HAL_ERROR;
  }
  (void)memset(rx, 0, sizeof(rx));
  (void)Esp8266_ReadUntilIdle(rx, sizeof(rx), wait_ms);
  Esp8266_DebugResponse(tag, rx);
  st = Esp8266_ResponseHasOk(rx) ? HAL_OK : HAL_ERROR;
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

HAL_StatusTypeDef Esp8266_Setup(void)
{
  char cmd[96];

  /* Boot garbage on UART + ESP power stable */
  HAL_Delay(300);
  Esp8266_DrainRx(200U);
  printf("[ESP8266] setup start: USART2 PA2->ESP_RX PA3<-ESP_TX, debug on USART1 PA9/PA10\r\n");
  if (Esp8266_SendLine("AT", "AT\r\n", ESP8266_AT_FIRST_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (Esp8266_SendLine("ATE0", "ATE0\r\n", ESP8266_AT_OTHER_MS) != HAL_OK)
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
  char json[ESP8266_JSON_BUF_SZ];
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
    if (Esp8266_ResponseHasSendSuccess(rx) == 0)
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

#endif /* APP_ENABLE_ESP8266 */
