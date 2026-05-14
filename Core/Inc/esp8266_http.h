#ifndef __ESP8266_HTTP_H__
#define __ESP8266_HTTP_H__

#include "main.h"

/* Set 1 after wiring: STM32 PA2(TX)->ESP RX, PA3(RX)->ESP TX, GND, 3.3V. USART1 stays printf. */
#ifndef APP_ENABLE_ESP8266
#define APP_ENABLE_ESP8266 1
#endif

/* Edit for your hotspot / server (same as PC Spring Boot listener). */
#define ESP8266_WIFI_SSID     "mywifi"
#define ESP8266_WIFI_PASS     "12345678"
#define ESP8266_TCP_HOST      "192.168.137.1"
#define ESP8266_TCP_PORT      8080U
#define ESP8266_HTTP_PATH     "/api/data"

/* AT debug is printed by USART1 printf (PA9/PA10); AT traffic itself uses USART2. */
#ifndef ESP8266_DEBUG_PRINT
#define ESP8266_DEBUG_PRINT   1
#endif

typedef struct
{
  float t;
  float rh;
  uint16_t co2;
  uint16_t tvoc;
  float t_min;
  float t_max;
  float t_avg;
  float rh_min;
  float rh_max;
  float rh_avg;
  uint16_t co2_min;
  uint16_t co2_max;
  uint16_t co2_avg;
  uint16_t tvoc_min;
  uint16_t tvoc_max;
  uint16_t tvoc_avg;
  uint16_t samples;
} Esp8266_UploadStats;

HAL_StatusTypeDef Esp8266_Setup(void);
HAL_StatusTypeDef Esp8266_PostSensorJson(const Esp8266_UploadStats *stats);
const char *Esp8266_GetLastDiagLine1(void);
const char *Esp8266_GetLastDiagLine2(void);
const char *Esp8266_GetLastDiagLine3(void);
const char *Esp8266_GetLastDiagLine4(void);
const char *Esp8266_GetLastDiagLine5(void);
const char *Esp8266_GetLastDiagLine6(void);

#endif /* __ESP8266_HTTP_H__ */
