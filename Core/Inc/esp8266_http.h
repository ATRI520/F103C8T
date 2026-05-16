#ifndef __ESP8266_HTTP_H__
#define __ESP8266_HTTP_H__

#include "main.h"

/* ESP8266 AT UART: USART3 PB10(TX)->ESP RX, PB11(RX)<-ESP TX, GND, 3.3V. USART1 = printf debug. */
#ifndef APP_ENABLE_ESP8266
#define APP_ENABLE_ESP8266 1
#endif

/* Edit for your hotspot / server (same as PC Spring Boot listener). */
#define ESP8266_WIFI_SSID     "mywifi"
#define ESP8266_WIFI_PASS     "12345678"
#define ESP8266_TCP_HOST      "192.168.137.1"
#define ESP8266_TCP_PORT      8080U
#define ESP8266_HTTP_PATH     "/api/data"

/* AT debug: USART1 printf (PA9); AT traffic: USART3 PB10/PB11. */
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

#endif /* __ESP8266_HTTP_H__ */
