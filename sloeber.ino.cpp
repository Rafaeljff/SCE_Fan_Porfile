#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2021-12-22 17:17:11

#include "Arduino.h"
#include "Arduino.h"
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_freertos_hooks.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

void setup() ;
void printDec(byte *buffer, byte bufferSize) ;
void vTask1(void *pvParameters) ;
void vTask2(void *pvParameters) ;
void vTask3(void *pvParameters) ;
void vTask4(void *pvParameters) ;
void vTask5(void *pvParameters) ;
bool my_vApplicationIdleHook(void) ;
void loop() ;

#include "SCE_PROJECT.ino"


#endif
