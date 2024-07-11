/*
Rafael Ferreira - 2172044
Ruben Susano - 2192281
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
LEEC - Licenciatura em Engenharia Eletrotécnica e de Computadores
SCE - Sistemas Computacionais Embebidos
TPF: Pretende-se  neste  trabalho  prático  a  implementação  de um  algoritmo  que  
permita  o controlo de um sistema de aquecimento através de tags RFID, utilizando 
um sistema operativo de tempo real FreeRTOS.
LINK Github: https://gist.github.com/TheLittleBigFish/6503ef507823a01b8adc039de2279279
*/

#include "Arduino.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_freertos_hooks.h"
// For LCD
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// For RFID
#include <SPI.h>
#include <MFRC522.h>

// Heater
#define LED_PIN 2
#define ADC_1_6 34
#define ADC_RESOLUTION 12
#define VREF_PLUS  5
#define VREF_MINUS  0

// For LCD
#define LCD_ADDRESS_1 0x3C
#define LCD_ADDRESS_2 0x3D

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// For RFID
#define SS_PIN 5
#define RST_PIN 27

MFRC522 rfid(SS_PIN, RST_PIN);   // Create MFRC522 instance.

// For Fan
#define FAN_PWM_PIN 4
#define PWM_FREQ 5000
#define FANCHANNEL 1
#define RESULUTION 8
#define FAN_INTERRUPT 15
#define FAN_RPM_MAX 13000 //1500

// For Interrupt clear
#define INPUT_PIN 0

//QUEUES
QueueHandle_t xQueueSensors;
QueueHandle_t xQueueRFID;
QueueHandle_t xQueueProfile;

//SHEMAPHORE
SemaphoreHandle_t xBinarySemaphoreClear;

//Mutex
portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE counterMutex = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t xMutex;

// Structs
typedef struct sensorVal {
    float temp;
    float speed;
} SensorValues;

typedef struct RFIDVal {
    long RFIDcodeLSB;
    long RFIDcodeMSB;
    int RFIDsize;
} RFIDValues;

typedef struct RFIDProf {
    int ProfileNumber;
    int PWM;
    int Temp[2];
    char ProfileName[6];
} RFIDProfiles;

static void updateVelTemp(void *pvParameters);
static void allRFIDProfiles(void *pvParameters);
static void readRFIDSensorTask(void *pvParameters);
static void readSensorsTask(void *pvParameters);
static void updateLCDPCTask(void *pvParameters);
void clearStop(void *pvParameters);
bool my_vApplicationIdleHook(void);
void IRAM_ATTR clear_EVENT(void);
void IRAM_ATTR counter_EVENT(void);

static char *pcStringsToPrint[] = { "Run: -------- Task Read RFID -------\r\n",
        "Run: ------ Task Read Sensors ------\r\n",
        "Run: -------- Task Write LCD -------\r\n",
        "Run: ----- Task Decide Profile -----\r\n",
        "Run: -------- Task Vel Temp --------\r\n",
        "Run: Clear ------------------- Clear\r\n" };

volatile unsigned long ticksCounted;

/**
 * The setup function is called once at startup of the sketch
 */
void setup() {
    TickType_t xLastWakeTime;

    // Idle Task
    esp_register_freertos_idle_hook(my_vApplicationIdleHook);

    // define Pins
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Set loopTask max priority before deletion
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    // Set I2C for LCD
    if (!display.begin(SSD1306_SWITCHCAPVCC, LCD_ADDRESS_1)) {
        Serial.println(F("Error: -------- SSD1306 allocation failed -------"));
        for (;;)
            ;
    }

    display.display();
    vTaskDelayUntil(&xLastWakeTime, (2000 / portTICK_PERIOD_MS));
    display.clearDisplay();
    display.setTextColor(WHITE);

    // Initiate USART and set Baud-rate to 115200
    Serial.begin(115200);

    // Initiate the RFID
    SPI.begin();      // Initiate  SPI bus
    rfid.PCD_Init();

    // Initiate PWM
    ledcSetup(FANCHANNEL, PWM_FREQ, RESULUTION);
    ledcAttachPin(FAN_PWM_PIN, FANCHANNEL);

    ledcWrite(FANCHANNEL, 255);

    // Define queues
    xQueueSensors = xQueueCreate(4, sizeof(SensorValues));
    xQueueRFID = xQueueCreate(4, sizeof(RFIDValues));
    xQueueProfile = xQueueCreate(4, sizeof(RFIDProfiles));

    // Define Semaphore
    xBinarySemaphoreClear = xSemaphoreCreateBinary();

    // Define mutex
    xMutex = xSemaphoreCreateMutex();

    if (xQueueSensors != NULL && xQueueRFID != NULL && xQueueProfile != NULL
            && xBinarySemaphoreClear != NULL && xMutex != NULL) {
        xTaskCreatePinnedToCore(readRFIDSensorTask, "ReadRFID", 1024, (void*) 0,
                3, NULL, 1);

        xTaskCreatePinnedToCore(readSensorsTask, "ReadSensor", 1024, (void*) 1,
                2, NULL, 1);

        xTaskCreatePinnedToCore(updateLCDPCTask, "UpdateLCDandPC", 2048,
                (void*) 2, 1, NULL, 1);

        xTaskCreatePinnedToCore(allRFIDProfiles, "DecideProfile", 1024,
                (void*) 3, 4, NULL, 1);

        xTaskCreatePinnedToCore(updateVelTemp, "UpdateVelocityTemperature",
                1024, (void*) 4, 2, NULL, 1);

        xTaskCreatePinnedToCore(clearStop, "Clear", 1024, (void*) 5, 4, NULL,
                1);

        // ADC Resolution
        analogReadResolution(ADC_RESOLUTION);

        //interrupts
        attachInterrupt(digitalPinToInterrupt(FAN_INTERRUPT), counter_EVENT,
        CHANGE);
        attachInterrupt(digitalPinToInterrupt(INPUT_PIN), clear_EVENT,
        RISING);
        interrupts();

        xSemaphoreGive(xBinarySemaphoreClear);
    }
}

/**
 * Atualiza a velocidade e temperatura da ventoinha e resistencia termica
 */
static void updateVelTemp(void *pvParameters) {
    TickType_t xLastWakeTime;
    RFIDProfiles rfidProfiles;
    SensorValues values;
    portBASE_TYPE xStatusSensores;
    portBASE_TYPE xStatusProfile;

    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, (2000 / portTICK_PERIOD_MS));

        Serial.print(pcStringsToPrint[(int) pvParameters]);
        Serial.flush();

        xStatusProfile = xQueuePeek(xQueueProfile, &rfidProfiles, 0);
        xStatusSensores = xQueuePeek(xQueueSensors, &values, 0);

        if (xStatusSensores == pdPASS && xStatusProfile == pdPASS) {
            if (rfidProfiles.ProfileNumber != 0) {
                Serial.print("Temp: ");
                Serial.println(values.temp);
                Serial.println(rfidProfiles.Temp[0]);
                Serial.println(rfidProfiles.Temp[1]);

                if (values.temp < rfidProfiles.Temp[0])
                    //ON
                    digitalWrite(LED_PIN, HIGH);
                else if (values.temp > rfidProfiles.Temp[1])
                    //OFF
                    digitalWrite(LED_PIN, LOW);
            } else {
                Serial.println("Clear");
                digitalWrite(LED_PIN, LOW);
                //ledcWrite(FANCHANNEL, 255);
            }

            int perc = 0;
            switch (rfidProfiles.PWM) {
            case 0:
                perc = 0;
                break;
            case 1:
                perc = 50;
                break;
            case 2:
                perc = 100;
                break;
            default:
                perc = 0;
            }

            float speed = perc + (100 - values.speed);
            Serial.print("Speed: ");
            Serial.println(speed);

            if (speed >= 100)
                ledcWrite(FANCHANNEL, 0);
            else {
                if (speed <= 0)
                    ledcWrite(FANCHANNEL, 255);
                else {
                    int speed100 = ((100 - speed) * 255) / 100;
                    Serial.println(speed100);
                    ledcWrite(FANCHANNEL, speed100);
                }
            }
        }
    }
}

/**
 * Guarda todos os prefies de RFID
 */
static void allRFIDProfiles(void *pvParameters) {
    SensorValues values;
    RFIDValues rfidValues;
    RFIDProfiles rfidProfiles;
    RFIDProfiles ProfileClean;
    portBASE_TYPE xStatusRFID;
    portBASE_TYPE xStatusProfile;

    rfidValues = { 0, 0, 0 };

    for (;;) {
        xStatusRFID = xQueueReceive(xQueueRFID, &rfidValues, portMAX_DELAY);

        if (xStatusRFID == pdPASS) {
            Serial.print(pcStringsToPrint[(int) pvParameters]);
            Serial.flush();

            long LSBProfile[4] = { 401, 534, 372, 647 };
            long MSBProfile[4] = { 0, 0, 0, 0 };

            long LSB = rfidValues.RFIDcodeLSB;
            long MSB = rfidValues.RFIDcodeMSB;

            // Profile 1
            // Speed 0 - 0% / 1 - 50% / 2 - 100%
            if (LSB == LSBProfile[0] && MSB == MSBProfile[0]) {
                rfidProfiles.ProfileNumber = 1;
                strcpy(rfidProfiles.ProfileName, "Joao");
                rfidProfiles.PWM = 1;
                rfidProfiles.Temp[0] = 10;
                rfidProfiles.Temp[1] = 20;
            } else if (LSB == LSBProfile[1] && MSB == MSBProfile[1]) {
                rfidProfiles.ProfileNumber = 2;
                strcpy(rfidProfiles.ProfileName, "Manel");
                rfidProfiles.PWM = 0;
                rfidProfiles.Temp[0] = 15;
                rfidProfiles.Temp[1] = 25;
            } else if (LSB == LSBProfile[2] && MSB == MSBProfile[2]) {
                rfidProfiles.ProfileNumber = 3;
                strcpy(rfidProfiles.ProfileName, "Pedro");
                rfidProfiles.PWM = 2;
                rfidProfiles.Temp[0] = 20;
                rfidProfiles.Temp[1] = 25;
            } else if (LSB == LSBProfile[3] && MSB == MSBProfile[3]) {
                rfidProfiles.ProfileNumber = 4;
                strcpy(rfidProfiles.ProfileName, "Jaquim");
                rfidProfiles.PWM = 1;
                rfidProfiles.Temp[0] = 25;
                rfidProfiles.Temp[1] = 30;
            } else {
                Serial.println("Error: ----- Profile does not exist --------");
                continue;
            }

            while (uxQueueMessagesWaiting(xQueueProfile) >= 1)
                xQueueReceive(xQueueProfile, &ProfileClean, 0);

            xStatusProfile = xQueueSendToBack(xQueueProfile, &rfidProfiles, 0);

            if (xStatusProfile != pdPASS) {
                Serial.print(
                        "Error: ----- Could not send to the queue. --------\r\n");
            }
        }
    }

    //}
}

/**
 * Le o cartao RFID e envia a soma dos bytes do mesmo
 */
static void readRFIDSensorTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    RFIDValues rfidValues;
    portBASE_TYPE xStatus;

    xLastWakeTime = xTaskGetTickCount();

    rfidValues = { 0, 0, 4 };

    //infinite for loop
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, (250 / portTICK_PERIOD_MS));

        //print the task number to serial COM
        Serial.print(pcStringsToPrint[(int) pvParameters]);
        Serial.flush();

        // Look for new cards
        xSemaphoreTake(xMutex, portMAX_DELAY);
        {
            if (rfid.PICC_IsNewCardPresent()) {
                if (rfid.PICC_ReadCardSerial()) {
                    long double *rfidSum = 0;

                    Serial.println("Card read");
                    MFRC522::PICC_Type piccType = rfid.PICC_GetType(
                            rfid.uid.sak);

                    byte rfidCode[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
                    rfidValues.RFIDcodeLSB = 0;
                    rfidValues.RFIDcodeMSB = 0;

                    for (int i = 0; i < rfid.uid.size; i++) {
                        rfidCode[i] = rfid.uid.uidByte[i];

                        Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
                        Serial.print(rfid.uid.uidByte[i], HEX);

                        if (i < 5)
                            rfidValues.RFIDcodeLSB += rfid.uid.uidByte[i];
                        else
                            rfidValues.RFIDcodeMSB += rfid.uid.uidByte[i];
                    }

                    rfidValues.RFIDsize = rfid.uid.size;

                    Serial.println();
                    Serial.print("LSB: ");
                    Serial.println(rfidValues.RFIDcodeLSB);
                    Serial.print("MSB: ");
                    Serial.println(rfidValues.RFIDcodeMSB);

                    xStatus = xQueueSendToBack(xQueueRFID, &rfidValues, 0);

                    if (xStatus != pdPASS) {
                        Serial.print(
                                "Error: ----- Could not send to the queue. --------\r\n");
                    }
                }
            }
        }
        xSemaphoreGive(xMutex);

    }
}

/**
 * Le os valores dos sensores de temperatura e velocidade
 */
static void readSensorsTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    int analog_value;
    float analog_voltage;
    float temp;
    SensorValues values;
    SensorValues ValuesClean;
    portBASE_TYPE xStatus;

    xLastWakeTime = xTaskGetTickCount();

    //infinite for loop
    for (;;) {
        //print the task number to serial COM
        Serial.print(pcStringsToPrint[(int) pvParameters]);
        Serial.flush();

        //----------

        //change led HIGH
        analog_value = analogRead(ADC_1_6);
        analog_voltage = analog_value * (VREF_PLUS - VREF_MINUS)
                / (pow(2.0, ADC_RESOLUTION)) + VREF_MINUS;
        // Temperature = voltage/Celsius
        temp = analog_voltage * 100;

        values.temp = temp;

        Serial.print("Ticks: ");
        Serial.println(ticksCounted);

        portENTER_CRITICAL(&counterMutex);
        {
            //2 Hall sensor
            //values.speed = ((((ticksCounted / 2) / 1.5) * 60 / 2) / FAN_RPM_MAX)* 100;
            //1 Hall sensor
            values.speed = ((((ticksCounted / 2) / 1.5) * 60) / FAN_RPM_MAX)* 100;
            ticksCounted = 0;
        }
        portEXIT_CRITICAL(&counterMutex);

        Serial.print("In Task Temp: ");
        Serial.println(values.temp);

        Serial.print("In Task Velocity: ");
        Serial.print(values.speed);
        Serial.println(" %");

        while (uxQueueMessagesWaiting(xQueueSensors) >= 1)
            xQueueReceive(xQueueSensors, &ValuesClean, 0);

        xStatus = xQueueSendToBack(xQueueSensors, &values, 0);
        if (xStatus != pdPASS) {
            /* We could not write to the queue because it was full, this must
             be an error as the queue should never contain more than one item! */
            Serial.print(
                    "Error: ----- Could not send to the queue. --------\r\n");
        }

        vTaskDelayUntil(&xLastWakeTime, (1500 / portTICK_PERIOD_MS));
    }
}

/**
 * Altera os valores amostrados no LCD
 */
static void updateLCDPCTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    SensorValues values;
    RFIDProfiles rfidProfile;
    portBASE_TYPE xStatusSensores;
    portBASE_TYPE xStatusProfile;
    const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;

    xLastWakeTime = xTaskGetTickCount();

    //infinite for loop
    for (;;) {
        //print the task number to serial COM
        Serial.print(pcStringsToPrint[(int) pvParameters]);
        Serial.flush();

        xStatusSensores = xQueuePeek(xQueueSensors, &values, xTicksToWait);
        xStatusProfile = xQueuePeek(xQueueProfile, &rfidProfile, 0);

        if (xStatusSensores == pdPASS && xStatusProfile == pdPASS) {
            // LCD display

            xSemaphoreTake(xMutex, portMAX_DELAY);
            {
                display.clearDisplay();

                display.setTextSize(1);
                display.setCursor(0, 0);
                display.print("RFID Profile: ");
                display.print(rfidProfile.ProfileName);
                // display temperature
                display.setTextSize(1);
                display.setCursor(0, 10);
                display.print("Temperature: ");
                display.print(rfidProfile.Temp[0]);
                display.print("-");
                display.print(rfidProfile.Temp[1]);
                display.print(" ");
                display.cp437(true);
                display.write(167);
                display.print("C");

                display.setTextSize(2);
                display.setCursor(0, 20);
                display.print(String(values.temp));
                display.print(" ");
                display.setTextSize(1);
                display.cp437(true);
                display.write(167);
                display.setTextSize(2);
                display.print("C");

                display.setTextSize(1);
                display.setCursor(0, 40);
                display.print("Velocity: ");
                display.setTextSize(2);
                display.setCursor(0, 50);
                display.print(rfidProfile.PWM);

                vTaskPrioritySet(NULL, 10);

                // display in LCD
                display.display();

                vTaskPrioritySet(NULL, 1);
            }
            xSemaphoreGive(xMutex);

            Serial.print("Display done \r\n");

        } else {
            /* We did not receive anything from the queue even after waiting for
             100ms.*/
            Serial.print(
                    "Error: ----- Could not receive from the queue. --------\r\n");
        }

        vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));
    }
}

/**
 * Para o prefil atual
 */
void clearStop(void *pvParameters) {
    portBASE_TYPE xStatusProfile;
    RFIDProfiles rfidProfiles;
    RFIDProfiles ProfileClean;
    portBASE_TYPE xStatusSemaphore;

    for (;;) {
        xStatusSemaphore = xSemaphoreTake(xBinarySemaphoreClear, portMAX_DELAY);

        if (xStatusSemaphore == pdTRUE) {
            Serial.println(pcStringsToPrint[(int) pvParameters]);
            Serial.flush();

            strcpy(rfidProfiles.ProfileName, "None");
            rfidProfiles.ProfileNumber = 0;
            rfidProfiles.PWM = 0;

            while (uxQueueMessagesWaiting(xQueueProfile) >= 1)
                xQueueReceive(xQueueProfile, &ProfileClean, 0);

            xStatusProfile = xQueueSendToBack(xQueueProfile, &rfidProfiles, 0);

            if (xStatusProfile != pdPASS) {
                Serial.print(
                        "Error: ----- Could not send to the queue. --------\r\n");
            }
        }
    }
}

/**
 * Idle
 */
bool my_vApplicationIdleHook(void) {
    //Serial.print("IDLE\r\n");
    //Serial.flush();
    return true;
}

/**
 * The loop function is called in an endless loop
 */
void loop() {
    vTaskDelete( NULL);
}

/**
 * Interrupt que chama a task clearStop
 */
void IRAM_ATTR clear_EVENT(void) {
    portENTER_CRITICAL(&myMutex);
    {
        static signed portBASE_TYPE xHigherPriorityTaskWoken;

        xHigherPriorityTaskWoken = pdFALSE;

        xSemaphoreGiveFromISR(xBinarySemaphoreClear,
                (signed portBASE_TYPE*)&xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
    portEXIT_CRITICAL(&myMutex);
}

/**
 * Interrupt que conta os ticks da ventuinha
 */
void IRAM_ATTR counter_EVENT(void) {
    portENTER_CRITICAL(&counterMutex);
    {
        ticksCounted++;
    }
    portEXIT_CRITICAL(&counterMutex);
}


