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

/*RC522-SPI
 SDA-GPIO5
 MISO-GPIO19
 MOSI-GPIO23
 SCK-GPIO18
 RST-EN-Pin9
 /*
 /*DHT11-Serial
 VCC-5V
 GND-0V
 DATA- GPIO4/*
 /*LCD SSD1306 128x32-I2C
 Slave ADDRESS-0X3C
 SDA-GPIO21
 SCL-GPIO22
 VCC-3.3V
 GND-0V
 */

#define mainDELAY_LOOP_COUNT  4000000
#define SS_PIN 5
#define RST_PIN 9
#define DHTPIN 4     // what pin we're connected to
#define DHTTYPE DHT11
#define tag_count 3

#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  15000

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

typedef struct Porfiles {
	int number;
	int temperature[2];
	int fan_speed[2];
	long tag_uid;

} MyPorfiles;

const int Resistor_led = 13;
const int fan_pwm = 16;
const int button = 17;
const int fan_tachometer = 25;
const int signal_led = 26;
DHT dht(DHTPIN, DHTTYPE);
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
// Inicializalcão de um array para guardar os bytes do ID serie da tag

void vTask1(void *pvParameters);
void vTask2(void *pvParameters);
void vTask3(void *pvParameters);
void vTask4(void *pvParameters);
void vTask5(void *pvParameters);
bool my_vApplicationIdleHook(void);
TaskHandle_t xTask2Handle;
SemaphoreHandle_t xBinarySemaphore;

const char *Task1_name = "TASK1-READING RFID\r\n"; //Leitura das tags
const char *Task2_name = "TASK2-ATRIBUICAO DE PERFIS\r\n"; //Comparação da Tag com os perfis
const char *Task3_name = "TASK3-TRATAMENTO DE DADOS\t\n"; //Comparação da temperatura  e velocidade da ventoinha atuais com os valores estabelecidos nos perfis
const char *Task4_name = "TASK4-CONTROLO DE VELOCIDADE e RESISTENCIA\t\n"; //Atuação através de PWM na ventoinha e resitência de aquecimento on/off
const char *Task5_name = "TASK5- LCD\r\n"; //Display da informação

/* Declare a variable of type QueueHandle_t.  This is used to store the queue
 that is accessed by tasks. */
QueueHandle_t xQueue; // tag uid
QueueHandle_t xQueue2; // Queue com os perfis associados
QueueHandle_t xQueue3; // real time fan speed
QueueHandle_t xQueue4; // velocidade do motor
volatile byte half_revolutions;

void setup() {

	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
	esp_register_freertos_idle_hook(my_vApplicationIdleHook);

	Wire.begin();
	Serial.begin(9600);
	SPI.begin(); // Init SPI bus
	rfid.PCD_Init(); // Init MFRC522
	dht.begin();
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
	pinMode(Resistor_led, OUTPUT);
	pinMode(button, OUTPUT);
	attachInterrupt(button, vInterruptHandler, RISING);
	attachInterrupt(fan_tachometer, vMeasure_fan_speed, RISING);

	ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
	ledcAttachPin(fan_pwm, PWM1_Ch);

	xQueue = xQueueCreate(1, sizeof(int));
	xQueue2 = xQueueCreate(1, sizeof(MyPorfiles));
	xQueue3 = xQueueCreate(1, sizeof(float));
	xQueue4 = xQueueCreate(1, sizeof(int));
	vSemaphoreCreateBinary(xBinarySemaphore);

	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0, 0);
	display.println("SCE-Projeto");
	display.setCursor(0, 10);
	display.print("Realizado por:");
	display.setCursor(0, 20);
	display.print("Ruben");
	display.setCursor(20, 20);
	display.print("e Rafael ");
	display.display();

	if (xQueue != NULL && xQueue2 != NULL && xQueue3 != NULL
			&& xBinarySemaphore != NULL) {
		xTaskCreatePinnedToCore(vTask1, "RFID Reader Task", 1024,
				(void*) Task1_name, 4, NULL, 1);
		xTaskCreatePinnedToCore(vTask2, "Porfile Selection Task", 1024,
				(void*) Task2_name, 3, &xTask2Handle, 1);
		xTaskCreatePinnedToCore(vTask3, "Data Processing Task", 1024,
				(void*) Task3_name, 4, NULL, 1);

		xTaskCreatePinnedToCore(vTask4, "Actuactor Task", 1024,
				(void*) Task4_name, 1, NULL, 1);

		xTaskCreatePinnedToCore(vTask5, "LCD Display Task", 1024,
				(void*) Task5_name, 1, NULL, 1);

	}
}

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
//void printHex(byte *buffer, byte bufferSize) {
//for (byte i = 0; i < bufferSize; i++) {
//Serial.print(buffer[i] < 0x10 ? " 0" : " ");
//Serial.print(buffer[i], HEX);
//}
//}
/**
 * Helper routine to dump a byte array as dec values to Serial.
 * Converte formato código ASCII para decimal
 *
 */

void vTask1(void *pvParameters) {
	TickType_t xLastWakeTime;
	unsigned portBASE_TYPE uxPriority;
	xLastWakeTime = xTaskGetTickCount();
	// Recebe prioridade da tarefa
	for (;;) {

		Serial.print("\nTASK1 IS RUNNING");
		display.clearDisplay();
		// Display static text
		display.setCursor(0, 0);
		display.setTextColor(0xFFFF, 0);
		display.setTextSize(1);
		display.setTextColor(SSD1306_WHITE);
		display.println("TASK1 IS RUNNING");
		//Serial.println(F("\nTASK1 IS RUNNING"));
		// Repeat anti collision loop
		//Verifica se existe colisão entre leituras, entra num loop de verificação de anti colosão

		if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {

			Serial.println(F("Tag:"));
			printDec(rfid.uid.uidByte, rfid.uid.size);
			Serial.println();

			// Halt PICC
			rfid.PICC_HaltA();
			// Stop encryption on PCD
			rfid.PCD_StopCrypto1();
			//vTaskPrioritySet(xTask2Handle, (uxPriority + 2));
			uxPriority = uxTaskPriorityGet( NULL);
			vTaskPrioritySet(xTask2Handle, (uxPriority + 1));
			Serial.print("Raise the Task2 priority to ");
			Serial.println(uxPriority + 1);
			vTaskDelayUntil(&xLastWakeTime, 3000 / portTICK_PERIOD_MS); //executa de 3 em 3 segundos para detetar tags

		}

	}

}

void vTask2(void *pvParameters) {

	char *Task_Name;
	int lReceivedValue;

	unsigned portBASE_TYPE uxPriority;

	TickType_t xLastWakeTime;
	MyPorfiles my_porfiles[3];
	MyPorfiles p_selected;
	my_porfiles[0].number = 605;
	my_porfiles[0].temperature[0] = 10;
	my_porfiles[0].temperature[1] = 20;
	my_porfiles[0].fan_speed[0] = 71;
	my_porfiles[0].fan_speed[1] = 80;
	my_porfiles[0].tag_uid = 39200186180;
	my_porfiles[1].number = 526;
	my_porfiles[1].temperature[0] = 20;
	my_porfiles[1].temperature[1] = 25;
	my_porfiles[1].fan_speed[0] = 10;
	my_porfiles[1].fan_speed[1] = 35;
	my_porfiles[1].tag_uid = 12147227140;
	my_porfiles[2].number = 151;
	my_porfiles[2].temperature[0] = 25;
	my_porfiles[2].temperature[1] = 30;
	my_porfiles[2].fan_speed[0] = 71;
	my_porfiles[2].fan_speed[1] = 100;
	my_porfiles[2].tag_uid = 2365954;

	/*my_porfiles[0] = { 605, (20, 22), (10, 35), 39200186180 };	//White tag
	 my_porfiles[1] = { 526, (22, 24), (36, 70), 12147227140 };	//Blue tag
	 my_porfiles[2] = { 151, (24, 26), (71, 100), 2365954 };   //yellow tag*/

	portBASE_TYPE xStatus;
	portBASE_TYPE xStatus2;

	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		Serial.print("\nTASK2 IS RUNNING");

		//display.println("TASK2 IS RUNNING");
		//Serial.println("TASK2 IS RUNNING");
		if (uxQueueMessagesWaiting(xQueue) >= 1)
			xStatus = xQueueReceive(xQueue, &lReceivedValue, 0);

		if (xStatus == pdPASS) {
			uxPriority = uxTaskPriorityGet( NULL);
			Serial.print("\nxQueue data received: ");
			Serial.println(lReceivedValue);

			for (int i = 0; i < tag_count; i++) {
				Serial.print("\nTASK2 IS RUNNING");
				if (lReceivedValue == my_porfiles[i].number) {
					Serial.print("\nPorfile with tag ");
					Serial.print(my_porfiles[i].tag_uid);
					Serial.print("\tselected");
					Serial.print("\nDesired temperature");
					Serial.print(my_porfiles[i].temperature[0]);
					Serial.print("-");
					Serial.print(my_porfiles[i].temperature[1]);
					Serial.print("ºC");
					Serial.print("\nFan speed range");
					Serial.print(my_porfiles[i].fan_speed[0]);
					Serial.print("-");
					Serial.print(my_porfiles[i].fan_speed[1]);
					Serial.print("%");

					if (i == tag_count
							&& lReceivedValue != my_porfiles[i].number)
						Serial.print("\nTag not recognizable ");

					p_selected = my_porfiles[i];
					Serial.print("TAG NUMBER");
					Serial.print(p_selected.tag_uid);
					xStatus2 = xQueueSendToBack(xQueue2, &p_selected, 0);// queue do perfil selectionado
					vTaskPrioritySet(xTask2Handle, (uxPriority - 2)); //apos enviar a queue com o perfil reduz a prioridade para deixar as outras correrem
					//vTaskDelayUntil(&xLastWakeTime, (4000 / portTICK_PERIOD_MS));

				}

			}
			//vTaskPrioritySet(xTask2Handle, (uxPriority - 2)); //apos enviar a queue com o perfil reduz a prioridade para deixar as outras correrem
			//vTaskDelayUntil(&xLastWakeTime, (4000 / portTICK_PERIOD_MS));
		}
		vTaskDelayUntil(&xLastWakeTime, (2000 / (portTICK_PERIOD_MS)));
	}

}

void vTask3(void *pvParameters) {
	float t = dht.readTemperature();
	float temp_diff = 0;
	int fan_rpm;
	TickType_t xLastWakeTime;
	MyPorfiles p;

	portBASE_TYPE xStatus2;
	portBASE_TYPE xStatus3;
	portBASE_TYPE xStatus4; //Rotacoes da ventoinha

	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		//Serial.println(F("\nTASK3 IS RUNNING"));
		float t = dht.readTemperature();

		xStatus3 = xQueueSendToBack(xQueue3, &t, 0);
		xStatus2 = xQueuePeek(xQueue2, &p, 0);
		xStatus4 = xQueuePeek(xQueue4, &fan_rpm, 0);

		if (xStatus2 == pdPASS) {
			xStatus2 = xQueueReceive(xQueue2, &p, 0);
			Serial.print("\nxQueue2 data received with tag uid: ");
			Serial.print(p.tag_uid);
			Serial.print("\nTemperature read: ");
			Serial.print(t);
			if (xStatus4 == pdPASS) {
			}
			Serial.print("\nxQueue4 RPM count received: ");
			Serial.print(fan_rpm);
			Serial.print("RPM");



				float duty_cycle=255-((((p.fan_speed[1]-p.fan_speed[0])/2)+p.fan_speed[0])*2.55);

				ledcWrite(PWM1_Ch, (duty_cycle));



			if (t != NAN) {

				if (t > p.temperature[1]) {

					Serial.println(

					"\nTemperature is too high! Setting heating resistor OFF!");
					digitalWrite(Resistor_led, LOW);
				} else if (t < p.temperature[0]) {
					digitalWrite(Resistor_led, HIGH);
					Serial.println(

					"\nTemperature is too low! Setting heating reasistor ON!");
				}
			} else
				Serial.println(

				"\nTemperature is too high! Setting heating reasistor ON!");

		}

	}
	//vTaskDelayUntil(&xLastWakeTime, (1500 / (portTICK_PERIOD_MS)));
}

void vTask4(void *pvParameters) {

	Serial.print("\nTASK4 IS RUNNING");

	ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
	ledcAttachPin(fan_pwm, PWM1_Ch);

	for (;;) {

	}

	//rpm = (half_revolutions / time_task) * 60;
}
void vTask5(void *pvParameters) {
	TickType_t xLastWakeTime;
	portBASE_TYPE xStatus2;		// queue do perfil selecionado
	portBASE_TYPE xStatus3;		//queue da temperatura atual
	portBASE_TYPE xStatus4;		// queue das rpm
	MyPorfiles p;
	float t;


	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		xSemaphoreTake(xBinarySemaphore, 0);//task should remain in the Blocked state to wait for the semaphore indefinitely
		Serial.print("\nTASK5 IS RUNNING");

		xStatus2 = xQueuePeek(xQueue2, &p, 0);
		xStatus3 = xQueuePeek(xQueue3, &t, 0);
		//xStatus4 = xQueuePeek(xQueue4, &rpm, 0);
		if (digitalRead(button) == HIGH) { /*se o botao de interrupcao continuar premido seleciona se a informacao
		 no display será o perfil selecionado ou os parametros atuais medidos*/

			display.clearDisplay();
			display.setCursor(0, 0);
			display.print("Porfile:");
			display.setCursor(10, 0);						//pos x=10,y=0
			display.print(p.tag_uid);
			display.setCursor(0, 10);
			display.print("Temperature:");
			display.setCursor(15, 10);						//pos x=15,y=10
			display.print(p.temperature[0]);
			display.setCursor(17, 10);
			display.print(p.temperature[1]);
			display.setCursor(0, 20);
			display.print("Speed:");
			display.display();

		} else {

			display.clearDisplay();
			display.setCursor(0, 0);
			display.print("Current Parameters:");
			display.setCursor(0, 10);
			display.print("Temperature:");
			display.setCursor(15, 10);						//pos x=15,y=10
			display.print(t);
			display.setCursor(17, 10);
			display.print("ºC");
			display.setCursor(0, 20);
			//display.print("Speed:");
			//display.setCursor(10, 20);
			//display.print(rpm);
			display.display();

		}
		vTaskDelayUntil(&xLastWakeTime, (3000 / (portTICK_PERIOD_MS)));
	}
}

bool my_vApplicationIdleHook(void) {

	return true;

}
void loop() {
	vTaskDelete( NULL);
}

void printDec(byte *buffer, byte bufferSize) {

	byte id[bufferSize];
	portBASE_TYPE xStatus;
	int Sum = 0;
	long Tag_UID = 0;
	long Tag_UID2 = 0;
	long Tag_UID3 = 0;

	Tag_UID = (buffer[0]) + (buffer[1] << 8) + (buffer[2] << 16)
			+ (buffer[3] << 24);

	Sum = buffer[0] + buffer[1] + buffer[2] + buffer[3];

	if (bufferSize >= 7) {
		Tag_UID2 = (buffer[4] << 32) + (buffer[5] << 40) + (buffer[6] << 48);

		Sum = Sum + buffer[4] + buffer[5] + buffer[6];

		if (bufferSize == 10) {

			Tag_UID3 = (buffer[7] << 56) + (buffer[8] << 64)
					+ (buffer[9] << 72);
		}
		Sum = Sum + buffer[7] + buffer[8] + buffer[9];

	}

	for (byte i = 0; i < bufferSize; i++) {
		id[i] = (Tag_UID + Tag_UID2 + Tag_UID3) >> 8 * i;
	}

	Sum = buffer[0] + buffer[1] + buffer[2] + buffer[3];

	for (byte i = 0; i < bufferSize; i++) {
		Serial.print(id[i] < 0x10 ? " 0" : " ");
		Serial.print(id[i]);

	}
//Apenas manda para a queue 4 bytes, correspondentes a 1 long, porque as tags que utilizamos são apenas de 4 bytes
	xStatus = xQueueSendToBack(xQueue, &Sum, 0);
	Serial.print("\t\tSum:");
	Serial.print(Sum);
	/*Caso se usassem tags de bytes superiores a 4 neste projeto
	 *
	 * xStatus = xQueueSendToBack( xQueue, & Tag_UID, 0 );
	 xStatus = xQueueSendToBack( xQueue, & Tag_UID2, 0 );
	 xStatus = xQueueSendToBack( xQueue, & Tag_UID3, 0 );
	 */
	if (xStatus != pdPASS)
		Serial.print("Could not send to the queue.\r\n");
	if (xStatus == errQUEUE_FULL)
		Serial.print("Queue is full.\r\n");

}

void vInterruptHandler(void) //interrupção para mudar a tela do lcd (VTASK5)
		{
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(xBinarySemaphore,
			(portBASE_TYPE*)&xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken == pdTRUE) {
		vPortYield();
	}
}
void vMeasure_fan_speed(void) {
	portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
	taskENTER_CRITICAL(&myMutex);
	{
		Serial.print(half_revolutions);
		half_revolutions++;
	}
	taskEXIT_CRITICAL(&myMutex);

}
