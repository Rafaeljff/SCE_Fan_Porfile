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

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

struct Porfiles {
	int number;
	int temperature[2];
	int fan_speed[2];
	long tag_uid;

} my_porfiles[tag_count];



const int Resistor_led = 25;
const int fan_pwm = 16;
DHT dht(DHTPIN, DHTTYPE);
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
// Inicializalão de um array para guardar os bytes do ID serie da tag

void vTask1(void *pvParameters);
void vTask2(void *pvParameters);
void vTask3(void *pvParameters);
void vTask4(void *pvParameters);
void vTask5(void *pvParameters);
bool my_vApplicationIdleHook(void);
TaskHandle_t xTask2Handle;

const char *Task1_name = "TASK1-READING RFID\r\n"; //Leitura das tags
const char *Task2_name = "TASK2-ATRIBUICAO DE PERFIS\r\n"; //Comparação da Tag com os perfis
const char *Task3_name = "TASK3-TRATAMENTO DE DADOS\t\n"; //Comparação da temperatura atual com a real
const char *Task4_name = "TASK4-CONTROLO DE VELOCIDADE e TEMPERATURA\t\n"; //Atuação do motor conforme a velocidade definida
const char *Task5_name = "TASK5- LCD\r\n"; //Display da informação

/* Declare a variable of type QueueHandle_t.  This is used to store the queue
 that is accessed by tasks. */
QueueHandle_t xQueue;// tag uid
QueueHandle_t xQueue2; // real time temperature
QueueHandle_t xQueue3;// real time fan speed


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

	xQueue = xQueueCreate(1, sizeof(int));
	xQueue2 = xQueueCreate(1, sizeof(Porfiles));

	my_porfiles[0] = { 605, (20, 22),(10,35), 39200186180};	//White tag
	my_porfiles[1] = { 526, (22, 24),(35,70), 12147227140};	//Blue tag
	my_porfiles[2] = { 151, (24, 26),(70,100), 2365954};   //yellow tag


	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0, 0);
	display.println("SCE-Projeto");
	display.setCursor(0, 10);
	display.print("Realizado por:");
	display.setCursor(0, 20);
	display.print("Ruben");
	display.setCursor(10, 20);
	display.print("e Rafael ");
	display.display();
	delay(5000);
	display.clearDisplay();

	if (xQueue != NULL) {
		xTaskCreatePinnedToCore(vTask1, "RFID Reader Task", 1024,
				(void*) Task1_name, 2, NULL, 1);
		xTaskCreatePinnedToCore(vTask2, "Porfile Selection Task", 1024,
				(void*) Task2_name, 3, &xTask2Handle, 1);
		xTaskCreatePinnedToCore(vTask3, "Data Processing Task", 1024,
				(void*) Task3_name, 1, NULL, 1);

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
	portBASE_TYPE xStatus;
	unsigned portBASE_TYPE uxPriority;
	for (;;) {
		display.clearDisplay();
		// Display static text
		display.setCursor(0, 0);
		display.setTextColor(0xFFFF, 0);
		display.setTextSize(1);
		display.setTextColor(SSD1306_WHITE);
		display.println("TASK1 IS RUNNING");
		Serial.println(F("\nTASK1 IS RUNNING"));
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
		}
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
}

void vTask2(void *pvParameters) {
	const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
	char *Task_Name;
	long lReceivedValue;

	Porfiles p_selected;
	portBASE_TYPE xStatus;
	portBASE_TYPE xStatus2;


	for (;;) {
		display.clearDisplay();
		display.setCursor(0, 0);
		display.setTextColor(0xFFFF, 0);
		display.setTextSize(1);
		display.setTextColor(SSD1306_WHITE);
		display.println("TASK2 IS RUNNING");
		Serial.println("TASK2 IS RUNNING");

		xStatus = xQueueReceive(xQueue, &lReceivedValue, 0);

		if (xStatus == pdPASS) {
			int var = 0;
			float t = dht.readTemperature();

			/* Data was successfully received from the queue, print out the received
			 value. */
			Serial.print("\nxQueue data received: ");
			Serial.println(lReceivedValue);

			for (int i = 0; i < tag_count; i++) {
				if (lReceivedValue == my_porfiles[i].number) {
					Serial.print("\nPorfile with tag ");
					Serial.print(my_porfiles[i].tag_uid);
					Serial.print("selected");
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

				}
				p_selected=my_porfiles[i];
				xStatus2 = xQueueSendToBack(xQueue2, &p_selected, 0);
			}

		}
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}

}

void vTask3(void *pvParameters) {
	float t = dht.readTemperature();
	float temp_diff=0;
	Porfiles p;
	portBASE_TYPE xStatus2;
	portBASE_TYPE xStatus3;
	portBASE_TYPE xStatus4;
	Serial.print("Temperature: ");
	Serial.print(t);

	for (;;) {
		xStatus2 = xQueueReceive(xQueue2, &p, 0);
		float t = dht.readTemperature();

		if(t!=NAN){

		if(t>p.temperature[1])
		temp_diff=t-p.temperature[1];
		else if(t<p.temperature[0])
		temp_diff=t-p.temperature[0];
		else
		temp_diff=0;
}
		xStatus3 = xQueueSendToBack(xQueue3, &temp_diff, 0);

	}
	vTaskDelay(2000/ portTICK_PERIOD_MS);
}
void vTask4(void *pvParameters) {
	for (;;) {
	}

}
void vTask5(void *pvParameters) {
	portBASE_TYPE xStatus2;
	Porfiles p;
	display.clearDisplay();

	display.setCursor(0, 10);
		display.print("Porfile:");
		display.setCursor(0, 20);
		display.print("Temperature:");
		display.setCursor(70, 10);
		display.print("Speed:");
		display.display();


	for (;;) {
		display.setCursor(0, 19);
		display.print("N:");
		display.setCursor(0, 32);
		display.print("T");
		display.setCursor(70, 19);
		xStatus2 = xQueueReceive(xQueue2, &p, 0);
		display.print(xStatus2);
		display.display();

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

void fan_speed_percentage(void) {
	 unsigned int rpm;
	 unsigned long timeold;
unsigned int percentage=0;
	   Serial.begin(9600);
	   attachInterrupt(0,rpm_fun, RISING);// input pin
	   half_revolutions = 0;
	   rpm = 0;
	   timeold = 0;

	 for(;;)
	 {
		 half_revolutions++;
	   if (half_revolutions >= 20) {
	     //Update RPM every 20 counts, increase this for better RPM resolution,
	     //decrease for faster update
	     rpm = 30*1000/(millis() - timeold)*half_revolutions;
	     timeold = millis();
	     half_revolutions = 0;

	     Serial.println(rpm,DEC);
	     percentage=(rpm,DEC)/10;
	   }
	 }

	//-----------------------------------------------

}
void rpm_fun(void)
	 {

	 half_revolutions++;
	   //Each rotation, this interrupt function is run twice
	 }
