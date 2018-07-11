#include <Arduino.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "./BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif


// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

#include <LiquidCrystal.h>

int analogpin = 3;

int val = 0;

// Create the bluefruit SoftwareSerial port

//SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
//
//Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
//	BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);


// A small helper
void error(const __FlashStringHelper*err) {
	Serial.println(err);
	while (1);
}

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void setupSensor()
{

	//Set the magnetometer sensitivity
	lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

}




void setup() {
	// set up the LCD's number of columns and rows:
	lcd.begin(16, 2);
	// Print a message to the LCD.
	lcd.print("hello, world!");

	pinMode(7, INPUT);

	Serial.begin(115200);
	Serial.println(F("Adafruit Bluefruit Command Mode Example"));
	Serial.println(F("---------------------------------------"));

	/* Initialise the module */
	Serial.print(F("Initialising the Bluefruit LE module: "));

	if (!ble.begin(VERBOSE_MODE))
	{
		error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
	}
	Serial.println(F("OK!"));

	

	/* Disable command echo from Bluefruit */
	ble.echo(false);

	//Serial.println("Requesting Bluefruit info:");
	///* Print Bluefruit information */
	//ble.info();

	//Set name of the BT device
	ble.sendCommandCheckOK("AT+GAPDEVNAME=dogtrainer");
	Serial.println("Device name set");

	ble.verbose(false);  // debug info is a little annoying after this point!

						 /* Wait for connection */
	while (!ble.isConnected()) {
		delay(500);
	}

	// LED Activity command is only supported from 0.6.6
	if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION))
	{
		// Change Mode LED Activity
		Serial.println(F("******************************"));
		Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
		ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
		Serial.println(F("******************************"));
	
	}

	lsm.begin();
	setupSensor();

}


void loop() {

	int distsensor, i;
	long anVolt, mm;

	anVolt = 0;

	anVolt += analogRead(3);

	mm = anVolt * 5; //Takes bit count and converts it to mm


	delay(100);
	

	lcd.setCursor(0, 0);                
	lcd.print("Value: ");               
	lcd.setCursor(8, 0);
	lcd.print("    ");
	lcd.setCursor(8, 0);
	lcd.print(mm);
	Serial.println(mm);

	delay(500);                        // Read every half second



	lsm.read();

	/* Get a new sensor event */
	sensors_event_t a, m, g, temp;

	lsm.getEvent(&a, &g, &m, &temp);

	Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
	Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
	Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

	Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
	Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
	Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

	Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
	Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
	Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");

	Serial.println();
	delay(2000);


									   // Check for user input
	char inputs[BUFSIZE + 1];

		ble.print("AT+BLEUARTTX=");
		ble.println(mm);
		ble.println("\n\r");
		

		// check response status
		if (!ble.waitForOK()) {
			Serial.println(F("Failed to send?"));
		}
	//}

	// Check for incoming characters from Bluefruit
	ble.println("AT+BLEUARTRX");
	ble.readline();
	if (strcmp(ble.buffer, "OK") == 0) {
		// no data
		return;
	}
	// Some data was found, its in the buffer
	Serial.print(F("[Recv] ")); Serial.println(ble.buffer);
	ble.waitForOK();
}

