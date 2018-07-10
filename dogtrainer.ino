#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "./BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"


#include <LiquidCrystal.h>

int analogpin = 3;

int val = 0;

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
	BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
	Serial.println(err);
	while (1);
}

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

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

	if (FACTORYRESET_ENABLE)
	{
		/* Perform a factory reset to make sure everything is in a known state */
		Serial.println(F("Performing a factory reset: "));
		if (!ble.factoryReset()) {
			error(F("Couldn't factory reset"));
		}
	}

	/* Disable command echo from Bluefruit */
	ble.echo(false);

	Serial.println("Requesting Bluefruit info:");
	/* Print Bluefruit information */
	ble.info();

	Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
	Serial.println(F("Then Enter characters to send to Bluefruit"));
	Serial.println();

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
}

void loop() {

	int distsensor, i;
	long anVolt, mm;

	anVolt = 0;

	anVolt += analogRead(5);

	mm = anVolt * 5; //Takes bit count and converts it to mm


	delay(100);






	lcd.setCursor(0, 0);                // write data to LCD display via I2C backpack
	lcd.print("Value: ");               // write to LCD
	lcd.setCursor(8, 0);
	lcd.print("    ");
	lcd.setCursor(8, 0);
	lcd.print(mm);
	ble.print("Value");
	//ble.print("AT+BLEUARTTX=");
	//ble.println(mm);

	delay(500);                        // Read every half second

									   // Check for user input
	char inputs[BUFSIZE + 1];

	if (getUserInput(inputs, BUFSIZE))
	{
		// Send characters to Bluefruit
		Serial.print("[Send] ");
		Serial.println(inputs);

		ble.print("AT+BLEUARTTX=");
		ble.println(inputs);

		// check response stastus
		if (!ble.waitForOK()) {
			Serial.println(F("Failed to send?"));
		}
	}

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

/**************************************************************************/
/*!
@brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
bool getUserInput(char buffer[], uint8_t maxSize)
{
	// timeout in 100 milliseconds
	TimeoutTimer timeout(100);

	memset(buffer, 0, maxSize);
	while ((!Serial.available()) && !timeout.expired()) { delay(1); }

	if (timeout.expired()) return false;

	delay(2);
	uint8_t count = 0;
	do
	{
		count += Serial.readBytes(buffer + count, maxSize);
		delay(2);
	} while ((count < maxSize) && (Serial.available()));

	return true;
}
