/**
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2015 Sensnology AB
* Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation..
*
*******************************
*
* REVISION HISTORY
* Version 1.0 - Mikael Ljunggren
*
* DESCRIPTION
* Sensor with double temp sensors, one HTU21D combined temp with humidity and one DS18B20 for extrenal measurement 
* HTU21D values are shown in one device and DS18B20 in another.
*
*/

// Enable debug prints to serial monitor
//#define MY_DEBUG 

// Define a static node address, remove if you want auto address assignment
//#deine MY_NODE_ID 3

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#include <SPI.h>
#include <MySensors.h>
#include <Wire.h>
#include <Adafruit_HTU21DF_Library\Adafruit_HTU21DF.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define ONE_WIRE_BUS 3 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 16
#define COMPARE_TEMP 0 // Send temperature only if changed? 1 = Yes 0 = No

#define RELEASE "1.4"

// Child sensor ID's
#define CHILD_ID_TEMPDS18B20  0
#define CHILD_ID_TEMP  1
#define CHILD_ID_HUM   2

// How many milli seconds between each measurement
#define MEASURE_INTERVAL 60000

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
Adafruit_HTU21DF humiditySensor;

// Sensor messages
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

// Initialize temperature message
MyMessage msg(CHILD_ID_TEMPDS18B20, V_TEMP);

// Global settings
int numSensors = 0;
bool receivedConfig = false;
int sendBattery = 0;
float lastTemperatureDS18B20;

boolean transmission_occured = false;

// Storage of old measurements
float lastTemperature = -100;
int lastHumidity = -100;
long lastBattery = -100;

void before()
{
	// Startup up the OneWire library
	sensors.begin();
}

/****************************************************
*
* Setup code
*
****************************************************/
void setup() {

	Serial.begin(9600);
	Serial.print(F("Sensebender Micro FW "));
	Serial.print(RELEASE);
	Serial.flush();

	humiditySensor.begin();
	sensors.setWaitForConversion(false);

	Serial.flush();
	Serial.println(F(" - Online!"));

	sendTempHumidityMeasurements();
	sendTemp();

	//sendBattLevel(false);

}

void presentation() {
	sendSketchInfo("TempHum", RELEASE);

	present(CHILD_ID_TEMPDS18B20, S_TEMP);
	present(CHILD_ID_TEMP, S_TEMP);
	present(CHILD_ID_HUM, S_HUM);

}

/***********************************************
*
*  Main loop function
*
***********************************************/
void loop() {

	sendTempHumidityMeasurements();
	sendTemp();
	sendBattLevel();

	sleep(MEASURE_INTERVAL);
}

/*********************************************
*
* Sends temperature and humidity from DS18B20 sensor
*
*********************************************/
void sendTemp()
{
	// Fetch temperatures from Dallas sensors
	sensors.requestTemperatures();

	// query conversion time and sleep until conversion completed
	int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
	// sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
	sleep(conversionTime);

	// Fetch and round temperature to one decimal
	float temperature = sensors.getTempCByIndex(0);
		
	Serial.print("T2: ");Serial.println(temperature);
	// Send in the new temperature
	send(msg.set(temperature, 1));
	// Save new temperatures for next compare
	lastTemperatureDS18B20 = temperature;

}

/*********************************************
*
* Sends temperature and humidity from Si7021 sensor
*
*********************************************/
void sendTempHumidityMeasurements()
{

	float humd = humiditySensor.readHumidity();
	float temp = humiditySensor.readTemperature();

		Serial.print("T1: ");Serial.println(temp);
		Serial.print("H1: ");Serial.println(humd);

		send(msgTemp.set(temp, 1));
		send(msgHum.set(humd,1));

		lastTemperature = temp;
		lastHumidity = humd;
		transmission_occured = true;

	}

/********************************************
*
* Sends battery information (battery percentage)
*
*******************************************/
void sendBattLevel()
{
	long vcc = readVcc();
	Serial.print("VCC: ");Serial.println(vcc);
	if (vcc != lastBattery) {
		lastBattery = vcc;

		// Calculate percentage

		vcc = vcc - 1900; // subtract 1.9V from vcc, as this is the lowest voltage we will operate at

		long percent = vcc / 14.0;
		sendBatteryLevel(percent);
		transmission_occured = true;
	}
}

/*******************************************
*
* Internal battery ADC measuring
*
*******************************************/
long readVcc() {
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADcdMUX = _BV(MUX3) | _BV(MUX2);
#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA, ADSC)); // measuring

	uint8_t low = ADCL; // must read ADCL first - it then locks ADCH  
	uint8_t high = ADCH; // unlocks both

	long result = (high << 8) | low;

	result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	return result; // Vcc in millivolts

}

