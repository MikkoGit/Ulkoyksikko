// Sample RFM69 sender/node sketch, with ACK and optional encryption, and Automatic Transmission Control
// Sends periodic messages of increasing length to gateway (id=1)
// It also looks for an onboard FLASH chip, if present
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <Sodaq_SHT2x.h>
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include <FaBoLCD_PCF8574.h>
#include <Sodaq_SHT2x.h>
#include <Sodaq_BMP085.h>
#include <lowpower.h>
#include "../../Yhteiset.h"
//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NODEID        ULKOYKSIKKO    //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID     MYNETWORKID  //the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID     1
#define FREQUENCY   RF69_433MHZ
#define ENCRYPTKEY    MYENCRYPTKEY //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level (ATC_RSSI)
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
//*********************************************************************************************

#define LED           9 // Moteinos have LEDs on D9
#define FLASH_SS      8 // and FLASH SS on D8
#define PHOTOCELLPIN	7
#define SERIAL_BAUD   115200

int TRANSMITPERIOD = 500; //transmit a packet to gateway so often (in ms)
char payload[] = "123MABCDEFGHIJKLMNOPQRSTUVWXYZ";
char buff[20];

boolean requestACK = true;
int sendSize = 0;
								  // initialize the library
FaBoLCD_PCF8574 lcd(0x27);
SHT2xClass Mittari;
Sodaq_BMP085 Baro;

#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif

void setup() {
	Serial.begin(SERIAL_BAUD);
	radio.initialize(FREQUENCY, NODEID, NETWORKID);
#ifdef IS_RFM69HW
	radio.setHighPower(); //uncomment only for RFM69HW!
#endif
	radio.encrypt(ENCRYPTKEY);
	//radio.setFrequency(919000000); //set frequency to some custom frequency

	//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
	//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
	//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
	//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
	radio.enableAutoPower(ATC_RSSI);
#endif

	char buff[50];
	sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
	Serial.println(buff);


#ifdef ENABLE_ATC
	Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif
	radio.promiscuous(false);

	// set up the LCD's number of columns and rows:
	lcd.begin(16, 2);
	// Print a message to the LCD.
	lcd.print("Ulkoyksikko Up");
	Baro.begin();

}
void Blink(byte PIN, int DELAY_MS)
{
	pinMode(PIN, OUTPUT);
	digitalWrite(PIN, HIGH);
	delay(DELAY_MS);
	digitalWrite(PIN, LOW);
}


int SendTemphum(float temp, float hum)
{
	payload[0] = TEMPHUM;
	payload[1] = ':';
	dtostrf(temp, 6, 2, (payload + 2));
	payload[8] = ':';
	dtostrf(hum, 6, 2, (payload + 9));
	return radio.sendWithRetry(MASTER, payload, 15/*sendSize*/,5,100);
}
int SendBaro_Alt(uint32_t baro, float alt)
{
	payload[0] = BARO_ALT;
	payload[1] = ':';
	sprintf((payload + 2), "%5lu", baro);
	payload[7] = ':';
	dtostrf(alt, 6, 2, (payload + 8));

	return radio.sendWithRetry(MASTER, payload, 14/*sendSize*/,5,100);
}
int SendLuminess(int luminess)
{
	payload[0] = LUMINESS;
	payload[1] = ':';
	sprintf((payload + 2), "%5i", luminess);

	return radio.sendWithRetry(MASTER, payload, 5/*sendSize*/,5,100);
}




long lastPeriod = 0;
float temp, hum, dew;
char temp_c[10];
float altitude, temp_baro;
uint32_t pressure;
int Photocell_reading = 0;


void loop() {
	//process any serial input
	if (Serial.available() > 0)
	{
		char input = Serial.read();
	}

	//check for any received packets
	if (radio.receiveDone())
	{
		Serial.print('['); Serial.print(radio.SENDERID, DEC); Serial.print("] ");
		for (byte i = 0; i < radio.DATALEN; i++)
			Serial.print((char)radio.DATA[i]);
		Serial.print("   [RX_RSSI:"); Serial.print(radio.RSSI); Serial.print("]");
		lcd.clear();
		lcd.print("RX_RSSI:"); lcd.print(radio.RSSI);

		if (radio.ACKRequested())
		{
			radio.sendACK();
		}
		Blink(LED, 3);
		Serial.println();
	}

	int currPeriod = millis() / TRANSMITPERIOD;
	if (currPeriod != lastPeriod)
	{
		lastPeriod = currPeriod;
		temp = Mittari.GetTemperature();
		hum = Mittari.GetHumidity();
		dew = Mittari.GetDewPoint();
		altitude = Baro.readAltitude();
		pressure = Baro.readPressure();
		temp_baro = Baro.readTemperature();
		lcd.clear();
		sprintf(temp_c, "%lu", pressure);
		lcd.setCursor(0, 0);
		lcd.print(temp_c);

		dtostrf(altitude, 6, 1, temp_c);
		lcd.setCursor(5, 0);
		lcd.print(temp_c);
		dtostrf(temp_baro, 5, 2, temp_c);
		lcd.setCursor(12, 0);
		lcd.print(temp_c);



		dtostrf(temp, 5, 2, temp_c);
		lcd.setCursor(0, 1);
		lcd.print(temp_c);

		Photocell_reading = analogRead(PHOTOCELLPIN);
		sprintf(temp_c, "%i", Photocell_reading);
		lcd.setCursor(7, 1);
		lcd.print(temp_c);


		dtostrf(hum, 7, 2, temp_c);
		lcd.setCursor(9, 1);
		lcd.print(temp_c);
		
		delay(100);
		if(!SendTemphum(temp, hum))
			Serial.print("TempSend Failed\n");
		Blink(LED, 3);
		for (byte i = 0; i < 15; i++)
			Serial.print((char)payload[i]);
		Serial.print("  -->TempHum\n");

		delay(100);
		if (!SendBaro_Alt(pressure, altitude))
			Serial.print("BaroSend Failed\n");
		Blink(LED, 3);

		for (byte i = 0; i < 14; i++)
			Serial.print((char)payload[i]);
		Serial.print("  -->BaroAlt\n");

		delay(100);
		if(!SendLuminess(Photocell_reading))
			Serial.print("LumiSend Failed\n");
		Blink(LED, 3);
		for (byte i = 0; i < 5; i++)
			Serial.print((char)payload[i]);
		Serial.print("  -->Luminess\n");


	}
}


