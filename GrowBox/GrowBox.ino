/******************************************************************
  Created with Arduino - 09.12.2015 14:02:04
  Project      : GrowBox v0.7.7 (release.firmware.web)
  Author       : Michele Valentini - Red Sheep Labs
  Description  : GrowBox Controller with EmonCMS interaction
  Website	   : www.yarosia.it
  Email		   : valentini.michele@gmail.com
  Copyright (C): 2015-2018 Michele Valentini (www.yarosia.it - Red Sheep Labs)
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
******************************************************************/


//RobTillartDHTlib or AdafruitDHTLib
#define AdafruitDHTLib
//debug or release_version
#define debug

#include <DallasTemperature.h>
#include <DS3232RTC.h>
#include <Streaming.h>
#include <Time.h>
#include <TimeLib.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#ifdef AdafruitDHTLib
#include <DHT.h>
#endif

#ifdef RobTillartDHTlib
#include <dht.h>
#endif

#include <Ethernet2.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <string.h>

#ifdef AdafruitDHTLib
#define TEMPHUM DHT22
#endif

#define RELAYON  0
#define RELAYOFF 1
#define REQ_BUFF_SZ 120

#define LCD_SENSORS 0
#define LCD_NETWORK 1
#define LCD_RELAYS 2
#define LCD_INIT 3

//Increase this if EEPROM changes are made
#define EEPROM_VERSION 5

#define LIGHTS 0
#define HEATER 1
#define FAN 2
#define MIST 3

const int ethCS = 53;
const int sdCS = 4;
const int internalTHpin_PIN = 29;
const int externalTHpin_PIN = 27;
const int internalTEMP_PIN = 25;
const int sampleTEMP_PIN = 23;

const int pinDisable = 10;

/** Relays **/
const int relay_Lights1 = 22;
const int relay_Heater1 = 24;
const int relay_Fan1 = 26;
const int relay_Water1 = 28;
const int relay_Lights2 = 30;
const int relay_Heater2 = 32;
const int relay_Fan2 = 34;
const int relay_Water2 = 36;

/** EEPROM Addresses **/
const int addFirstRun = 0;
const int addLightOnHour = 1;
const int addLightOnMinute = 2;
const int addLightOffHour = 3;
const int addLightOffMinute = 4;
const int addMinIntTemp = 5;
const int addMaxIntTemp = 9;
const int addMinIntHum = 13;
const int addMaxIntHum = 17;
const int addTransmissionRate = 21;
const int addSampleRate = 25;
const int addFanIntervalNight = 29;
const int addFanDurationNight = 30;
const int addIP1 = 31;
const int addIP2 = 32;
const int addIP3 = 33;
const int addIP4 = 34;
const int addDNS1 = 35;
const int addDNS2 = 36;
const int addDNS3 = 37;
const int addDNS4 = 38;
const int addSNM1 = 39;
const int addSNM2 = 40;
const int addSNM3 = 41;
const int addSNM4 = 42;
const int addGW1 = 43;
const int addGW2 = 44;
const int addGW3 = 45;
const int addGW4 = 46;
const int addDHCPFlag = 47;
const int addFanIntervalDay = 48;
const int addFanDurationDay = 49;
const int addFanAlwaysOnDay = 50;
const int addFanAlwaysOnNight = 51;
const int addMinGroundT = 52;
const int addMaxGroundT = 53;

/** Network Addresses **/
int IP1 = 192;
int IP2 = 168;
int IP3 = 1;

#ifdef release_version
int IP4 = 69;
#endif

#ifdef debug
int IP4 = 140;
#endif

int GW1 = 192;
int GW2 = 168;
int GW3 = 1;
int GW4 = 1;
int SNM1 = 255;
int SNM2 = 255;
int SNM3 = 255;
int SNM4 = 0;
int DNS1 = 8;
int DNS2 = 8;
int DNS3 = 8;
int DNS4 = 8;
boolean DHCPFlag = false;

unsigned long int lastFanStart = 0;
unsigned long int transmissionRate = 300000; //Value in Minutes
unsigned long int sampleRate = 10000; //Value in Seconds
unsigned long int lastPost = 0;
unsigned long int lastUpdate = 0;



#ifdef AdafruitDHTLib
DHT dht_internalTH(internalTHpin_PIN, TEMPHUM);
DHT dht_externalTH(externalTHpin_PIN, TEMPHUM);
#endif

#ifdef RobTillartDHTlib
dht DHT;
#endif

OneWire ow_internalT(internalTEMP_PIN);
OneWire ow_sampleT(sampleTEMP_PIN);
DallasTemperature ds_internalT(&ow_internalT);
DallasTemperature ds_sampleT(&ow_sampleT);
DeviceAddress ds_internalT_add;
DeviceAddress ds_sampleT_add;

float internalT = 0;
float internalH = 0;
float externalT = 0;
float externalH = 0;
float internalT2 = 0;
float sampleT = 0;

const char internalTSerial[] = "internal_t";
const char internalHSerial[] = "internal_h";
const char externalTSerial[] = "external_t";
const char externalHSerial[] = "external_h";
const char internalT2Serial[] = "internal_t_top";
const char sampleTSerial[] = "ground_t";


float minIntTemp = 15;
float maxIntTemp = 30;

float minIntHum = 50;
float maxIntHum = 80;

uint8_t minGroundT = 30;
uint8_t maxGroundT = 35;

uint8_t lightOnHour = 8;
uint8_t lightOnMinute = 30;
uint8_t lightOffHour = 20;
uint8_t lightOffMinute = 30;

uint8_t fanDurationDay = 10; //Value in Minutes
uint8_t fanIntervalDay = 6; //Value in Hours

uint8_t fanDurationNight = 10;
uint8_t fanIntervalNight = 6;

uint8_t fanAlwaysOnDay = false;
uint8_t fanAlwaysOnNight = false;

boolean fanOn = false;
unsigned long fanStartTime;

File logDir;
File currentLog;
char logDirName[] = "/logs/";
String currentLogFileName;

File webFile;
char HTTP_req[REQ_BUFF_SZ] = {0};
char req_index = 0;
boolean RELAY_state[4] = {0, 0, 0, 0};

const String index = "index.htm";
const String download = "download.htm";
const String css = "style.css";
const String settings = "settings.htm";

#ifdef debug
LiquidCrystal_I2C lcd(0x27, 20, 4);
#endif

#ifdef release_version
/** LCD display, address 0x3F, 20x4 **/
LiquidCrystal_I2C lcd(0x3F, 20, 4);
#endif

int lcdPage = LCD_INIT;
unsigned long int lastPageChange = 0;
unsigned long int lcdPageChange = 10000;
boolean display_setup = false;

/** Cloud EmonCMS config **/
const char serverEmon[] = "www.emoncms.org";		//Your EmonCMS location es. www.emoncms.org
const String apikey = "abcdefghijklmnopqrstuvwxyz123456";	//Your EmonCMS APIKEY

#ifdef debug
//NODE TEST - sends data to EmonCMS as node 2
int node = 2;
#endif

#ifdef release_version
//NODE GROWBOX - sends data to EmonCMS as node 1
int node = 1;
#endif

/** Network configuration **/
#ifdef debug
//IP TEST
byte mac[6] = {0x90, 0xa2, 0xda, 0x10, 0x76, 0xd9};
#endif

#ifdef release_version
//IP GROWBOX
byte mac[6] = {0x90, 0xa2, 0xda, 0x10, 0x76, 0xc8};
#endif

IPAddress subnet (255, 255, 255, 0);

int port = 80;
EthernetServer server(port);
EthernetClient client;

/** Utility **/
const byte degree[8] = { //  CHARACTER "°C" DEFINITION
  B10111,
  B01000,
  B10000,
  B10000,
  B10000,
  B01000,
  B00111,
};

String addZero(uint16_t data) {
  if (data < 10) {
    return "0" + (String)data;
  } else {
    return (String)data;
  }
}

char* intToDay(uint8_t day) {
  switch (day) {
    case 1:
      return "Sunday";
      break;
    case 2:
      return "Monday";
      break;
    case 3:
      return "Tuesday";
      break;
    case 4:
      return "Wednesday";
      break;
    case 5:
      return "Thursday";
      break;
    case 6:
      return "Friday";
      break;
    case 7:
      return "Saturday";
      break;
    default:
      return "";
      break;
  }
}

/** EEPROM **/
void initializeParameters() {
  // On first run writes default parameters to EEPROM
  int PREVIOUS_EEPROM_VERSION = EEPROM.read(addFirstRun);
  if (PREVIOUS_EEPROM_VERSION < EEPROM_VERSION || PREVIOUS_EEPROM_VERSION == 255) {
  
#ifdef debug
    Serial.println(F("Write EEPROM with default parameters"));
#endif

    EEPROM.put(addFirstRun, EEPROM_VERSION);
    EEPROM.put(addLightOnHour, lightOnHour);
    EEPROM.put(addLightOnMinute, lightOnMinute);
    EEPROM.put(addLightOffHour, lightOffHour);
    EEPROM.put(addLightOffMinute, lightOffMinute);
    EEPROM.put(addMinIntTemp, minIntTemp);
    EEPROM.put(addMaxIntTemp, maxIntTemp);
    EEPROM.put(addMinIntHum, minIntHum);
    EEPROM.put(addMaxIntHum, maxIntHum);
    EEPROM.put(addTransmissionRate, transmissionRate);
    EEPROM.put(addSampleRate, sampleRate);
    EEPROM.put(addFanIntervalNight, fanIntervalNight);
    EEPROM.put(addFanDurationNight, fanDurationNight);
    EEPROM.put(addFanIntervalDay, fanIntervalDay);
    EEPROM.put(addFanDurationDay, fanDurationDay);
    EEPROM.put(addFanAlwaysOnDay, fanAlwaysOnDay);
    EEPROM.put(addFanAlwaysOnNight, fanAlwaysOnNight);
    EEPROM.put(addIP1, IP1);
    EEPROM.put(addIP2, IP2);
    EEPROM.put(addIP3, IP3);
    EEPROM.put(addIP4, IP4);
    EEPROM.put(addGW1, GW1);
    EEPROM.put(addGW2, GW2);
    EEPROM.put(addGW3, GW3);
    EEPROM.put(addGW4, GW4);
    EEPROM.put(addSNM1, SNM1);
    EEPROM.put(addSNM2, SNM2);
    EEPROM.put(addSNM3, SNM3);
    EEPROM.put(addSNM4, SNM4);
    EEPROM.put(addDNS1, DNS1);
    EEPROM.put(addDNS2, DNS2);
    EEPROM.put(addDNS3, DNS3);
    EEPROM.put(addDNS4, DNS4);
    EEPROM.put(addDHCPFlag, DHCPFlag);
    EEPROM.put(addMinGroundT, minGroundT);
    EEPROM.put(addMaxGroundT, maxGroundT);
  }
  
#ifdef debug
  Serial.println(F("Reading EEPROM"));
#endif

  EEPROM.get(addLightOnHour, lightOnHour);
  EEPROM.get(addLightOnMinute, lightOnMinute);
  EEPROM.get(addLightOffHour, lightOffHour);
  EEPROM.get(addLightOffMinute, lightOffMinute);
  EEPROM.get(addMinIntTemp, minIntTemp);
  EEPROM.get(addMaxIntTemp, maxIntTemp);
  EEPROM.get(addMinIntHum, minIntHum);
  EEPROM.get(addMaxIntHum, maxIntHum);
  EEPROM.get(addTransmissionRate, transmissionRate);
  EEPROM.get(addSampleRate, sampleRate);
  EEPROM.get(addFanIntervalNight, fanIntervalNight);
  EEPROM.get(addFanDurationNight, fanDurationNight);
  EEPROM.get(addFanIntervalDay, fanIntervalDay);
  EEPROM.get(addFanDurationDay, fanDurationDay);
  EEPROM.get(addFanAlwaysOnDay, fanAlwaysOnDay);
  EEPROM.get(addFanAlwaysOnNight, fanAlwaysOnNight);
  EEPROM.get(addIP1, IP1);
  EEPROM.get(addIP2, IP2);
  EEPROM.get(addIP3, IP3);
  EEPROM.get(addIP4, IP4);
  EEPROM.get(addGW1, GW1);
  EEPROM.get(addGW2, GW2);
  EEPROM.get(addGW3, GW3);
  EEPROM.get(addGW4, GW4);
  EEPROM.get(addSNM1, SNM1);
  EEPROM.get(addSNM2, SNM2);
  EEPROM.get(addSNM3, SNM3);
  EEPROM.get(addSNM4, SNM4);
  EEPROM.get(addDNS1, DNS1);
  EEPROM.get(addDNS2, DNS2);
  EEPROM.get(addDNS3, DNS3);
  EEPROM.get(addDNS4, DNS4);
  EEPROM.get(addDHCPFlag, DHCPFlag);
  EEPROM.get(addMinGroundT, minGroundT);
  EEPROM.get(addMaxGroundT, maxGroundT);
  
#ifdef debug
  Serial.println("First Run: " + (String) EEPROM.read(addFirstRun));
  Serial.println("Light On Hour: " + (String) lightOnHour);
  Serial.println("Light On Minute: " + (String) lightOnMinute);
  Serial.println("Light Off Hour: " + (String) lightOffHour);
  Serial.println("Light Off Minute: " + (String) lightOffMinute);
  Serial.println("Min Temp: " + (String) minIntTemp);
  Serial.println("Max Temp: " + (String) maxIntTemp);
  Serial.println("Min Hum: " + (String) minIntHum);
  Serial.println("Max Hum: " + (String) maxIntHum);
  Serial.println("Sample Rate:" + (String)sampleRate);
  Serial.println("Transmission Rate: " + (String)transmissionRate);
  Serial.println("Fan Interval Day: " + (String)fanIntervalDay);
  Serial.println("Fan Duration Day: " + (String)fanDurationDay);
  Serial.println("Fan Interval Night: " + (String)fanIntervalNight);
  Serial.println("Fan Duration Night: " + (String)fanDurationNight);
  Serial.println("Fan Always On Day: " + (String)fanAlwaysOnDay);
  Serial.println("Fan Always On Night: " + (String)fanAlwaysOnNight);
#endif

}

void setIPAddress(int ip1, int ip2, int ip3, int ip4) {
  IP1 = ip1;
  IP2 = ip2;
  IP3 = ip3;
  IP4 = ip4;
  EEPROM.put(addIP1, IP1);
  EEPROM.put(addIP2, IP2);
  EEPROM.put(addIP3, IP3);
  EEPROM.put(addIP4, IP4);
}

void setSNMAddress(int snm1, int snm2, int snm3, int snm4) {
  SNM1 = snm1;
  SNM2 = snm2;
  SNM3 = snm3;
  SNM4 = snm4;
  EEPROM.put(addSNM1, SNM1);
  EEPROM.put(addSNM2, SNM2);
  EEPROM.put(addSNM3, SNM3);
  EEPROM.put(addSNM4, SNM4);
}

void setGWAddress(int gw1, int gw2, int gw3, int gw4) {
  GW1 = gw1;
  GW2 = gw2;
  GW3 = gw3;
  GW4 = gw4;
  EEPROM.put(addGW1, GW1);
  EEPROM.put(addGW2, GW2);
  EEPROM.put(addGW3, GW3);
  EEPROM.put(addGW4, GW4);
}

void setDNSAddress(int dns1, int dns2, int dns3, int dns4) {
  DNS1 = dns1;
  DNS2 = dns2;
  DNS3 = dns3;
  DNS4 = dns4;
  EEPROM.put(addDNS1, DNS1);
  EEPROM.put(addDNS2, DNS2);
  EEPROM.put(addDNS3, DNS3);
  EEPROM.put(addDNS4, DNS4);
}

void setDHCP(boolean flag) {
  DHCPFlag = flag;
  EEPROM.put(addDHCPFlag, DHCPFlag);
}

void setSampleRate(unsigned long sampleRateI) {
  sampleRate = sampleRateI;
  EEPROM.put(addSampleRate, sampleRate);
}

void setTransmissionRate(unsigned long transmissionRateI) {
  transmissionRate = transmissionRateI;
  EEPROM.put(addTransmissionRate, transmissionRate);
}

void setFanDurationNight(uint8_t fanDurationI) {
  fanDurationNight = fanDurationI;
  EEPROM.put(addFanDurationNight, fanDurationNight);
}

void setFanIntervalNight(uint8_t fanIntervalI) {
  fanIntervalNight = fanIntervalI;
  EEPROM.put(addFanIntervalNight, fanIntervalNight);
}

void setFanDurationDay(uint8_t fanDurationI) {
  fanDurationDay = fanDurationI;
  EEPROM.put(addFanDurationDay, fanDurationDay);
}

void setFanIntervalDay(uint8_t fanIntervalI) {
  fanIntervalDay = fanIntervalI;
  EEPROM.put(addFanIntervalDay, fanIntervalDay);
}

void setFansAlwaysONNight(boolean fanAlwaysOnNightI) {
  fanAlwaysOnNight = fanAlwaysOnNightI;
  EEPROM.put(addFanAlwaysOnNight, fanAlwaysOnNight);
}

void setFansAlwaysONDay(boolean fanAlwaysOnDayI) {
  fanAlwaysOnDay = fanAlwaysOnDayI;
  EEPROM.put(addFanAlwaysOnDay, fanAlwaysOnDay);
}

void setLightOnHour(uint8_t onHour) {
  lightOnHour = onHour;
  EEPROM.put(addLightOnHour, lightOnHour);

}

void setLightOnMinute(uint8_t onMinute) {
  lightOnMinute = onMinute;
  EEPROM.put(addLightOnMinute, lightOnMinute);

}

void setLightOffHour(uint8_t offHour) {
  lightOffHour = offHour;
  EEPROM.put(addLightOffHour, lightOffHour);

}

void setLightOffMinute(uint8_t offMinute) {
  lightOffMinute = offMinute;
  EEPROM.put(addLightOffMinute, lightOffMinute);
}

void setMinHum(float minHum) {
  minIntHum = minHum;
  EEPROM.put(addMinIntHum, minIntHum);
}

void setMaxHum(float maxHum) {
  maxIntHum = maxHum;
  EEPROM.put(addMaxIntHum, maxIntHum);
}

void setMinTemp(float minTemp) {
  minIntTemp = minTemp;
  EEPROM.put(addMinIntTemp, minIntTemp);
}

void setMaxTemp(float maxTemp) {
  maxIntTemp = maxTemp;
  EEPROM.put(addMaxIntTemp, maxIntTemp);
}

void setMinGroundT(uint8_t minGT) {
  minGroundT = minGT;
  EEPROM.put(addMinGroundT, minGroundT);
}

void setMaxGroundT(uint8_t maxGT) {
  maxGroundT = maxGT;
  EEPROM.put(addMaxGroundT, maxGT);
}

/** LCD **/
void lcdDispatch() {
  unsigned long int ora = millis();
  if (ora > lcdPageChange + lastPageChange) {
    lastPageChange = ora;
    switch (lcdPage) {
      case LCD_INIT:
        printHome();
        break;
      case LCD_SENSORS:
        printStatus();
        break;
      case LCD_RELAYS:
        printRelays();
        break;
      case LCD_NETWORK:
        printNetwork();
        break;
      default:
        break;
    }
  }
}

void printHome() {
  char lcdRow0[21] = "     Welcome  to    ";
  char lcdRow1[21] = "   Arduino Growbox  ";
  char lcdRow2[21] = "     Created By     ";
  char lcdRow3[21] = " Michele  Valentini ";
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(lcdRow0);
  lcd.setCursor(0, 1);
  lcd.print(lcdRow1);
  lcd.setCursor(0, 2);
  lcd.print(lcdRow2);
  lcd.setCursor(0, 3);
  lcd.print(lcdRow3);
  lcd.display();
  lcdPage = LCD_SENSORS;
}

void printStatus() {
  lcd.clear();
  lcd.setCursor(0, 0);
  char  lcdRow0[20] = "";
  tmElements_t tm;
  RTC.read(tm);
  sprintf(lcdRow0, "%02d/%02d/%4d %02d:%02d:%02d", tm.Day, tm.Month, (tm.Year + 1970), tm.Hour, tm.Minute, tm.Second);
  lcd.print(lcdRow0);
  lcd.setCursor(0, 1);
  char intT[6];
  char extT[6];
  char intH[6];
  char extH[6];
  char sampT[6];
  char intT2[6];
  dtostrf(internalT, 6, 2, intT);
  lcd.print("T:");
  lcd.setCursor(2, 1);
  lcd.print(intT);
  dtostrf(internalH, 6, 2, intH);
  lcd.setCursor(9, 1);
  lcd.print("H:");
  lcd.setCursor(11, 1);
  lcd.print(intH);
  lcd.setCursor(0, 2);
  lcd.print("T:");
  lcd.setCursor(2, 2);
  dtostrf(externalT, 6, 2, extT);
  lcd.print(extT);
  lcd.setCursor(9, 2);
  lcd.print("H:");
  lcd.setCursor(11, 2);
  dtostrf(externalH, 6, 2, extH);
  lcd.print(extH);
  lcd.setCursor(0, 3);
  lcd.print("T:");
  lcd.setCursor(2, 3);
  dtostrf(internalT2, 6, 2, intT2);
  lcd.print(intT2);
  lcd.setCursor(9, 3);
  lcd.print("T:");
  lcd.setCursor(11, 3);
  dtostrf(sampleT, 6, 2, sampT);
  lcd.print(sampT);
  lcd.display();
  lcdPage = LCD_RELAYS;
}
void printRelays() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Relays:");
  lcd.setCursor(0, 1);
  lcd.print("Lights: ");
  lcd.setCursor(7, 1);
  if (RELAY_state[LIGHTS] == 1) {
    lcd.print("ON");
  } else {
    lcd.print("OFF");
  }
  lcd.setCursor(11, 1);
  lcd.print("Mist: ");
  lcd.setCursor(16, 1);
  if (RELAY_state[MIST] == 1) {
    lcd.print("ON");
  } else {
    lcd.print("OFF");
  }
  lcd.setCursor(0, 2);
  lcd.print("Heater: ");
  lcd.setCursor(7, 2);
  if (RELAY_state[HEATER] == 1) {
    lcd.print("ON");
  } else {
    lcd.print("OFF");
  }
  lcd.setCursor(11, 2);
  lcd.print("Fan: ");
  lcd.setCursor(16, 2);
  if (RELAY_state[FAN] == 1) {
    lcd.print("ON");
  } else {
    lcd.print("OFF");
  }
  lcd.display();
  lcdPage = LCD_NETWORK;
}

void printNetwork() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Network: ");
  lcd.setCursor(0, 1);
  if (DHCPFlag) {
    lcd.print("DHCP Enabled");
  } else {
    lcd.print("DHCP Disabled");
  }
  char lcdRowIP[21];
  sprintf(lcdRowIP, "IP: %d.%d.%d.%d", Ethernet.localIP()[0], Ethernet.localIP()[1], Ethernet.localIP()[2], Ethernet.localIP()[3]);
  lcd.setCursor(0, 2);
  lcd.print(lcdRowIP);
  lcd.display();
  lcdPage = LCD_SENSORS;

}

/** Init **/
void initializePins() {
  pinMode(ethCS, OUTPUT);
  digitalWrite(ethCS, HIGH);
  pinMode(sdCS, OUTPUT);
  digitalWrite(sdCS, HIGH);
  pinMode(pinDisable, OUTPUT);
  digitalWrite(pinDisable, HIGH);
  //set pin 53 as output - needed on Arduino Mega to make SPI work properly
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  pinMode(50, OUTPUT);
  // digitalWrite(50, HIGH);
  pinMode(51, OUTPUT);
  // digitalWrite(51, HIGH);
  pinMode(52, OUTPUT);
  SPI.begin();
}

void initializeEthShield() {

}

void initializeSDCard() {

#ifdef debug
  Serial.println(F("Initializing SD Card..."));
#endif

  if (!SD.begin(sdCS)) {
  
#ifdef debug
    Serial.println(F("ERROR - SD Card initializations failed!"));
#endif

    return;
  }
  
#ifdef debug
  Serial.println(F("SUCCESS - SD Card initialized!"));
#endif

  if (!SD.exists(index)) {
  
#ifdef debug
    Serial.print(F("ERROR - Can't find "));
    Serial.print(index);
    Serial.println(F(" file!"));
#endif

    return;
  }
  
#ifdef debug
  Serial.print(F("SUCCESS - Found "));
  Serial.print(index);
  Serial.println(F(" file!"));
#endif

  if (!SD.exists(download)) {
  
#ifdef debug
    Serial.print(F("ERROR - Can't find "));
    Serial.print(download);
    Serial.println(F(" file!"));
#endif

    return;
  }
  
#ifdef debug
  Serial.print(F("SUCCESS - Found "));
  Serial.print(download);
  Serial.println(F(" file!"));
#endif

}

void initializeRelays() {
  pinMode(relay_Lights1, OUTPUT);
  digitalWrite(relay_Lights1, RELAYOFF);
  pinMode(relay_Heater1, OUTPUT);
  digitalWrite(relay_Heater1, RELAYOFF);
  pinMode(relay_Fan1, OUTPUT);
  digitalWrite(relay_Fan1, RELAYOFF);
  pinMode(relay_Water1, OUTPUT);
  digitalWrite(relay_Water1, RELAYOFF);
  pinMode(relay_Lights2, OUTPUT);
  digitalWrite(relay_Lights2, RELAYOFF);
  pinMode(relay_Heater2, OUTPUT);
  digitalWrite(relay_Heater2, RELAYOFF);
  pinMode(relay_Fan2, OUTPUT);
  digitalWrite(relay_Fan2, RELAYOFF);
  pinMode(relay_Water2, OUTPUT);
  digitalWrite(relay_Water2, RELAYOFF);
}

void initializeSensors() {
  ds_internalT.begin();
  ds_sampleT.begin();

  ds_internalT.getAddress(ds_internalT_add, 0);
  ds_sampleT.getAddress(ds_sampleT_add, 0);
  ds_internalT.setResolution(ds_internalT_add, 12);
  ds_sampleT.setResolution(ds_sampleT_add, 12);
}

void initializeLCD() {
  lcd.init();
  lcd.backlight();
}

void initializeServer() {
  if (DHCPFlag) {
    Ethernet.begin(mac);
  } else {
    Ethernet.begin(mac, {IP1, IP2, IP3, IP4}, {GW1, GW2, GW3, GW4}, {SNM1, SNM2, SNM3, SNM4});
  }
  server.begin();
  delay(1000);
}

void setup() {
#ifdef debug
  Serial.begin(57600);
#endif
  initializePins();
  initializeParameters();
  initializeEthShield();
  initializeSDCard();
  initializeRelays();
  initializeSensors();
  initializeLCD();
  initializeServer();
  readAllSensors();
  initializeParameters();
  postJSON();
}

void writeToSD() {
  if (SD.exists(logDirName + getFileName())) {
    currentLog = SD.open(logDirName + currentLogFileName, FILE_WRITE);
    if (currentLog) {
    
#ifdef debug
      Serial.println(F("Writing file"));
#endif

      currentLog.println(getSensorString());
      currentLog.close();
    }
  } else {
  
#ifdef debug
    Serial.println(F("File not found, creating..."));
#endif

    SD.mkdir(logDirName);
    currentLog = SD.open(logDirName + currentLogFileName, FILE_WRITE);
    currentLog.println(F("TimeStamp;InternalTemp;InternalHum;ExternalTemp;ExternalHum;InternalTemp2;SampleTemp;Lights;Heater;Fans;Humidifier;"));
    currentLog.close();
  }
}

String getSensorString() {
  String ret;
  ret += getTimeStampForHTML();
  ret += ";";
  ret += (String)internalT + ";";
  ret += (String)internalH + ";";
  ret += (String)externalT + ";";
  ret += (String)externalH  + ";";
  ret += (String)internalT2 + ";";
  ret += (String)sampleT + ";";
  for (int i = 0; i < 4; i++) {
    ret += (String)getState(RELAY_state[i]) + ";";
  }
  return ret;
}
String getState(boolean relay) {
  if (relay) {
    return "ON";
  } else {
    return "OFF";
  }
  return "";
}

/** Sensors Reading **/
void readAllSensors() {
  ds_internalT.requestTemperatures();
  internalT2 = ds_internalT.getTempC(ds_internalT_add);
  ds_sampleT.requestTemperatures();
  sampleT = ds_sampleT.getTempC(ds_sampleT_add);
  
#ifdef AdafruitDHTLib
  internalT = dht_internalTH.readTemperature();
  internalH = dht_internalTH.readHumidity();
  externalT = dht_externalTH.readTemperature();
  externalH = dht_externalTH.readHumidity();
#endif

#ifdef RobTillartDHTlib
  int chk = DHT.read44(internalTHpin_PIN);
#endif

#ifdef RobTillartDHTlib
  internalT = DHT.temperature;
  internalH = DHT.humidity;
  chk = DHT.read22(externalTHpin_PIN);
#endif

#ifdef RobTillartDHTlib
  externalT = DHT.temperature;
  externalH = DHT.humidity;
#endif

}

void updateSensors() {
  unsigned long  int ora = millis();
  if (ora > sampleRate + lastUpdate) {
    lastUpdate = ora;
    readAllSensors();
    //writeToSD();
  }
}

String getTimeStampForWeb() {
  String ret;
  tmElements_t tm;
  RTC.read(tm);
  ret += (addZero(tm.Year + 1970) + "-" + addZero(tm.Month) + "-" + addZero(tm.Day) + " " + addZero(tm.Hour) + ":" + addZero(tm.Minute) + ":" + addZero(tm.Second) );
  return ret;
}

String getTimeStampForHTML() {
  String ret;
  tmElements_t tm;
  RTC.read(tm);
  ret += (addZero(tm.Day)  + "-" + addZero(tm.Month) + "-" + addZero(tm.Year + 1970) +  " " + addZero(tm.Hour) + ":" + addZero(tm.Minute) + ":" + addZero(tm.Second) );
  return ret;
}

char* getFormattedTimeForLCD() {
  char  ret[20] = "";
  tmElements_t tm;
  RTC.read(tm);
  sprintf(ret, "%02d/%02d/%4d %02d:%02d:%02d", tm.Day, tm.Month, (tm.Year + 1970), tm.Hour, tm.Minute, tm.Second);
  return ret;
}

String getFileName() {
  String ret;
  tmElements_t tm;
  RTC.read(tm);
  ret += (addZero(tm.Year + 1970) + addZero(tm.Month) + addZero(tm.Day) + ".csv");
  currentLogFileName = ret;
  
#ifdef debug
  Serial.print(F("Current Log File Name: "));
  Serial.println(currentLogFileName);
#endif

  return ret;
}

/** Ethernet methods **/
void ntpCheck() {
	/** TODO **/
}

void postOnOffEvent(String device, int state) {
  if (client.connect(serverEmon, 80)) {
  
#ifdef debug
    Serial.println(F("Connecting..."));
#endif

    client.print(F("GET /emoncms/input/post.json?"));
    if (node > 0) {
    
#ifdef debug
      Serial.println(F("Node ok"));
#endif

      client.print(F("node="));
      client.print(node);
      client.print(F("&"));
    }
    
    client.print(F("json={"));
    client.print(device);
    client.print(F(":"));
    client.print(state);
    client.print(F("}&apikey="));
    client.print(apikey);
    client.println(F(" HTTP/1.1"));
    client.println(F("Host: www.emoncms.org"));     //CHANGE THIS IF NEEDED
    client.println(F("User-Agent: Arduino-ethernet"));
    client.println(F("Connection: close"));
    client.println();
  }
  
  delay(1);
  client.stop();
  
}

void postJSON() {
  if (client.connect(serverEmon, 80)) {
  
#ifdef debug
    Serial.println(F("Connecting..."));
#endif

    client.print(F("GET /emoncms/input/post.json?"));

    if (node > 0) {
    
#ifdef debug
      Serial.println(F("Node ok"));
#endif

      client.print(F("node="));
      client.print(node);
      client.print(F("&"));
    }
    
#ifdef debug
    externalT = 0;
    externalH = 0;
    internalT2 = 0;
    sampleT = 0;
#endif

    client.print(F("json={"));
    client.print(internalTSerial);
    client.print(F(":"));
    client.print(internalT);
    //client.print("100");
    client.print(F(","));
    client.print(internalHSerial);
    client.print(F(":"));
    //client.print("100");
    client.print(internalH);
    client.print(F(","));
    client.print(externalTSerial);
    client.print(F(":"));
    //client.print("100");
    client.print(externalT);
    client.print(F(","));
    client.print(externalHSerial);
    client.print(F(":"));
    //client.print("100");
    client.print(externalH);
    client.print(F(","));
    client.print(internalT2Serial);
    client.print(F(":"));
    //client.print("100");
    client.print(internalT2);
    client.print(F(","));
    client.print(sampleTSerial);
    client.print(F(":"));
    //client.print("100");
    client.print(sampleT);
    client.print(F("}&apikey="));
    client.print(apikey);
    client.println(F(" HTTP/1.1"));
    client.println(F("Host: www.emoncms.org"));		//CHANGE THIS IF NEEDED
    client.println(F("User-Agent: Arduino-ethernet"));
    client.println(F("Connection: close"));
    client.println();
  }
  
  delay(1);
  client.stop();
}

void sendToEmonCMS() {
  unsigned long ora = millis();
  if (ora > transmissionRate + lastPost) {
  
#ifdef debug
    Serial.println(F("sendToEmonCMS()"));
#endif

    writeToSD();
    lastPost = ora;
    postJSON();
  }
}

void listenForEthernetAJAX() {
  EthernetClient client = server.available();
  if (client) {
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (req_index < (REQ_BUFF_SZ - 1)) {
          HTTP_req[req_index] = c;
          req_index++;
        }
        
        if (c == '\n' && currentLineIsBlank) {
          client.println(F("HTTP/1.1 200 OK"));
          if (StrContains(HTTP_req, "ajax_inputs")) {
            client.println(F("Content-Type: text/xml"));
            client.println(F("Cnnection: keep-alive"));
            client.println();
            setRelays();
            XML_response(client);
          } else if (StrContains(HTTP_req, "settings/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            webFile = SD.open(settings);
            if (webFile) {
              while (webFile.available()) {
                client.write(webFile.read());
              }
            
              webFile.close();
            }

            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            break;

          } else if (StrContains(HTTP_req, "lights/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            client.println(HTTP_req);
            String subString = HTTP_req;
            subString = subString.substring(subString.indexOf("?") + 1, subString.indexOf("HTTP"));
            int startP = 0;
            int endP = subString.indexOf("&");
            String onHour = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String onMinute = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String offHour = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String offMinute = subString.substring(startP, endP);
            client.println(onHour);
            client.println(onMinute);
            client.println(offHour);
            client.println(offMinute);
            uint8_t onHourValue = onHour.substring(onHour.indexOf("=") + 1, onHour.length()).toInt();
            uint8_t onMinuteValue = onMinute.substring(onMinute.indexOf("=") + 1, onMinute.length()).toInt();
            uint8_t offHourValue = offHour.substring(offHour.indexOf("=") + 1, offHour.length()).toInt();
            uint8_t offMinuteValue = offMinute.substring(offMinute.indexOf("=") + 1, offMinute.length()).toInt();
            setLightOnHour(onHourValue);
            setLightOnMinute(onMinuteValue);
            setLightOffHour(offHourValue);
            setLightOffMinute(offMinuteValue);
            //TODO: Page with settings report
            client.println(onHourValue);
            client.println(onMinuteValue);
            client.println(offHourValue);
            client.println(offMinuteValue);
            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            break;

          } else if (StrContains(HTTP_req, "temperature/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();

            String subString = HTTP_req;
            subString = subString.substring(subString.indexOf("?") + 1, subString.indexOf("HTTP"));
            int startP = 0;
            int endP = subString.indexOf("&");
            String minTemp = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String maxTemp = subString.substring(startP, endP);
            client.println(minTemp);
            client.println(maxTemp);
            int minTempValue = minTemp.substring(minTemp.indexOf("=") + 1, minTemp.length()).toInt();
            int maxTempValue = maxTemp.substring(maxTemp.indexOf("=") + 1, maxTemp.length()).toInt();
            setMinTemp(minTempValue);
            setMaxTemp(maxTempValue);
            //TODO: Page with settings report
            client.println(minTempValue);
            client.println(maxTempValue);
            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            break;

          } else if (StrContains(HTTP_req, "humidity/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            client.println(HTTP_req);
            String subString = HTTP_req;
            subString = subString.substring(subString.indexOf("?") + 1, subString.indexOf("HTTP"));
            int startP = 0;
            int endP = subString.indexOf("&");
            String minHum = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String maxHum = subString.substring(startP, endP);
            client.println(minHum);
            client.println(maxHum);
            int minHumValue = minHum.substring(minHum.indexOf("=") + 1, minHum.length()).toInt();
            int maxHumValue = maxHum.substring(maxHum.indexOf("=") + 1, maxHum.length()).toInt();
            setMinHum(minHumValue);
            setMaxHum(maxHumValue);
            //TODO: Page with settings report
            client.println(minHumValue);
            client.println(maxHumValue);
            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            break;

          } else if (StrContains(HTTP_req, "fans/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            client.println(HTTP_req);
            String subString = HTTP_req;
            subString = subString.substring(subString.indexOf("?") + 1, subString.indexOf("HTTP"));
            int startP = 0;
            int endP = subString.indexOf("&");
            String fanDurationS = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String fanIntervalS = subString.substring(startP, endP);
            if (subString.indexOf("always=1") == -1) {
              setFansAlwaysONDay(false);
              client.println("<p>Fans Always ON whith lights ON Disabled</p>");
            } else {
              setFansAlwaysONDay(true);
              client.println("<p>Fans Always ON whith lights ON Enabled</p>");
            }
            client.println(fanDurationS);
            client.println(fanIntervalS);
            int fanDurationValue = fanDurationS.substring(fanDurationS.indexOf("=") + 1, fanDurationS.length()).toInt();
            int fanIntervalValue = fanIntervalS.substring(fanIntervalS.indexOf("=") + 1, fanIntervalS.length()).toInt();
            setFanDurationDay(fanDurationValue);
            setFanIntervalDay(fanIntervalValue);
            //TODO: Page with settings report
            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            break;

          } else if (StrContains(HTTP_req, "fansLO/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            client.println(HTTP_req);
            String subString = HTTP_req;
            subString = subString.substring(subString.indexOf("?") + 1, subString.indexOf("HTTP"));
            int startP = 0;
            int endP = subString.indexOf("&");
            String fanDurationS = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String fanIntervalS = subString.substring(startP, endP);
            if (subString.indexOf("always=1") == -1) {
              setFansAlwaysONNight(false);
              client.println("<p>Fans Always ON whith lights OFF Disabled</p>");
            } else {
              setFansAlwaysONNight(true);
              client.println("<p>Fans Always ON whith lights OFF Enabled</p>");
            }
            client.println(fanDurationS);
            client.println(fanIntervalS);
            int fanDurationValue = fanDurationS.substring(fanDurationS.indexOf("=") + 1, fanDurationS.length()).toInt();
            int fanIntervalValue = fanIntervalS.substring(fanIntervalS.indexOf("=") + 1, fanIntervalS.length()).toInt();
            setFanDurationNight(fanDurationValue);
            setFanIntervalNight(fanIntervalValue);
            //TODO: Page with settings report
            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            break;

          } else if (StrContains(HTTP_req, "rtc/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            client.println(HTTP_req);
            // Day, Month, Year, Hour, Minute, Second
            tmElements_t tm;
            time_t t;

            String subString = HTTP_req;
            subString = subString.substring(subString.indexOf("?") + 1, subString.indexOf("HTTP"));
            int startP = 0;
            int endP = subString.indexOf("&");
            String setDay = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String setMonth = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String setYear = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String setHour = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String setMinute = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String setSecond = subString.substring(startP, endP);

            client.println(setDay);
            client.println(setMonth);
            client.println(setYear);
            client.println(setHour);
            client.println(setMinute);
            client.println(setSecond);

            tm.Day = setDay.substring(setDay.indexOf("=") + 1, setDay.length()).toInt();
            tm.Month = setMonth.substring(setMonth.indexOf("=") + 1, setMonth.length()).toInt();
            int setYearValue = setYear.substring(setYear.indexOf("=") + 1, setYear.length()).toInt();
            if (setYearValue >= 100 && setYearValue < 1000) {
#ifdef debug
              Serial << F("Error: Year must be two digits or four digits!") << endl;
#endif
            } else {
              if (setYearValue >= 1000)
                tm.Year = CalendarYrToTm(setYearValue);
              else    //(setYearValue < 100)
                tm.Year = y2kYearToTm(setYearValue);
              tm.Hour = setHour.substring(setHour.indexOf("=") + 1, setHour.length()).toInt();
              tm.Minute = setMinute.substring(setMinute.indexOf("=") + 1, setMinute.length()).toInt();
              tm.Second = setSecond.substring(setSecond.indexOf("=") + 1, setSecond.length()).toInt();

              t = makeTime(tm);
              RTC.set(t);
              setTime(t);
            }
            //TODO: Page with settings report
            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            break;

          } else if (StrContains(HTTP_req, "samplerate/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            client.println(HTTP_req);
            String subString = HTTP_req;
            subString = subString.substring(subString.indexOf("?") + 1, subString.indexOf("HTTP"));
            int startP = 0;
            int endP = subString.indexOf("&");
            String sampleRateS = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String transmissionIntervalS = subString.substring(startP, endP);
            client.println(sampleRateS);
            client.println(transmissionIntervalS);
            int sampleRateValue = sampleRateS.substring(sampleRateS.indexOf("=") + 1, sampleRateS.length()).toInt();
            int transmissionIntervalValue = transmissionIntervalS.substring(transmissionIntervalS.indexOf("=") + 1, transmissionIntervalS.length()).toInt();
            setSampleRate(sampleRateValue);
            setTransmissionRate(transmissionIntervalValue * 60000);
            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            break;

          } else if (StrContains(HTTP_req, "groundt/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            client.println(HTTP_req);
            String subString = HTTP_req;
            subString = subString.substring(subString.indexOf("?") + 1, subString.indexOf("HTTP"));
            int startP = 0;
            int endP = subString.indexOf("&");
            String minGroundTS = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String maxGroundTS = subString.substring(startP, endP);
            client.println(minGroundTS);
            client.println(maxGroundTS);
            int minGroundTValue = minGroundTS.substring(minGroundTS.indexOf("=") + 1, minGroundTS.length()).toInt();
            int maxGroundTValue = maxGroundTS.substring(maxGroundTS.indexOf("=") + 1, maxGroundTS.length()).toInt();
            setMinGroundT(minGroundTValue);
            setMaxGroundT(maxGroundTValue);
            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            break;

          } else if (StrContains(HTTP_req, "network/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            client.println(HTTP_req);
            String subString = HTTP_req;
            subString = subString.substring(subString.indexOf("?") + 1, subString.indexOf("HTTP"));
            int startP = 0;
            int endP = subString.indexOf("&");
            String ip1S = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String ip2S = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String ip3S = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String ip4S = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String snm1S = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String snm2S = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String snm3S = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String snm4S = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String gw1S = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String gw2S = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String gw3S = subString.substring(startP, endP);
            startP = endP + 1;
            endP = subString.indexOf("&", startP);
            String gw4S = subString.substring(startP, endP);

            client.println(ip1S + " " + ip2S + " " + ip3S + " " + ip4S);
            client.println(snm1S + " " + snm2S + " " + snm3S + " " + snm4S);
            client.println(gw1S + " " + gw2S + " " + gw3S + " " + gw4S);
            int ip1V = ip1S.substring(ip1S.indexOf("=") + 1, ip1S.length()).toInt();
            int ip2V = ip2S.substring(ip2S.indexOf("=") + 1, ip2S.length()).toInt();
            int ip3V = ip3S.substring(ip3S.indexOf("=") + 1, ip3S.length()).toInt();
            int ip4V = ip4S.substring(ip4S.indexOf("=") + 1, ip4S.length()).toInt();

            int snm1V = snm1S.substring(snm1S.indexOf("=") + 1, snm1S.length()).toInt();
            int snm2V = snm2S.substring(snm2S.indexOf("=") + 1, snm2S.length()).toInt();
            int snm3V = snm3S.substring(snm3S.indexOf("=") + 1, snm3S.length()).toInt();
            int snm4V = snm4S.substring(snm4S.indexOf("=") + 1, snm4S.length()).toInt();

            int gw1V = gw1S.substring(gw1S.indexOf("=") + 1, gw1S.length()).toInt();
            int gw2V = gw2S.substring(gw2S.indexOf("=") + 1, gw2S.length()).toInt();
            int gw3V = gw3S.substring(gw3S.indexOf("=") + 1, gw3S.length()).toInt();
            int gw4V = gw4S.substring(gw4S.indexOf("=") + 1, gw4S.length()).toInt();
            setIPAddress(ip1V, ip2V, ip3V, ip4V);
            setSNMAddress(snm1V, snm2V, snm3V, snm4V);
            setGWAddress(gw1V, gw2V, gw3V, gw4V);
            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            initializeServer();
            break;

          } else if (StrContains(HTTP_req, "dhcp/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            client.println(HTTP_req);
            String subString = HTTP_req;
            if (subString.indexOf("dhcp=1") == -1) {
              setDHCP(false);
              client.println("<p>DHCP Disabled</p>");
            } else {
              setDHCP(true);
              client.println("<p>DHCP Enabled</p>");
            }

            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            break;

          } else if (StrContains(HTTP_req, "parameters/")) {
            client.println(F("Content-Type: text/plain"));
            client.println(F("Connection: keep-alive"));
            client.println();
            client.println(HTTP_req);
            client.println(F("Current Parameters"));
            client.println("Light On Hour: " + (String) lightOnHour);
            client.println("Light On Minute: " + (String) lightOnMinute);
            client.println("Light Off Hour: " + (String) lightOffHour);
            client.println("Light Off Minute: " + (String) lightOffMinute);
            client.println("Min Temp: " + (String) minIntTemp);
            client.println("Max Temp: " + (String) maxIntTemp);
            client.println("Min Hum: " + (String) minIntHum);
            client.println("Max Hum: " + (String) maxIntHum);
            client.println("Sample Rate:" + (String)sampleRate);
            client.println("Transmission Rate: " + (String)transmissionRate);
            client.println("Fan Interval: " + (String)fanIntervalDay);
            client.println("Fan Duration: " + (String)fanDurationDay);
            client.println("Fan Always On Day: " + (String)fanAlwaysOnDay);

            client.println("Fan Interval: " + (String)fanIntervalNight);
            client.println("Fan Duration: " + (String)fanDurationNight);
            client.println("Fan Always On Night: " + (String)fanAlwaysOnNight);
            req_index = 0;
            StrClear(HTTP_req, REQ_BUFF_SZ);
            break;

          } else if (StrContains(HTTP_req, "ok")) {
            Serial.println("OK");
          } else if (StrContains(HTTP_req, "download/")) {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            client.println(F("<!DOCTYPE html>"));
            client.println(F("<meta charset=\"utf-8\" />"));
            client.println(F("<html>"));
            client.println(F("<head>"));
            client.println(F("<title>Arduino GrowBox - Download CSV</title>"));
            webFile = SD.open(css);
            if (webFile) {
              while (webFile.available()) {
                client.write(webFile.read());
              }
              webFile.close();
            }
            
            client.println(F("<body>"));
            client.println(F("<h1>Arduino GrowBox Data Download</h1>"));
            client.println(F("<a href =\"/\">Home</a>"));
            client.println(F("<div class=\"IO_box\">"));

            client.println(F("<h2>File List</h2>"));
            //print file list
            client.println(F("<ul>"));
            File root = SD.open(F("/logs/"));

            while (true) {
              File entry = root.openNextFile();
              
#ifdef debug
              Serial.println(entry.name());
#endif

              if (!entry) {
                break;
              }
              
              client.println("<li><a href=\"/file/" + (String)entry.name() + "\">" + (String)entry.name() + "</a>");
              entry.close();
            }

            client.println(F("</ul>"));
            client.println(F("</div>"));
            client.println(F("</body>"));
            client.println(F("</html>"));
            StrClear(HTTP_req, REQ_BUFF_SZ);
          } else if (StrContains(HTTP_req, "/file/")) {
            client.println(F("Content-Type: application/octet-stream"));
            client.println(F("Connection: keep-alive"));
            client.println();
            String fileToServeName;
            for (int i = 10; i < 22; i++) {
              //fileToDownload[i-10] = HTTP_req[i];
              fileToServeName += HTTP_req[i];
            }

            String downloadFile = logDirName + fileToServeName;
            File fileToServe = SD.open(downloadFile, FILE_READ);
            if (fileToServe) {
              while (fileToServe.available()) {
                client.write(fileToServe.read());
              }
              fileToServe.close();
            }

            StrClear(HTTP_req, REQ_BUFF_SZ);
          } else {
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: keep-alive"));
            client.println();
            webFile = SD.open(index);
            if (webFile) {
              while (webFile.available()) {
                client.write(webFile.read());
              }
              webFile.close();
            }
          }
          
#ifdef debug
          Serial.println(HTTP_req);
#endif

          req_index = 0;
          StrClear(HTTP_req, REQ_BUFF_SZ);
          break;
        }
        
        if (c == '\n') {
          currentLineIsBlank = true;
        } else if ( c != '\r') {
          currentLineIsBlank = false;
        }
      }
    }
    
    delay(1);
    client.stop();
  }
}

void XML_response(EthernetClient cl){
  
  int count;                 
  //int sw_arr[] = {2, 3, 5};
  float sensors[] = { internalT, internalH, internalT2, sampleT, externalT, externalH};
  cl.print(F("<?xml version = \"1.0\" ?>"));
  cl.print(F("<inputs>"));
  // read analog inputs
  for (count = 0; count <= 5; count++) { 
    cl.print(F("<analog>"));
    cl.print(sensors[count]);
    cl.println(F("</analog>"));
  }

  // checkbox RELAY states
  // RELAY1
  cl.print(F("<RELAY>"));
  if (RELAY_state[0]) {
    cl.print(F("on"));
  }
  else {
    cl.print(F("off"));
  }
  cl.println(F("</RELAY>"));
  // RELAY2
  cl.print(F("<RELAY>"));
  if (RELAY_state[1]) {
    cl.print(F("on"));
  }
  else {
    cl.print(F("off"));
  }
  cl.println(F("</RELAY>"));
  // RELAY3
  cl.print(F("<RELAY>"));
  if (RELAY_state[2]) {
    cl.print(F("on"));
  }
  else {
    cl.print(F("off"));
  }
  cl.println(F("</RELAY>"));
  // RELAY4
  cl.print(F("<RELAY>"));
  if (RELAY_state[3]) {
    cl.print(F("on"));
  }
  else {
    cl.print(F("off"));
  }
  cl.println(F("</RELAY>"));

  cl.print(F("</inputs>"));
}

void setRelays() {
  // RELAY1 (pin 6)
  if (StrContains(HTTP_req, "RELAY1=1")) {

    startLight();
  }
  else if (StrContains(HTTP_req, "RELAY1=0")) {

    stopLight();
  }
  // RELAY2 (pin 7)
  if (StrContains(HTTP_req, "RELAY2=1")) {

    startHeater();
  }
  else if (StrContains(HTTP_req, "RELAY2=0")) {

    stopHeater();
  }
  // RELAY3 (pin 8)
  if (StrContains(HTTP_req, "RELAY3=1")) {

    startFan();
  }
  else if (StrContains(HTTP_req, "RELAY3=0")) {

    stopFan();
  }
  // RELAY4 (pin 9)
  if (StrContains(HTTP_req, "RELAY4=1")) {

    startWater();
  }
  else if (StrContains(HTTP_req, "RELAY4=0")) {

    stopWater();
  }
}

void StrClear(char *str, char length) {
  for (int i = 0; i < length; i++) {
    str[i] = 0;
  }
}

char StrContains(const char *str, const char *sfind) {
  char found = 0;
  char index = 0;
  char len;

  len = strlen(str);

  if (strlen(sfind) > len) {
    return 0;
  }
  while (index < len) {
    if (str[index] == sfind[found]) {
      found++;
      if (strlen(sfind) == found) {
        return 1;
      }
    }
    else {
      found = 0;
    }
    index++;
  }

  return 0;
}

void startHeater() {
  if (RELAY_state[HEATER] == 0) {
    digitalWrite(relay_Heater1, RELAYON);
    RELAY_state[HEATER] = 1;  // save RELAY state
    postOnOffEvent("heater", 1);
  }
}

void stopHeater() {
  if (RELAY_state[HEATER] == 1) {
    digitalWrite(relay_Heater1, RELAYOFF);
    RELAY_state[HEATER] = 0;  // save RELAY state
    postOnOffEvent("heater", 0);
  }
}

void startLight() {
  if (RELAY_state[LIGHTS] == 0) {
    digitalWrite(relay_Lights1, RELAYON);
    RELAY_state[LIGHTS] = 1;  // save RELAY state
    postOnOffEvent("lights", 1);
  }
}

void stopLight() {
  if (RELAY_state[LIGHTS] == 1) {
    digitalWrite(relay_Lights1, RELAYOFF);
    RELAY_state[LIGHTS] = 0;  // save RELAY state
    postOnOffEvent("lights", 0);
  }
}

void startFan() {
  if (RELAY_state[FAN] == 0) {
    digitalWrite(relay_Fan1, RELAYON);
    RELAY_state[FAN] = 1;  // save RELAY state
    fanOn = true;
    postOnOffEvent("fans", 1);
  }
}

void stopFan() {
  if (RELAY_state[FAN] == 1) {
    digitalWrite(relay_Fan1, RELAYOFF);
    RELAY_state[FAN] = 0;  // save RELAY state
    fanOn = false;
    postOnOffEvent("fans", 0);
  }
}

void startWater() {
  if (RELAY_state[MIST] == 0) {
    digitalWrite(relay_Water1, RELAYON);
    RELAY_state[MIST] = 1;  // save RELAY state
    postOnOffEvent("mist", 1);
  }
}

void stopWater() {
  if (RELAY_state[MIST] == 1) {
    digitalWrite(relay_Water1, RELAYOFF);
    RELAY_state[MIST] = 0;  // save RELAY state
    postOnOffEvent("mist", 0);
  }
}

void checkHeater() {
  if (internalT < minIntTemp) {
    if (sampleT < minGroundT) {
      startHeater();
    } else if (sampleT > maxGroundT) {
      stopHeater();
    }
  }

  if (internalT > maxIntTemp) {
    stopHeater();
  }
}

void checkLights() {
  tmElements_t tm;
  RTC.read(tm);
  uint16_t nowMin = tm.Hour * 60 + tm.Minute;
  uint16_t startMin = lightOnHour * 60 + lightOnMinute;
  uint16_t stopMin = lightOffHour * 60 + lightOffMinute;

  if (lightOnHour <= lightOffHour) {
    if ((startMin <= (nowMin)) && (nowMin <= (stopMin))) {
      startLight();
    } else {
      stopLight();
    }
  } else if (((startMin) <= (nowMin)) || (nowMin <= (stopMin))) {
    startLight();
  } else {
    stopLight();
  }
}

void checkFan() {
  if (RELAY_state[LIGHTS]) {
    //Lights ON
    if (fanAlwaysOnDay) {
      startFan();
      fanStartTime = millis();
    } else {
	  //Turn on the fan for fanDuration*60000 seconds every fanInterval hours
      if (fanOn && millis() > fanStartTime + (fanDurationDay * 60000)) {
        stopFan();
        fanStartTime = millis();
      }
      if (!fanOn && millis() > fanStartTime + (fanIntervalDay * 60000 * 60)) {
        startFan();
        fanStartTime = millis();
      }
    }
  } else {
    //Lights OFF
    if (fanAlwaysOnNight) {
      startFan();
      fanStartTime = millis();
    } else {
      //Turn on the fan for fanDuration*60000 seconds every fanInterval hours
      if (fanOn && millis() > fanStartTime + (fanDurationNight * 60000)) {
        stopFan();
        fanStartTime = millis();
      }
      if (!fanOn && millis() > fanStartTime + (fanIntervalNight * 60000 * 60)) {
        startFan();
        fanStartTime = millis();
      }
    }
  }
}

void checkMist() {
  if (internalH < minIntHum) {
    startWater();
  }
  if (internalH > maxIntHum) {
    stopWater();
  }
}

void checkParameters() {
  checkHeater();
  checkLights();
  checkFan();
  checkMist();
}

void loop() {
  updateSensors();
  lcdDispatch();
  checkParameters();
  sendToEmonCMS();
  listenForEthernetAJAX();
  //Ethernet.maintain();
}

