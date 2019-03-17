// Copyright 2018 Steve Quinn
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
// Requires MQTT PubSubClient Library found here: https://github.com/knolleary/pubsubclient
// Requires ESP8266WiFi Library found here:       https://github.com/ekstrand/ESP8266wifi/
// Requires Adafruit DHT Library found here:      https://github.com/adafruit/DHT-sensor-library
// Requires MCP4561 Library found here:           https://github.com/SteveQuinn1/MCP4561_DIGI_POT
// Requires Arduino IDE support for ESP8266 found here: http://esp8266.github.io/Arduino/versions/2.2.0-rc1/doc/installing.html
// Requires SD file system support found in ESP8266 Core or here : https://github.com/esp8266/Arduino/tree/master/libraries/SD https://github.com/arduino-libraries/SD
// Requires ESP8266WebServer Library found here:  http://esp8266.github.io/Arduino/versions/2.2.0-rc1/doc/installing.html
// Requires ESP8266mDNS Library found here:       http://esp8266.github.io/Arduino/versions/2.2.0-rc1/doc/installing.html
// Requires Modified Adafruit MCP23017 I2C library found here : https://github.com/SteveQuinn1/Adafruit-MCP23017-Arduino-Library
// Requires Adafruit AM2320 Library found here : https://github.com/adafruit/Adafruit_AM2320
// Requires Adafruit Unified Sensor Driver found here : https://github.com/adafruit/Adafruit_Sensor
// 
// ESP8266-07 MQTT WiFi Temperature and Humidity Sensor, 3 Led outputs and an SP0256-AL2 Speech Synth with digital volume control.
//
// Compiled on Arduino IDE 1.6.9
//
// Maintenance History
// 25/08/18 : 18   Integrated ESP8266TempHumi7_17_5 for connection state machine, and added MQTTWiFiAll16_9_7 inproved T&H reporting. #defined out LED on Pin 0 as this uses an ESP8266-7 programmed in situ. Works with SPIFFS
// 25/08/18 : 19   Ported over to SD memory
// 25/08/18 : 20   Started integration of original ESP826612E_SpeechSynth17
// 26/08/18 : 21   Got switched index page working. Needs fully intgrating and testing.
// 26/08/18 : 21_1 Tried closing and beginning the WebServer in order to get it to recognise a new HTTP_GET handler. Didn't work.
// 26/08/18 : 22   Used gating var for HTTP_GET Handler. Dirty fix but it now works. Rewired (added digital pot) and documented target circuit.
// 08/09/18 : 22_1 Branch to support control of the digital potentiometer. Working. Fixed 'bool' file read issue 'FILE_VAR_INSTANCE_TYPE_BOOL'. Corrected all other code which inherits fileRead.
// 22/10/18 : 22_2 Added updated digital pot library
// 31/10/18 : 22_3 Added AM2320 SHT10 library and second temperature/himidity sensor.
// 02/03/19 : 22_4 Corrected topic paths. Ensuring explicit Command/Confirms. Spotted via InfluxDB and Grafana infrastructure.
// 
//
//
// Start up sequence
// -----------------
// Unit starts up in STA_AP mode and looks for SD file SECURITY_PARAMETERS_FILE. If the file doesn't exist the IoT device switches to AP mode, 
// starts a web server and initialises an mDNS service. The device then broadcasts the SSID AP_NETWORK_SSID_DEFAULT + DeviceMACAddress. To connect 
// to this APnetwork use the following password AP_NETWORK_PASSWORD_DEFAULT once connected enter the following URL into your browser nDNSHostName + '.local'
// The IoT device will then serve up a configuration web page allowing new sensor network security parameters to be submitted.
// If the SD file is present the information in this file is used by the IoT device to connect to the sensor network. The device then stays in STA_AP 
// mode until it has connected. Once connected to the sensor network the IoT device switches to STA mode.
// If the information in the SECURITY_PARAMETERS_FILE does not allow connection to the sensor network it is possible to connect to the sensors APnetwork as above,
// only now you must enter the IP address 192.168.4.1. This is due to a flaw in the mDNS library of the ESP8266. When in STA_AP mode mDNS service
// will not work.
// Once the device has connected to the WiFi network it attempts to connect to the MQTT broker which it expects at the following address MQTT_BROKER_IP_DEFAULT
// and port MQTT_BROKER_PORT_DEFAULT. If the IoT device exceeds mqtt_broker_connection_attempts it will re-initialise as if no SECURITY_PARAMETERS_FILE were present.
// Once connected to the MQTT broker, if the connection to the broker is lost the system re-initialises.
// If mqtt_broker_connection_attempts=0 the IoT device will continue to attempt an MQTT Broker connection.
//
// Once WiFi and MQTT connections are made, the device offers two Speech server interfaces 1. Via a web server by entering nDNSHostName + '.local' into yourr browser 
// URL bar and the other via MQTT topics
//
// To give a visual indication of the above connection states, the IoT device will flash the local (lightPin) led as follows.
// 1. When no onboard configuration file is present SECURITY_PARAMETERS_FILE 1 quick flash.
// 2. When attempting to connect to a given WiFi network 2 quick flashes in succession.
// 3. Once a WiFi n/w connection has been achieved. Whilst attempting to connect to an MQTT Broker the led will be on continuously.
// 4. Once WiFi n/w and MQTT Broker connections are in place the led will extinguish.
//
//
// 'calvals1.txt' contains five entries. These values are exposed for read write via MQTT topics. For AM2320.
// - 1st Calibration zero offset for Temperature a float
// - 2nd Calibration scale factor for Temperature a float
// - 3rd Calibration zero offset for Humidity a float
// - 4th Calibration scale factor for Humidity a float
// - 5th Value is the reporting strategy value. 0 = Send and update when a change is detected in the monitored value. 1...60 = report back the monitored value every 'n' minutes
//
// 'calvals2.txt' contains five entries. These values are exposed for read write via MQTT topics. For DHT22.
// - 1st Calibration zero offset for Temperature a float
// - 2nd Calibration scale factor for Temperature a float
// - 3rd Calibration zero offset for Humidity a float
// - 4th Calibration scale factor for Humidity a float
// - 5th Value is the reporting strategy value. 0 = Send and update when a change is detected in the monitored value. 1...60 = report back the monitored value every 'n' minutes
//
// 'secvals.txt' contains five entries. These values are write only via MQTT topics.
// - 1st MQTT Broker IP Address. In dotted decimal form AAA.BBB.CCC.DDD
// - 2nd MQTT Broker Port. In Integer form.
// - 3rd MQTT Broker connection attempts to make before switching from STA mode to AP mode. In Integer form. 
// - 4th WiFi Network SSID. In free form text.
// - 5th WiFi Network Password. In free form text.
// - 6th WiFi Network connection attempts to make before switching from STA mode to AP mode. In Integer form. // NOTE this is not implemented
//
// 'confvals.txt' contains three entries 
// - 1st Digital potentiometer present. 1 = Present, 0 = Not Fitted
// - 2nd Initial volume level. 0...255
// - 3rd Announce system ready. Use the speech synth to announce when the system has completed boot sequence. 1 = Announce, 0 = Do Not announce
//
// 'index.htm'
// Contains web page served up when the device can't connect to the Network using the password held in the 'secvals.txt' file
//
// Arduino IDE programming parameters.
// 
// From Tools Menu
// Board: 'Generic ESP8266 Module'
// Flash Mode: 'DIO'
// Flash Size: '1M (512K SPIFFS)'
// Debug Port: 'Disabled'
// Debug Level: 'None'
// Reset Method: 'ck'
// Flash Frequency '40MHz'
// CPU Frequency '80 MHz'
// Upload Speed: '115200'
// 


#define DEBUG_GENERAL        // Undefine this for general debug information via the serial port. Note, you must deploy with this undefined as your network security parameters will be sent to serial port
//#define DEBUG_WEB            // Undefine this for comprehensive debug information on web interface via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_MDNS           // Undefine this for comprehensive debug information on mDNS support via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_TIMER          // Undefine this for timer debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_SD             // Undefine this for SD debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_VALIDATION     // Undefine this for validation debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_STATE_CHANGE   // Undefine this for 'eSENSORSTATE' State Change debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_LEDFLASH       // Undefine this for 'eLEDFLASHSTATE' State Change debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_SECVALS        // Undefine this for MQTT SecVals Topic debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_PARMGRAB       // Undefine this for parmGrab function debug information via the serial port. Requires DEBUG_GENERAL.
//#define DEBUG_FILE_RW        // Undefine this for fileRead/fileWrite debug information via the serial port. Requires DEBUG_GENERAL.


#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <DHT.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MCP23017.h>
#include "MCP4561_DIGI_POT.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"

MCP4561 digiPot;  //declare an instance of MAX4561 with Device addr 46 ( 0101 110 ), ie, A0 pin pulled low

typedef struct {
  uint8_t hexAddress;
  const char *Allophone;
} tAllophoneAddressTableStruct;

const tAllophoneAddressTableStruct allophoneAddressTable[] = {
  {0x00, "PA1"}, {0x01, "PA2"}, {0x02, "PA3"}, {0x03, "PA4"}, {0x04, "PA5"}, {0x05, "OY" }, {0x07, "AY" }, {0x07, "EH" },
  {0x08, "KK3"}, {0x09, "PP" }, {0x0A, "JH" }, {0x0B, "NN1"}, {0x0C, "IH" }, {0x0D, "TT2"}, {0x0E, "RR1"}, {0x0F, "AX" },
  {0x10, "MM" }, {0x11, "TT1"}, {0x12, "DH1"}, {0x13, "IY" }, {0x14, "EY" }, {0x15, "DD1"}, {0x16, "UW1"}, {0x17, "AO" },
  {0x18, "AA" }, {0x19, "YY2"}, {0x1A, "AE" }, {0x1B, "HH1"}, {0x1C, "BB1"}, {0x1D, "TH" }, {0x1E, "UH" }, {0x1F, "UW2"},
  {0x20, "AW" }, {0x21, "DD2"}, {0x22, "GG3"}, {0x23, "VV" }, {0x24, "GG1"}, {0x25, "SH" }, {0x26, "ZH" }, {0x27, "RR2"},
  {0x28, "FF" }, {0x29, "KK2"}, {0x2A, "KK1"}, {0x2B, "ZZ" }, {0x2C, "NG" }, {0x2D, "LL" }, {0x2E, "WW" }, {0x2F, "XR" },
  {0x30, "WH" }, {0x31, "YY1"}, {0x32, "CH" }, {0x33, "ER1"}, {0x34, "ER2"}, {0x35, "OW" }, {0x36, "DH2"}, {0x37, "SS" },
  {0x38, "NN2"}, {0x39, "HH2"}, {0x3A, "OR" }, {0x3B, "AR" }, {0x3C, "YR" }, {0x3D, "GG2"}, {0x3E, "EL" }, {0x3F, "BB2"}
};

#define SP0256_PA1 0x00
#define SP0256_PA2 0x01
#define SP0256_PA3 0x02
#define SP0256_PA4 0x03
#define SP0256_PA5 0x04
#define PHONEME_UNRECOGNISED 0xFF
uint8_t maxPhonemes = 0;

#define SPBit0                 0
#define SPBit1                 1
#define SPBit2                 2
#define SPBit3                 3
#define SPBit4                 4
#define SPBit5                 5
#define ADDR_LOAD              6
#define LOAD_REQUEST           7

#define RED_LED                8  // Port B0 of the MCP23017
#define BLUE_LED               9  // Port B1 of the MCP23017
#define SPEECH_PROCESSOR_RESET 10 // Port B2 of the MCP23017
#define lightPin               11 // Port B3 of the MCP23017


typedef enum {
    eSPEECH_IDLE = 0,
    eSPEECH_SPEAKING_HEX_PHONEMES = 1,
    eSPEECH_SPEAKING_PHONEMES = 2,
    eSPEECH_SPEAKING_FIXED_PHRASE = 3,
    eSPEECH_HTTP = 4,
    eSPEECH_SPEAKING_WORDS = 5,
    eSPEECH_QUERYING_DATABASE = 6
} eSpeechProcessorState;

eSpeechProcessorState volatile SpeechProcessorCurrentState = eSPEECH_IDLE;


#define PHONEME_END_MARKER 0xFF
uint8_t volatile whichPhonemePhrase = 0;
uint8_t volatile currentPhoneme = 0;
uint8_t maxFixedPhrases = 0;

const uint8_t PHONEME_PHRASE_1[] = {SP0256_PA1, 0x37, 0x0D, 0x13, 0x01, 0x23, 0x02, 0x0B, 0x03, 0x08, 0x00, 0x2E, 0x0C, 0x00, 0x0B, SP0256_PA1, PHONEME_END_MARKER}; // Steve Quinn
const uint8_t PHONEME_PHRASE_2[] = {SP0256_PA1, 0x37, 0x00, 0x0C, 0x37, 0x00, 0x0D, 0x00, 0x07, 0x10, 0x00, 0x0E, 0x07, 0x21, 0x0C, 0x14, SP0256_PA1, PHONEME_END_MARKER}; // System Ready
const uint8_t PHONEME_PHRASE_3[] = {SP0256_PA1, 0x1B, 0x07, 0x07, 0x2D, 0x00, 0x35, 0x03, 0x2E, 0x00, 0x34, 0x2D, 0x00, 0x15, SP0256_PA1, PHONEME_END_MARKER}; // Hello World
const uint8_t PHONEME_PHRASE_4[] = {0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, SP0256_PA1, PHONEME_END_MARKER}; // Test Pattern
const uint8_t *PHONEME_FIXED_PHRASE_STORE[] = {PHONEME_PHRASE_1, PHONEME_PHRASE_2, PHONEME_PHRASE_3, PHONEME_PHRASE_4};

typedef enum {
    eSPEECH_FIXED_PHRASE_STEVE_QUINN = 0,
    eSPEECH_FIXED_PHRASE_SYSTEM_READY = 1,
    eSPEECH_FIXED_PHRASE_HELLO_WORLD = 2,
    eSPEECH_FIXED_PHRASE_TEST_PATTERN = 3
} eSpeechProcessorFixedPhrase;

#define MAX_PHONEMES 100
uint8_t phonemeBufferStore[MAX_PHONEMES];


#define MCP23017_ADDR ((uint8_t)0x07) // This is because all address lines of the MCP23017 have been tied high
#define I2C_SDA       4               // s/w defined I2C lines on the ESP8266-07
#define I2C_SCL       5               // s/w defined I2C lines on the ESP8266-07
Adafruit_MCP23017     mcp23017;

const char hexString[] = {"0123456789ABCDEF"};

#define TO_UPPER 0xDF
#define MAKE_UPPER_CASE(c)     ((c) & TO_UPPER)
#define SET_BIT(p,whichBit)    ((p) |=  (1    << (whichBit)))
#define CLEAR_BIT(p,whichBit)  ((p) &= ~((1)  << (whichBit)))
#define TOGGLE_BIT(p,whichBit) ((p) ^=  (1    << (whichBit)))
#define BIT_IS_SET(p,whichBit) ((p) &   (1    << (whichBit)))

#define SD_FILE_READ_MODE  FILE_READ
//#define SD_FILE_WRITE_MODE FILE_WRITE
#define SD_FILE_WRITE_MODE (O_WRITE | O_CREAT | O_TRUNC)


#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)


#define CALIBRATION_PARAMETERS_FILE_AM2320            ((char *)("/calvals1.txt"))
#define CALIBRATION_PARAMETERS_FILE_DHT22             ((char *)("/calvals2.txt"))
#define SECURITY_PARAMETERS_FILE                      ((char *)("/secvals.txt"))
#define CONFIGURATION_PARAMETERS_FILE                 ((char *)("/confvals.txt"))
#define LOWER_REPORTING_STRATEGY_VALUE                0  // Report any change
#define UPPER_REPORTING_STRATEGY_VALUE                60 // Send updates every hour
#define TEMPERATURE_CALIBRATION_OFFSET_DHT22_DEFAULT  ((float)0.0)
#define TEMPERATURE_CALIBRATION_SCALE_DHT22_DEFAULT   ((float)1.0)
#define HUMIDITY_CALIBRATION_OFFSET_DHT22_DEFAULT     ((float)0.0) 
#define HUMIDITY_CALIBRATION_SCALE_DHT22_DEFAULT      ((float)1.0) 
#define REPORTING_STRATEGY_DHT22_DEFAULT              LOWER_REPORTING_STRATEGY_VALUE
#define DEFAULT_TH_SENSOR_INTERVAL_DHT22              5000  // Temperature/Humidity Sensor read interval in Milliseconds
#define TEMPERATURE_CALIBRATION_OFFSET_AM2320_DEFAULT ((float)0.0)
#define TEMPERATURE_CALIBRATION_SCALE_AM2320_DEFAULT  ((float)1.0)
#define HUMIDITY_CALIBRATION_OFFSET_AM2320_DEFAULT    ((float)0.0) 
#define HUMIDITY_CALIBRATION_SCALE_AM2320_DEFAULT     ((float)1.0) 
#define REPORTING_STRATEGY_AM2320_DEFAULT             LOWER_REPORTING_STRATEGY_VALUE
#define DEFAULT_TH_SENSOR_INTERVAL_AM2320             5000  // Temperature/Humidity Sensor read interval in Milliseconds

#define DEFAULT_DIGITAL_POT_PRESENCE                  false // Assume no digital pot fitted
#define DEFAULT_DIGITAL_POT_SETTING                   (int)128 // Mid scale
#define DEFAULT_DIGITAL_POT_SETTING_MIN               (int)0   // Zero scale
#define DEFAULT_DIGITAL_POT_SETTING_MAX               (int)255 // Full scale
#define DEFAULT_ANNOUNCE_SYSTEM_READY                 false // Stay silent


bool  bDigitalPotPresence  = DEFAULT_DIGITAL_POT_PRESENCE;
int   iDigitalPotSetting   = DEFAULT_DIGITAL_POT_SETTING;
bool  bAnnounceSystemReady = DEFAULT_ANNOUNCE_SYSTEM_READY;


Adafruit_AM2320 am2320 = Adafruit_AM2320();

#define DHTPIN 2        // The digital o/p pin we're connected to. GPIO2
#define DHTTYPE DHT22   // DHT 22, AM2321

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);


typedef enum eDHTParameterTypeTag {
  eDHTParameterTemperature  = 0,
  eDHTParameterHumidity     = 1
} eDHTParameterType;

// Values read from Temperature/Humidity sensor
volatile eDHTParameterType whichDHTParameterToRead = eDHTParameterTemperature;


// Values read from DHT22 sensor
float temp_c_oldDHT22;                 // Temperature in Centigrade, earlier reading
float temp_c_newDHT22;                 // Temperature in Centigrade, latest reading
float humidity_oldDHT22;               // Relative humidity in %age, earlier reading
float humidity_newDHT22;               // Relative humidity in %age, latest reading
float hic_oldDHT22;                    // Heat Index in Centigrade, earlier reading
float hic_newDHT22;                    // Heat Index in Centigrade, latest reading
unsigned long previousMillisDHT22 = 0; // previous time value, so routine is non blocking
unsigned long readIntervalTempHumiDHT22 = DEFAULT_TH_SENSOR_INTERVAL_DHT22;  // interval at which to read Temp/Humi sensor, in milli seconds
float tempCalOffsetDHT22     = TEMPERATURE_CALIBRATION_OFFSET_DHT22_DEFAULT;
float tempCalScaleDHT22      = TEMPERATURE_CALIBRATION_SCALE_DHT22_DEFAULT;
float humCalOffsetDHT22      = HUMIDITY_CALIBRATION_OFFSET_DHT22_DEFAULT;
float humCalScaleDHT22       = HUMIDITY_CALIBRATION_SCALE_DHT22_DEFAULT;
int     reportingStrategyDHT22 = REPORTING_STRATEGY_DHT22_DEFAULT;
boolean sendTHUpdateDHT22      = false;

// Values read from AM2320 sensor
float temp_c_oldAM2320;                 // Temperature in Centigrade, earlier reading
float temp_c_newAM2320;                 // Temperature in Centigrade, latest reading
float humidity_oldAM2320;               // Relative humidity in %age, earlier reading
float humidity_newAM2320;               // Relative humidity in %age, latest reading
float hic_oldAM2320;                    // Heat Index in Centigrade, earlier reading
float hic_newAM2320;                    // Heat Index in Centigrade, latest reading
unsigned long previousMillisAM2320 = 0; // previous time value, so routine is non blocking
unsigned long readIntervalTempHumiAM2320 = DEFAULT_TH_SENSOR_INTERVAL_AM2320;  // interval at which to read Temp/Humi sensor, in milli seconds
float tempCalOffsetAM2320     = TEMPERATURE_CALIBRATION_OFFSET_AM2320_DEFAULT;
float tempCalScaleAM2320      = TEMPERATURE_CALIBRATION_SCALE_AM2320_DEFAULT;
float humCalOffsetAM2320      = HUMIDITY_CALIBRATION_OFFSET_AM2320_DEFAULT;
float humCalScaleAM2320       = HUMIDITY_CALIBRATION_SCALE_AM2320_DEFAULT;
int     reportingStrategyAM2320 = REPORTING_STRATEGY_AM2320_DEFAULT;
boolean sendTHUpdateAM2320      = false;

boolean bBrokerPresent    = false;


#define MQTT_VERSION MQTT_VERSION_3_1

#define MQTT_BROKER_IP_STRING_MAX_LEN           30
#define MQTT_BROKER_IP_DEFAULT                  "192.168.1.44"
#define MQTT_BROKER_PORT_DEFAULT                ((int)1883)
#define STA_NETWORK_SSID_DEFAULT                "SPEECHSYN"
#define STA_NETWORK_PASSWORD_DEFAULT            "PASSWORD"
#define AP_NETWORK_SSID_DEFAULT                 "SPEECHSYN"
#define AP_NETWORK_PASSWORD_DEFAULT             "PASSWORD"
#define NETWORK_CONNECTION_ATTEMPTS_DEFAULT     ((int)10)
#define MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT ((int)10)
#define NETWORK_SSID_STRING_MAX_LEN             32  
#define NETWORK_PASSWORD_STRING_MAX_LEN         40
#define CONNECTION_ATTEMPTS_MAX                 100
#define CONNECTION_ATTEMPTS_MIN                 0

char   mqtt_broker_ip[MQTT_BROKER_IP_STRING_MAX_LEN];
int    mqtt_broker_port;
String macStrForAPSSID;
char   sta_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
char   sta_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
char   ap_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
char   ap_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
int    network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
int    mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
#ifdef DEBUG_GENERAL
int    conDotCountNW   = 0; // Used to print a NW connecting dot each 500ms
int    conDotCountMQTT = 0; // Used to print a MQTT connecting dot each 500ms
#endif



// Topic to publish to, to request this device publish the status of its local software version (Generic Device, MAC Addr, Filename.ino). Caution : This a THSen Broadcast message.
const char* swVerTopic = "WFD/SpeechTH/SwVer/Command";

char swVerThisDeviceTopic[50];

//topic to publish request for software version
const char* swVerConfirmTopic = "WFD/SpeechTH/SwVer/Confirm";

//topic to subscribe to for the setting of system pot levels 0...255
const char* volumeTopic = "WFD/SpeechTH/1/Volume/Command";

//topic to publish to confirm the system pot has recieved a command. Volume Level 0 = Set, 1 = Error
const char* volumeConfirmTopic = "WFD/SpeechTH/1/Volume/Confirm";

//topic to subscribe to for the red led control signal
const char* redLightTopic = "WFD/SpeechTH/1/RedLed/Command";

//topic to subscribe to for the blue led control signal
const char* blueLightTopic = "WFD/SpeechTH/1/BlueLed/Command";

//topic to publish to confirm the led has recieved a command. Payload 0 = Led Off, 1 = Red Led On, 2 = Blue Led On, 5 = Error
const char* lightConfirmTopic = "WFD/SpeechTH/1/Led/Confirm";

//topic to subscribe to for the phonemes to speak. '370D130123020B0308002E0C000B00' = Steven Quinn
const char* hexPhonemesTopic = "WFD/SpeechTH/1/HexPhonemes/Command";

//topic to publish to confirm the phonemes have been spoken. Payload 0 = Done, 1 = Busy, 6 = Error, uneven number of hex phonemes
const char* hexPhonemesConfirmTopic = "WFD/SpeechTH/1/HexPhonemes/Confirm";

//topic to subscribe to for the phonemes to speak. 'SS,TT2,IY,PA2,VV,PA3,NN1,PA4,KK3,PA1,WW,IH,PA1,NN1,PA1' = Steven Quinn
const char* phonemesTopic = "WFD/SpeechTH/1/Phonemes/Command";

//topic to publish to confirm the phonemes have been spoken. Payload 0 = Done, 2 = Busy, 5 = Error, bad/unrecognised phoneme
const char* phonemesConfirmTopic = "WFD/SpeechTH/1/Phonemes/Confirm";

//topic to subscribe to for the fixed phrase to speak. Payload is index number of phrase to speak
const char* fixedPhraseTopic = "WFD/SpeechTH/1/FixedPhrase/Command";

//topic to publish to confirm the fixed phrases have been spoken. Payload 0 = Done, 3 = Busy, 5 = Error, unrecognised phrase index
const char* fixedPhraseConfirmTopic = "WFD/SpeechTH/1/FixedPhrase/Confirm";

// ToDo, requires MySQL database connection
//topic to subscribe to for the speaking of a sigle word. Word is looked up in database
const char* wordTopic = "WFD/SpeechTH/1/Word/Command";

// ToDo, requires MySQL database connection
//topic to publish to confirm the word been spoken. Payload 0 = Done, 4 = Busy, 5 = Error, unrecognised word, not is database
const char* wordConfirmTopic = "WFD/SpeechTH/1/Word/Confirm";

// Topic to subscribe to, to receive publication of this device's software version. In form (Generic Device, MAC Addr, Filename.ino)
//const char* swVerConfirmTopic = "WFD/SpeechTH/SwVer/Confirm";

// Topic to publish to, to control the status of this device's local led state
const char* lightSystemTopic = "WFD/SpeechTH/1/Led/Command/1";

// Topic to subscribe to, to receive confirmation that this device has recieved a Led control command
const char* lightSystemConfirmTopic = "WFD/SpeechTH/1/Led/Confirm/1";

// Topic to subscribe to, to receive publication of the status of this devices local temperature
const char* temperatureAM2320Topic = "WFD/SpeechTH/1/TempStatus/1";

// Topic to subscribe to, to receive publication of the status of this devices local humidity
const char* humidityAM2320Topic = "WFD/SpeechTH/1/HumdStatus/1";

// Topic to subscribe to, to receive publication of the status of this devices local heat index
const char* heatIndexAM2320Topic = "WFD/SpeechTH/1/HeatIndStatus/1";

// Topic to subscribe to, to receive publication of the status of this devices local temperature
const char* temperatureDHT22Topic = "WFD/SpeechTH/1/TempStatus/2";

// Topic to subscribe to, to receive publication of the status of this devices local humidity
const char* humidityDHT22Topic = "WFD/SpeechTH/1/HumdStatus/2";

// Topic to subscribe to, to receive publication of the status of this devices local heat index
const char* heatIndexDHT22Topic = "WFD/SpeechTH/1/HeatIndStatus/2";

// Topic to subscribe to, to request this device publish the status of its local RSSI for SSID
const char* rssiTopic = "WFD/SpeechTH/1/RSSILev";

// Topic to subscribe to, to receive publication of the status of this devices  local RSSI in dBm
const char* rssiConfirmTopic = "WFD/SpeechTH/1/RSSILev/Confirm";

// Topic to publish to, to request this device re-read all the values stored in it's local SD filing system (specifically CALIBRATION_PARAMETERS_FILE_AM2320). Response; 0 = Done, Error = 1
// Response is sent via 'sdConfirmTopic'
const char* sdReadAM2320Topic = "WFD/SpeechTH/1/SD/Read/Command/1";

// Topic to publish to, to request this device re-read all the values stored in it's local SD filing system (specifically CALIBRATION_PARAMETERS_FILE_DHT22). Response; 0 = Done, Error = 1
// Response is sent via 'sdConfirmTopic'
const char* sdReadDHT22Topic = "WFD/SpeechTH/1/SD/Read/Command/2";

// Topic to publish to, to request this device store a new Temperature Calibration Zero Offset in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 3
// Temperature command parameter in units of degrees celcius is a float in range -x.x to y.y
// Response is sent via 'sdConfirmTopic'
const char* sdTemperatureZeroOffsetAM2320Topic = "WFD/SpeechTH/1/SD/CalVal/Temperature/ZeroOff/Command/1";

// Topic to publish to, to request this device store a new Temperature Calibration Scaling Factor in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 5
// Temperature command parameter, unitless multiplication factor is a float in range 1.0 to x.x
// Response is sent via 'sdConfirmTopic'
const char* sdTemperatureScalingFactorAM2320Topic = "WFD/SpeechTH/1/SD/CalVal/Temperature/ScaleFact/Command/1";

// Topic to publish to, to request this device store a new Humidity Calibration Zero Offset in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 2
// Humidity command parameter in units of percent is a float in range -x.x to y.y
// Response is sent via 'sdConfirmTopic'
const char* sdHumidityZeroOffsetAM2320Topic = "WFD/SpeechTH/1/SD/CalVal/Humidity/ZeroOff/Command/1";

// Topic to publish to, to request this device store a new Humidity Calibration Scaling Factor in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 4
// Humidity command parameter, unitless multiplication factor is a float in range 1.0 to x.x
// Response is sent via 'spiffsConfirmTopic'
const char* sdHumidityScalingFactorAM2320Topic = "WFD/SpeechTH/1/SD/CalVal/Humidity/ScaleFact/Command/1";

// Topic to publish to, to request this device store a new Temperature Calibration Zero Offset in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 3
// Temperature command parameter in units of degrees celcius is a float in range -x.x to y.y
// Response is sent via 'sdConfirmTopic'
const char* sdTemperatureZeroOffsetDHT22Topic = "WFD/SpeechTH/1/SD/CalVal/Temperature/ZeroOff/Command/2";

// Topic to publish to, to request this device store a new Temperature Calibration Scaling Factor in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 5
// Temperature command parameter, unitless multiplication factor is a float in range 1.0 to x.x
// Response is sent via 'sdConfirmTopic'
const char* sdTemperatureScalingFactorDHT22Topic = "WFD/SpeechTH/1/SD/CalVal/Temperature/ScaleFact/Command/2";

// Topic to publish to, to request this device store a new Humidity Calibration Zero Offset in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 2
// Humidity command parameter in units of percent is a float in range -x.x to y.y
// Response is sent via 'sdConfirmTopic'
const char* sdHumidityZeroOffsetDHT22Topic = "WFD/SpeechTH/1/SD/CalVal/Humidity/ZeroOff/Command/2";

// Topic to publish to, to request this device store a new Humidity Calibration Scaling Factor in it's local SD filing system. Response; 0 = Done, Error = 1, Bad Float Format = 4
// Humidity command parameter, unitless multiplication factor is a float in range 1.0 to x.x
// Response is sent via 'spiffsConfirmTopic'
const char* sdHumidityScalingFactorDHT22Topic = "WFD/SpeechTH/1/SD/CalVal/Humidity/ScaleFact/Command/2";

// Topic to publish to, to request this device to publish the value of a given entry in the CALIBRATION_PARAMETERS_FILE_AM2320 
// SD Send command parameter = 1..n, where n is the value in the CALIBRATION_PARAMETERS_FILE_AM2320 file to return. It effectively 
// represents a given line number and responds in freeform text 
// Response is sent via 'sdConfirmTopic'
// Temperature Zero Offset AM2320  = 1
// Temperature Scale Factor AM2320 = 2
// Humidity Zero Offset AM2320     = 3
// Humidity Scale Factor AM2320    = 4
// Reporting Strategy AM2320       = 5
//
const char* sdSendAM2320Topic = "WFD/SpeechTH/1/SD/Send/1";

// Topic to publish to, to request this device to publish the value of a given entry in the CALIBRATION_PARAMETERS_FILE_DHT22
// SD Send command parameter = 1..n, where n is the value in the CALIBRATION_PARAMETERS_FILE_DHT22 file to return. It effectively 
// represents a given line number and responds in freeform text 
// Response is sent via 'sdConfirmTopic'
// Temperature Zero Offset DHT22   = 1
// Temperature Scale Factor DHT22  = 2
// Humidity Zero Offset DHT22      = 3
// Humidity Scale Factor DHT22     = 4
// Reporting Strategy DHT22        = 5
//
const char* sdSendDHT22Topic = "WFD/SpeechTH/1/SD/Send/2";

// Topic to publish to, to request this device store new Security Values in it's local SD filing system. 
// Responses; 
// 0  = Done, 
// 1  = Failed to open SECURITY_PARAMETERS_FILE for write, 
// 6  = MQTT Broker IP address malformed,
// 7  = MQTT Broker Port number invalid,
// 8  = Network SSID or Network Password Wrong length,
// 9  = MQTT Broker Connection Attempts number invalid,
// 10 = MQTT Broker Connection Attempts out of range,
// 11 = Network Connection Attempts number invalid,
// 12 = Network Connection Attempts out of range,
// 13 = One or more items in the parameter string is missing
// Parameter is in the following form 'BrokerIPAddress,BrokerPort,MQTTBrokerConnectionAttempts,NetworkSSID,NetworkPassword,NetworkConnectionAttempts'
// Where;
// BrokerIPAddress : AAA.BBB.CCC.DDD dotted decimal form
// BrokerPort : Integer form. Typically 1883 for Mosquitto MQTT Broker
// MQTTBrokerConnectionAttempts : CONNECTION_ATTEMPTS_MIN ... CONNECTION_ATTEMPTS_MAX. 0 is a special case meaning keep retrying
// NetworkSSID : Free form text
// NetworkPassword : Free form text
// NetworkConnectionAttempts : Integer form. can be any value as this field is not implemented.
// 
// Response is sent via 'sdConfirmTopic'
const char* sdNewSecValsTopic  = "WFD/SpeechTH/1/SD/SecVals";

// Topic to subscribe to, to receive publication of response that a given SD command has received and executed 
const char* sdConfirmTopic = "WFD/SpeechTH/1/SD/Conf";

// Topic to publish to, to request this device control the reporting strategy 
// Reporting Strategy command parameter 0..60, where n is the value in minutes to send a temperature/humidity update. The value 0 means send whenever there is a change. Response; 0 = Done, Error in range = 1
const char* reportingStrategyAM2320Topic = "WFD/SpeechTH/1/RepStrat/1";

// Topic to publish to, to request this device control the reporting strategy 
// Reporting Strategy command parameter 0..60, where n is the value in minutes to send a temperature/humidity update. The value 0 means send whenever there is a change. Response; 0 = Done, Error in range = 1
const char* reportingStrategyDHT22Topic = "WFD/SpeechTH/1/RepStrat/2";

// Topic to publish a pass_response that a given Reporting Strategy command has received and executed
const char* reportingStrategyConfirmAM2320DHT22Topic = "WFD/SpeechTH/1/RepStrat/Conf";



// This line is here to cure the following compiler error;
//
extern  void callback(char* topic, byte* payload, unsigned int length);
//
//  XXXXXXXXX:35: error: 'callback' was not declared in this scope
//
//  PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);
//
// At the time of writing this code it was a known issue with Arduino IDE 1.6.8 and the 2.1.0 esp code this sketch fails to compile:
// TITLE : Function not declared in this scope #1881 
// URL   : https://github.com/esp8266/Arduino/issues/1881
//



const char* subscriptionsArray[] = {swVerThisDeviceTopic,swVerTopic, volumeTopic, lightSystemTopic, rssiTopic, sdReadAM2320Topic, sdReadDHT22Topic, sdHumidityZeroOffsetAM2320Topic, sdHumidityScalingFactorAM2320Topic, sdTemperatureZeroOffsetAM2320Topic, sdTemperatureScalingFactorAM2320Topic, sdHumidityZeroOffsetDHT22Topic, sdHumidityScalingFactorDHT22Topic, sdTemperatureZeroOffsetDHT22Topic, sdTemperatureScalingFactorDHT22Topic, sdSendAM2320Topic, sdSendDHT22Topic, reportingStrategyAM2320Topic, reportingStrategyDHT22Topic, sdNewSecValsTopic, redLightTopic, blueLightTopic, hexPhonemesTopic, phonemesTopic, fixedPhraseTopic, wordTopic};
int maxSubscriptions = 0;

#define WiFiConnected (WiFi.status() == WL_CONNECTED)
WiFiClient wifiClient;
PubSubClient MQTTclient(wifiClient);

String clientName;
const char* THIS_GENERIC_DEVICE = "esp8266";
String swVersion;


// Struct to hold a single timer instance
typedef struct tsTimerInstance {
  void (*tmrcallback)(void);   // Function called when timing period exceeded
  boolean bRunning;            // Flag, set with timer running
  unsigned long ulTimerPeriod; // Timing period in milliseconds
  unsigned long ulStartValue;  // Grab of value from millis() when timer was started, used to calculate elapsed time
} TimerInstance;

#define TOTAL_TIME_IN_MILLISECONDS(H, M, S) ((unsigned long)((((unsigned long)H)*60UL*60UL*1000UL)+(((unsigned long)M)*60UL*1000UL)+(((unsigned long)S)*1000UL)))
#define MAX_TIMERS                    3
#define PERIODIC_UPDATE_TIMER_AM2320  0
#define PERIODIC_UPDATE_TIMER_DHT22   1
#define LED_FLASH_TIMER               2

TimerInstance stiTimerArray[MAX_TIMERS];  // Array for holding all the active timer instances


#define FILE_VAR_INSTANCE_TYPE_STRING 0
#define FILE_VAR_INSTANCE_TYPE_FLOAT  1
#define FILE_VAR_INSTANCE_TYPE_INT    2
#define FILE_VAR_INSTANCE_TYPE_BOOL   3

typedef struct tsFileVarInstance {
  int iVarType;  
  void *ptrVar;
} FileVarInstance;


FileVarInstance SecurityVarArray[] = 
{
  {FILE_VAR_INSTANCE_TYPE_STRING, (void *)mqtt_broker_ip                  },
  {FILE_VAR_INSTANCE_TYPE_INT,    (void *)&mqtt_broker_port               },
  {FILE_VAR_INSTANCE_TYPE_INT,    (void *)&mqtt_broker_connection_attempts},
  {FILE_VAR_INSTANCE_TYPE_STRING, (void *)sta_network_ssid                },
  {FILE_VAR_INSTANCE_TYPE_STRING, (void *)sta_network_password            },
  {FILE_VAR_INSTANCE_TYPE_INT,    (void *)&network_connection_attempts    }
};



FileVarInstance CalibrationVarArrayAM2320[] = 
{
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&tempCalOffsetAM2320     },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&tempCalScaleAM2320      },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&humCalOffsetAM2320      },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&humCalScaleAM2320       },
  {FILE_VAR_INSTANCE_TYPE_INT,   (void *)&reportingStrategyAM2320 }
};



FileVarInstance CalibrationVarArrayDHT22[] = 
{
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&tempCalOffsetDHT22     },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&tempCalScaleDHT22      },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&humCalOffsetDHT22      },
  {FILE_VAR_INSTANCE_TYPE_FLOAT, (void *)&humCalScaleDHT22       },
  {FILE_VAR_INSTANCE_TYPE_INT,   (void *)&reportingStrategyDHT22 }
};


FileVarInstance ConfigurationVarArray[] = 
{
  {FILE_VAR_INSTANCE_TYPE_BOOL,  (void *)&bDigitalPotPresence  },
  {FILE_VAR_INSTANCE_TYPE_INT,   (void *)&iDigitalPotSetting   },
  {FILE_VAR_INSTANCE_TYPE_BOOL,  (void *)&bAnnounceSystemReady }
};


typedef enum {
   eSENSORSTATE_INIT         = 0,
   eSENSORSTATE_NO_CONFIG    = 1,
   eSENSORSTATE_PENDING_NW   = 2,
   eSENSORSTATE_PENDING_MQTT = 3,
   eSENSORSTATE_ACTIVE       = 4
} eSENSORSTATE;

eSENSORSTATE eSENSORSTATE_STATE = eSENSORSTATE_INIT;

const char* nDNSHostName = "SPEECHSVR";


ESP8266WebServer server(80);
static bool hasSD = false;
File uploadFile;
//IPAddress APIPAddress (192,168,1,1);
//IPAddress APNWMask (255,255,255,0);
IPAddress tmpAPIPAddress;

const char* SPEECH_SNYTH_CONFIG_HOME_PAGE = "index.htm";
const char* SPEECH_SNYTH_WEB_PAGE = "index1.htm";
String strIndexFile;

typedef enum {
    eWEBSVR_IDLE                          = 0,
    eWEBSVR_SPEECH_SNYTH_CONFIG_HOME_PAGE = 1,
    eWEBSVR_SPEECH_SNYTH_WEB_PAGE         = 2
} eWebServerHttpGetHandlerState;

eWebServerHttpGetHandlerState volatile WebServerHttpGetHandlerCurrentState = eWEBSVR_IDLE;


#define LED_FLASH_PERIOD   500 // Time of flash period in mS
#define FLASH_SEQUENCE_MAX 11  // The maximum number of definable states a flash sequence can have

typedef enum {
   eLEDFLASH_NO_CONFIG    = 0,
   eLEDFLASH_PENDING_NW   = 1,
   eLEDFLASH_PENDING_MQTT = 2,
   eLEDFLASH_OFF          = 3,
   eLEDFLASH_SEQUENCE_END = 4
} eLEDFLASHSTATE;

eLEDFLASHSTATE eLEDFLASHSTATE_STATE = eLEDFLASH_OFF;
int iFlashSequenceIndex = 0;

char cFlashProfiles[][FLASH_SEQUENCE_MAX] = {
  "1000000000",  // No Config
  "1010000000",  // Pending NW
  "1111111111",  // Pending MQTT
  "0000000000",  // Off
};

#ifdef DEBUG_STATE_CHANGE
char printStateChangeBuf[200];
const char *SensorStates[]= {
  "eSENSORSTATE_INIT",
  "eSENSORSTATE_NO_CONFIG",
  "eSENSORSTATE_PENDING_NW",
  "eSENSORSTATE_PENDING_MQTT",
  "eSENSORSTATE_ACTIVE"
};

char *printStateChange(eSENSORSTATE ThisState, eSENSORSTATE NextState, const char *InThisFunction)
{
    printStateChangeBuf[0] = 0x00;
    sprintf(printStateChangeBuf,"State Change %s => %s : Within '%s'",SensorStates[ThisState],SensorStates[NextState],InThisFunction);
    return printStateChangeBuf;
}

#define SHOW_UPDATED_STATE(t,n,f) Serial.println(printStateChange((t),(n),(f)))
#endif

#ifdef DEBUG_LEDFLASH
char printLedStateChangeBuf[200];
const char *LedStates[]= {
  "eLEDFLASH_NO_CONFIG",
  "eLEDFLASH_PENDING_NW",
  "eLEDFLASH_PENDING_MQTT",
  "eLEDFLASH_OFF",
  "eLEDFLASH_SEQUENCE_END"
};

char *printLedStateChange(eLEDFLASHSTATE ThisState, eLEDFLASHSTATE NextState, const char *InThisFunction)
{
    printLedStateChangeBuf[0] = 0x00;
    sprintf(printLedStateChangeBuf,"LED State Change %s => %s : Within '%s'",LedStates[ThisState],LedStates[NextState],InThisFunction);
    return printLedStateChangeBuf;
}

#define SHOW_UPDATED_LED_STATE(t,n,f) Serial.println(printLedStateChange((t),(n),(f)))
#endif



//##############################################
//###                                        ###
//###          Function Declarations         ###
//###                                        ###
//##############################################
void handleSpeech(void);
void checkTemperatureAndHumidity(void);
void readAM2320(void);
void readDHT22(void);
void callback(char* topic, byte* payload, unsigned int length);
void grabParm(char **ptrToParmString, String *recipientString);
int fileWrite(File f, FileVarInstance *fviArray, int iTotalParametersToWrite);
int fileRead(File f, FileVarInstance *fviArray, int iTotalParametersToRead);
void readCalibrationValuesDHT22(void);
void readCalibrationValuesMA2302(void);
void readNetworkSecurityParameters(void);
void readConfigurationParameters(void);
void connectMQTT(void);
void makeSubscriptions(void);
String macToStr(const uint8_t* mac, boolean addColons);
void timer_create(int iTimerNumber, unsigned long ulTimerPeriod, void (*callbackfn)(void));
void timer_update(void);
void timer_start(int iTimerNumber);
void timer_stop(int iTimerNumber);
void timer_reset(int iTimerNumber);
boolean timer_isRunning(int iTimerNumber);
void timer_change_period(int iTimerNumber, unsigned long ulTimerPeriod);
void ledFlashTimerCallback(void);
void periodicUpdateTimerCallbackDHT22(void);
void periodicUpdateTimerCallbackAM2320(void);
void returnOK(String mess);
void returnFail(String mess);
bool loadFromSD(String path);
void handleFileUpload(void);
void deleteRecursive(String path);
void handleDelete(void);
void handleCreate(void);
void printDirectory(void);
void handleNetworkConfig(void);
void handleNotFound(void);
boolean isFloat(String tString);
boolean isValidNumber(String str);
bool isValidIpv4Address(char *st);
void initSP0256SpeechProcessor(void);
void testSP0256SpeechProcessor(void);
void resetSP0256SpeechProcessor(void);
void presentByteSP0256SpeechProcessor(uint8_t uPhoneme);
void writeByteSP0256SpeechProcessor(uint8_t uPhoneme);
boolean isBusySP0256SpeechProcessor(void);
boolean isNum(char n);
boolean isLetter(char l);
boolean isHex(char h);
uint8_t hexVal(char v);
uint8_t getPhoneme(const char* phonemeString);
void handleFixedPhrase(int whichPhrase, eSpeechProcessorState nextState);
void handleHttpGet(void);
void handleButtonPresses(void);
float convertCtoF(float c);
float convertFtoC(float f);
float computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit);


void setup(void) {
  // Start the serial line for debugging
  // This is enabled or the TX/RX port will require 10K pull ups to stop oscillations of the I/Ps which makes the ESP8266-01 pull more current
  //#ifdef DEBUG_GENERAL
  Serial.begin(115200);
  delay(100);
  //#endif

  // Set up I2C and configure I/O
  mcp23017.begin(MCP23017_ADDR,I2C_SDA,I2C_SCL);
  mcp23017.pinMode(RED_LED, OUTPUT);
  mcp23017.pinMode(BLUE_LED, OUTPUT);
  mcp23017.digitalWrite(RED_LED, HIGH);  // Led off
  mcp23017.digitalWrite(BLUE_LED, HIGH); // Led off

  mcp23017.pinMode(lightPin, OUTPUT);
  mcp23017.digitalWrite(lightPin, HIGH);  // Led off

  // Initialise speech processor
  initSP0256SpeechProcessor();
  SpeechProcessorCurrentState = eSPEECH_IDLE;
  maxPhonemes = sizeof(allophoneAddressTable)/sizeof(tAllophoneAddressTableStruct);
  maxFixedPhrases = sizeof(PHONEME_FIXED_PHRASE_STORE)/sizeof(uint8_t *);
  bDigitalPotPresence  = DEFAULT_DIGITAL_POT_PRESENCE;
  iDigitalPotSetting = DEFAULT_DIGITAL_POT_SETTING;
  bAnnounceSystemReady = DEFAULT_ANNOUNCE_SYSTEM_READY;

 
  #ifdef DEBUG_SPEECH
  Serial.print("maxFixedPhrases = ");
  Serial.println(maxFixedPhrases);
  #endif  

  // Initialise AM2320 sensor
  am2320.begin();
  humidity_oldAM2320            = 0.0;
  humidity_newAM2320            = 0.0;
  temp_c_oldAM2320              = 0.0;  
  temp_c_newAM2320              = 0.0;  
  hic_oldAM2320                 = 0.0;     
  hic_newAM2320                 = 0.0; 
  humCalOffsetAM2320            = HUMIDITY_CALIBRATION_OFFSET_AM2320_DEFAULT;
  humCalScaleAM2320             = HUMIDITY_CALIBRATION_SCALE_AM2320_DEFAULT;
  tempCalOffsetAM2320           = TEMPERATURE_CALIBRATION_OFFSET_AM2320_DEFAULT;
  tempCalScaleAM2320            = TEMPERATURE_CALIBRATION_SCALE_AM2320_DEFAULT;
  reportingStrategyAM2320       = REPORTING_STRATEGY_AM2320_DEFAULT;
  readIntervalTempHumiAM2320    = DEFAULT_TH_SENSOR_INTERVAL_AM2320;

  
  // Initialise DHT sensor
  dht.begin();
  humidity_oldDHT22            = 0.0;
  humidity_newDHT22            = 0.0;
  temp_c_oldDHT22              = 0.0;  
  temp_c_newDHT22              = 0.0;  
  hic_oldDHT22                 = 0.0;     
  hic_newDHT22                 = 0.0; 
  humCalOffsetDHT22            = HUMIDITY_CALIBRATION_OFFSET_DHT22_DEFAULT;
  humCalScaleDHT22             = HUMIDITY_CALIBRATION_SCALE_DHT22_DEFAULT;
  tempCalOffsetDHT22           = TEMPERATURE_CALIBRATION_OFFSET_DHT22_DEFAULT;
  tempCalScaleDHT22            = TEMPERATURE_CALIBRATION_SCALE_DHT22_DEFAULT;
  reportingStrategyDHT22       = REPORTING_STRATEGY_DHT22_DEFAULT;
  readIntervalTempHumiDHT22    = DEFAULT_TH_SENSOR_INTERVAL_DHT22;
  whichDHTParameterToRead = eDHTParameterTemperature;


  // Generate client name based on MAC address
  clientName = THIS_GENERIC_DEVICE;
  clientName += '-';
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac,true);
  macStrForAPSSID = macToStr(mac,false);
  macStrForAPSSID.trim();
  sprintf(swVerThisDeviceTopic,"WFD/%s/SwVer/Command",macToStr(mac, true).c_str());

  swVersion = THIS_GENERIC_DEVICE;
  swVersion += ',';
  swVersion += macToStr(mac,true);
  swVersion += ',';
  swVersion += __FILENAME__;
  #ifdef DEBUG_GENERAL  
  Serial.print("Client Name : ");
  Serial.println(clientName);
  Serial.print("SW Version : ");
  Serial.println(swVersion);
  #endif

  // Set up default security parameters. If all else fails so this device can become an AP.
  strcpy(ap_network_ssid,AP_NETWORK_SSID_DEFAULT);
  strcat(ap_network_ssid,macStrForAPSSID.c_str());
  strcpy(ap_network_password,AP_NETWORK_PASSWORD_DEFAULT);
  strcpy(mqtt_broker_ip, MQTT_BROKER_IP_DEFAULT);
  mqtt_broker_port = MQTT_BROKER_PORT_DEFAULT;
  mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
  strcpy(sta_network_ssid, STA_NETWORK_SSID_DEFAULT);
  strcpy(sta_network_password, STA_NETWORK_PASSWORD_DEFAULT);
  network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
  bBrokerPresent = true;

  // Set up MQTT auto topic subscription
  maxSubscriptions = sizeof(subscriptionsArray)/sizeof(const char*);  

  // Start filing subsystem
  SD.begin();
  delay(2000);
  
  // Try to read the calibration values file. If missing this will set defaults
  readCalibrationValuesDHT22();
  // Try to read the calibration values file. If missing this will set defaults
  readCalibrationValuesAM2320();
  // Try to read the security paramaters file. If missing this will set the ssid and p/w for the AP
  readNetworkSecurityParameters();
  // Try to read the configuration paramaters file. If missing this will set the defaults for volume level etc.
  readConfigurationParameters();
  
  // Set up initial conditions
  eSENSORSTATE_STATE = eSENSORSTATE_INIT;
  WiFi.mode(WIFI_OFF);


  // Set up timers for AM2320
  sendTHUpdateAM2320 = false;
  timer_create(PERIODIC_UPDATE_TIMER_AM2320, TOTAL_TIME_IN_MILLISECONDS(0, reportingStrategyAM2320, 0), periodicUpdateTimerCallbackAM2320);   
  if (reportingStrategyAM2320>0)
    timer_start(PERIODIC_UPDATE_TIMER_AM2320);

  // Set up timers for DHT22
  sendTHUpdateDHT22 = false;
  timer_create(PERIODIC_UPDATE_TIMER_DHT22, TOTAL_TIME_IN_MILLISECONDS(0, reportingStrategyDHT22, 0), periodicUpdateTimerCallbackDHT22);   
  if (reportingStrategyDHT22>0)
    timer_start(PERIODIC_UPDATE_TIMER_DHT22);

  iFlashSequenceIndex = 0;
  eLEDFLASHSTATE_STATE = eLEDFLASH_OFF;
  timer_create(LED_FLASH_TIMER, LED_FLASH_PERIOD, ledFlashTimerCallback);   
  //timer_create(LED_FLASH_TIMER, TOTAL_TIME_IN_MILLISECONDS(0, LED_FLASH_PERIOD, 0), ledFlashTimerCallback);   
  timer_start(LED_FLASH_TIMER);

  // Set up HTTP server. 
  strIndexFile = SPEECH_SNYTH_CONFIG_HOME_PAGE;
  WebServerHttpGetHandlerCurrentState = eWEBSVR_SPEECH_SNYTH_CONFIG_HOME_PAGE;
  server.on("/0", HTTP_GET, handleHttpGet);
  server.on("/list", HTTP_GET, printDirectory);
  server.on("/edit", HTTP_DELETE, handleDelete);
  server.on("/edit", HTTP_PUT, handleCreate);
  server.on("/edit", HTTP_POST, handleFileUpload);
  //server.on("/edit", HTTP_POST, [](){ returnOK(); }, handleFileUpload);
  server.onNotFound(handleNotFound);
  server.begin();
  #ifdef DEBUG_WEB
  Serial.println("handleNetworkConfig");
  Serial.println("HTTP server started");
  #endif

  if (bDigitalPotPresence)
  { 
    uint16_t u16Result;
    u16Result = digiPot.potConnectAll(MCP4561_WIPER_0);
    #ifdef DEBUG_GENERAL
    if (u16Result != MCP4561_SUCCESS)
      Serial.println("Failed to connect system pot");
    else
      Serial.println("System pot connected");

    #endif
    u16Result = digiPot.writeVal(MCP4561_VOL_WIPER_0, (uint16_t) iDigitalPotSetting);
    #ifdef DEBUG_GENERAL
    if (u16Result != MCP4561_SUCCESS)
      Serial.println("Failed to set system volume");
    else {
      Serial.print("Set system volume to : ");
      Serial.println(iDigitalPotSetting);
    }
    #endif
  }

  delay(2000);
}


void loop(void){
  timer_update(); // Update timers
  checkTemperatureAndHumidity(); // Read Temp and Humidity sensor
  //MDNS.update();   // Check for any mDNS queries and send responses

  switch (eSENSORSTATE_STATE) {
    case eSENSORSTATE_INIT : //
           WiFi.mode(WIFI_OFF);
           delay(1000);
           if ((SD.exists(SECURITY_PARAMETERS_FILE)) && (bBrokerPresent)) {
              eSENSORSTATE_STATE = eSENSORSTATE_PENDING_NW;
              eLEDFLASHSTATE_STATE = eLEDFLASH_PENDING_NW;
              #ifdef DEBUG_STATE_CHANGE
              SHOW_UPDATED_STATE(eSENSORSTATE_INIT,eSENSORSTATE_PENDING_NW,"loop");
              #endif
              WiFi.mode(WIFI_AP_STA);
              delay(1000);
              // Read the security paramaters file. 
              readNetworkSecurityParameters();
              // Start STA wifi subsystem
              WiFi.begin((const char *)sta_network_ssid, (const char *)sta_network_password);
              #ifdef DEBUG_GENERAL
              Serial.println("Switching to AP_STA Mode. SecVals Found");
              Serial.print("Connecting to "); Serial.println(sta_network_ssid);
              conDotCountNW = 0;  
              #endif
           } else {
              #ifdef DEBUG_STATE_CHANGE
              SHOW_UPDATED_STATE(eSENSORSTATE_INIT,eSENSORSTATE_NO_CONFIG,"loop");
              #endif
              eSENSORSTATE_STATE = eSENSORSTATE_NO_CONFIG;
              eLEDFLASHSTATE_STATE = eLEDFLASH_NO_CONFIG;
              WiFi.mode(WIFI_AP);
              delay(1000);
              #ifdef DEBUG_GENERAL
              if (bBrokerPresent)
                Serial.println("Switching to AP Mode. No SecVals found");
              else
                Serial.println("Switching to AP Mode. No MQTT Broker found");
              #endif
           }

           // Start AP wifi subsystem
           WiFi.encryptionType(ENC_TYPE_WEP);
           //WiFi.softAPConfig(APIPAddress,APIPAddress,APNWMask);
           WiFi.softAP((const char *)ap_network_ssid, (const char *)ap_network_password);
          
           // Late binding for MQTT client
           MQTTclient.setServer((const char *)mqtt_broker_ip, mqtt_broker_port); 
           MQTTclient.setCallback(callback);
           hasSD = true;

           tmpAPIPAddress = WiFi.softAPIP();
           #ifdef DEBUG_GENERAL
           Serial.print("AP IP address: "); Serial.println(tmpAPIPAddress);
           #endif    
          
           //if (MDNS.begin(nDNSHostName, APIPAddress)) {
           if (MDNS.begin(nDNSHostName)) {
             MDNS.addService("http", "tcp", 80);
             #ifdef DEBUG_MDNS
             Serial.println("MDNS responder started");
             Serial.print("You can now connect to http://");
             Serial.print(nDNSHostName);
             Serial.println(".local");
             #endif
           } else {
             #ifdef DEBUG_MDNS
             Serial.println("MDNS responder failed to start");
             #endif
           }

           // Set up HTTP server
           strIndexFile = SPEECH_SNYTH_CONFIG_HOME_PAGE;
           WebServerHttpGetHandlerCurrentState = eWEBSVR_SPEECH_SNYTH_CONFIG_HOME_PAGE;
           #ifdef DEBUG_WEB
           Serial.println("handleNetworkConfig");
           //Serial.println("HTTP server started");
           #endif

           break;

    case eSENSORSTATE_NO_CONFIG  : // Run only as an access point to allow the user to reconfigure to new network
           server.handleClient();
           delay(10); 
           break;

    case eSENSORSTATE_PENDING_NW : // Run as an access point to allow the user to reconfigure to new network and as a station trying to connnect to NW
           server.handleClient();
           delay(10); 
           if (WiFiConnected) {
              // Start wifi subsystem
              //WiFi.mode(WIFI_STA);  // Switch off access point
              //#ifdef DEBUG_GENERAL
              //Serial.println();
              //Serial.println("Switching to STA Mode. Now WiFi is connected.");
              //#endif
              //WiFi.begin((const char *)sta_network_ssid, (const char *)sta_network_password);
              eSENSORSTATE_STATE = eSENSORSTATE_PENDING_MQTT;
              eLEDFLASHSTATE_STATE = eLEDFLASH_PENDING_MQTT;
              #ifdef DEBUG_STATE_CHANGE
              SHOW_UPDATED_STATE(eSENSORSTATE_PENDING_NW,eSENSORSTATE_PENDING_MQTT,"loop");
              #endif
              
              //print out some more debug once connected
              #ifdef DEBUG_GENERAL
              Serial.println("WiFi connected");  
              Serial.print("IP address: ");
              Serial.println(WiFi.localIP());
              #endif
           } else {
              #ifdef DEBUG_GENERAL
              if (conDotCountNW > 50)
                  conDotCountNW = 0;
              if (conDotCountNW == 0)
                Serial.print(".");
              conDotCountNW++;  
              #endif
           }
           break;
    
    case eSENSORSTATE_PENDING_MQTT : // Try tp connect to MQTT Broker
           readCalibrationValuesDHT22();
           readCalibrationValuesAM2320();
           connectMQTT();
           break;
    
    case eSENSORSTATE_ACTIVE : // Run as a WiFi client in active mode
           // Reconnect if connection is lost
           if (!MQTTclient.connected()) {
            #ifdef DEBUG_STATE_CHANGE
            Serial.println();
            Serial.println("Switching to AP_STA Mode. As MQTT has disconnected.");
            #endif
            WiFi.mode(WIFI_AP_STA);
            delay(1000);
            WiFi.encryptionType(ENC_TYPE_WEP);
            //WiFi.softAPConfig(APIPAddress,APIPAddress,APNWMask);
            WiFi.softAP((const char *)ap_network_ssid, (const char *)ap_network_password);
            
            tmpAPIPAddress = WiFi.softAPIP();
            #ifdef DEBUG_GENERAL
            Serial.print("AP IP address: "); Serial.println(tmpAPIPAddress);
            #endif    
            connectMQTT();
           } else //maintain MQTT connection
            MQTTclient.loop();

           handleSpeech();
           // Delay to allow ESP8266 WIFI functions to run
           //yield();
           delay(10); 
           break;
  }
}


void handleSpeech(void)
{

  server.handleClient();
  
  //MUST delay to allow ESP8266 WIFI functions to run
  //yield();

  switch (SpeechProcessorCurrentState)
  {
    case eSPEECH_IDLE :
            // Do nothing
            break;
    case eSPEECH_SPEAKING_HEX_PHONEMES :
    case eSPEECH_SPEAKING_PHONEMES :
    case eSPEECH_SPEAKING_FIXED_PHRASE :
    case eSPEECH_HTTP :
            if (!isBusySP0256SpeechProcessor()) { // Only send a new byte when SP0256 Speech Processor has fininshed speaking the current byte
              writeByteSP0256SpeechProcessor(phonemeBufferStore[currentPhoneme++]);  // Send next byte
              if (phonemeBufferStore[currentPhoneme] == PHONEME_END_MARKER) { // If we are at the end of the sentence then drop back to idle
                switch (SpeechProcessorCurrentState) {
                    case eSPEECH_SPEAKING_HEX_PHONEMES : MQTTclient.publish(hexPhonemesConfirmTopic, "0");
                                                         mcp23017.digitalWrite(BLUE_LED, HIGH);
                                                         break;
                    case eSPEECH_SPEAKING_PHONEMES     : MQTTclient.publish(phonemesConfirmTopic, "0");
                                                         mcp23017.digitalWrite(BLUE_LED, HIGH);
                                                         break;
                    case eSPEECH_SPEAKING_FIXED_PHRASE : MQTTclient.publish(fixedPhraseConfirmTopic, "0");
                                                         mcp23017.digitalWrite(BLUE_LED, HIGH);
                                                         break;
                    case eSPEECH_HTTP                  : mcp23017.digitalWrite(RED_LED, HIGH);
                }
                SpeechProcessorCurrentState = eSPEECH_IDLE;
                #ifdef DEBUG_SPEECH
                Serial.println("Speech done");
                #endif
              }
            }
            break;
    case eSPEECH_SPEAKING_WORDS :
            break;
    case eSPEECH_QUERYING_DATABASE :
            break;
    default : ;
  }  
}


void checkTemperatureAndHumidity(void)
{
  readAM2320();
  readDHT22();
}


void readAM2320(void)
{
  String s1, s2, s3;
  bool bFreshData = false;

  if ((SpeechProcessorCurrentState != eSPEECH_SPEAKING_HEX_PHONEMES) && // Only handle callback functionality if the ESP8266-12E is not in the process of speaking
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_PHONEMES) && 
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_FIXED_PHRASE) &&
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_WORDS) &&
      (SpeechProcessorCurrentState != eSPEECH_HTTP))
  {

    // Wait at least 10 seconds between measurements.
    // if the difference between the current time and last time you read
    // the sensor is bigger than the interval you set, read the sensor
    // Works better than delay for things happening elsewhere also
    unsigned long currentMillis = millis();
  
    if(currentMillis - previousMillisAM2320 >= readIntervalTempHumiAM2320 ) {
      // save the last time you read the sensor 
      previousMillisAM2320 = currentMillis;   

      temp_c_newAM2320 = am2320.readTemperature();     // Read temperature as Centigrade
      temp_c_newAM2320 *= tempCalScaleAM2320;
      temp_c_newAM2320 += tempCalOffsetAM2320;
      #ifdef DEBUG_GENERAL
      Serial.println("Reading Temperature from AM2320 sensor!");
      #endif

      if (isnan(temp_c_newAM2320)) {
        #ifdef DEBUG_GENERAL
        Serial.println("Failed to read Temperature from AM2320 sensor!");
        #endif
        return;
      }

      humidity_newAM2320 = am2320.readHumidity();         // Read humidity (percent)
      humidity_newAM2320 *= humCalScaleAM2320;
      humidity_newAM2320 += humCalOffsetAM2320;
      #ifdef DEBUG_GENERAL
      Serial.println("Reading Humidity from AM2320 sensor!");
      #endif

      if (isnan(humidity_newAM2320)) {
        #ifdef DEBUG_GENERAL
        Serial.println("Failed to read Humidity from AM2320 sensor!");
        #endif
        return;
      }
    }
  
    if ((reportingStrategyAM2320 == 0) and (bFreshData)) {
      bFreshData = false;
      #ifdef DEBUG_GENERAL
      if ((temp_c_newAM2320 != temp_c_oldAM2320)     ||
          (humidity_newAM2320 != humidity_oldAM2320) ||
          (hic_newAM2320 != hic_oldAM2320)) {
        Serial.println("AM2320 Rep Strat ==0");
        Serial.print("AM2320 Humidity: ");
        Serial.print(humidity_newAM2320);
        Serial.print(" %\t");
        Serial.print("AM2320 Temperature: ");
        Serial.print(temp_c_newAM2320);
        Serial.print(" *C\t");
        Serial.print("AM2320 Heat index: ");
        Serial.print(hic_newAM2320);
        Serial.println(" *C");
      } 
      #endif
      
      if (temp_c_newAM2320 != temp_c_oldAM2320)
      {
        s1 = String(temp_c_newAM2320);
        if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          MQTTclient.publish(temperatureAM2320Topic, s1.c_str());  
        temp_c_oldAM2320 = temp_c_newAM2320;
      }
  
      if (humidity_newAM2320 != humidity_oldAM2320)
      {
        s1 = String(humidity_newAM2320);
        if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          MQTTclient.publish(humidityAM2320Topic, s1.c_str());        
        humidity_oldAM2320 = humidity_newAM2320;
      }
  
      if (hic_newAM2320 != hic_oldAM2320)
      {
        s1 = String(hic_newAM2320);
        if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          MQTTclient.publish(heatIndexAM2320Topic, s1.c_str());        
        hic_oldAM2320 = hic_newAM2320;
      }
  
    } else  {
      if (sendTHUpdateAM2320 == true) {
        sendTHUpdateAM2320 = false;
        
        if (!isnan(temp_c_newAM2320)) {
          if (temp_c_newAM2320 != temp_c_oldAM2320)
          {
            s1 = String(temp_c_newAM2320);
            if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
              MQTTclient.publish(temperatureAM2320Topic, s1.c_str());        
            temp_c_oldAM2320 = temp_c_newAM2320;
          }
        } else {
          s1 = String(temp_c_oldAM2320);
          if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
            MQTTclient.publish(temperatureAM2320Topic, s1.c_str());        
        }
        
        if (!isnan(humidity_newAM2320)) {
          if (humidity_newAM2320 != humidity_oldAM2320)
          {
            s1 = String(humidity_newAM2320);
            if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
              MQTTclient.publish(humidityAM2320Topic, s1.c_str());        
            humidity_oldAM2320 = humidity_newAM2320;
          }
        } else {
          s1 = String(humidity_oldAM2320);
          if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
            MQTTclient.publish(humidityAM2320Topic, s1.c_str());        
        }
        
        if ((!isnan(humidity_newAM2320)) && (!isnan(temp_c_newAM2320))) {
          hic_newAM2320 = computeHeatIndex(temp_c_newAM2320, humidity_newAM2320, false);   // Compute heat index in Celsius 
          if (hic_newAM2320 != hic_oldAM2320)
          {
            s1 = String(hic_newAM2320);
            if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
              MQTTclient.publish(heatIndexAM2320Topic, s1.c_str());        
            hic_oldAM2320 = hic_newAM2320;
          }
        } else {
            s1 = String(hic_oldAM2320);
            if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
              MQTTclient.publish(heatIndexAM2320Topic, s1.c_str());        
        }
        #ifdef DEBUG_GENERAL
        Serial.println("AM2320 Rep Strat <> 0");
        Serial.print("AM2320 Humidity: ");
        Serial.print(humidity_newAM2320);
        Serial.print(" %\t");
        Serial.print("AM2320 Temperature: ");
        Serial.print(temp_c_newAM2320);
        Serial.print(" *C\t");
        Serial.print("AM2320 Heat index: ");
        Serial.print(hic_newAM2320);
        Serial.println(" *C");
        #endif
      }
    }
  }
}


void readDHT22(void)
{  
  String s1, s2, s3;
  bool bFreshData = false;

  if ((SpeechProcessorCurrentState != eSPEECH_SPEAKING_HEX_PHONEMES) && // Only handle callback functionality if the ESP8266-12E is not in the process of speaking
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_PHONEMES) && 
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_FIXED_PHRASE) &&
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_WORDS) &&
      (SpeechProcessorCurrentState != eSPEECH_HTTP))
  {

    // Wait at least 10 seconds between measurements.
    // if the difference between the current time and last time you read
    // the sensor is bigger than the interval you set, read the sensor
    // Works better than delay for things happening elsewhere also
    unsigned long currentMillis = millis();
  
    if(currentMillis - previousMillisDHT22 >= readIntervalTempHumiDHT22 ) {
      // save the last time you read the sensor 
      previousMillisDHT22 = currentMillis;   



      // Reading temperature for humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
        switch (whichDHTParameterToRead) {
        case eDHTParameterTemperature:
          whichDHTParameterToRead = eDHTParameterHumidity;
          temp_c_newDHT22 = dht.readTemperature();     // Read temperature as Centigrade
          temp_c_newDHT22 *= tempCalScaleDHT22;
          temp_c_newDHT22 += tempCalOffsetDHT22;
          #ifdef DEBUG_GENERAL
          Serial.println("Reading Temperature from DHT sensor!");
          #endif
  
          if (isnan(temp_c_newDHT22)) {
            #ifdef DEBUG_GENERAL
            Serial.println("Failed to read Temperature from DHT sensor!");
            #endif
            return;
          }
          break;
        case eDHTParameterHumidity:
          whichDHTParameterToRead = eDHTParameterTemperature;
          humidity_newDHT22 = dht.readHumidity();          // Read humidity (percent)
          humidity_newDHT22 *= humCalScaleDHT22;
          humidity_newDHT22 += humCalOffsetDHT22;
          #ifdef DEBUG_GENERAL
          Serial.println("Reading Humidity from DHT sensor!");
          #endif
  
          if (isnan(humidity_newDHT22)) {
            #ifdef DEBUG_GENERAL
            Serial.println("Failed to read Humidity from DHT sensor!");
            #endif
            return;
          }
          break;
      }
      hic_newDHT22 = dht.computeHeatIndex(temp_c_newDHT22, humidity_newDHT22, false);   // Compute heat index in Celsius 
      // Check if any reads failed and exit early (to try again).
      bFreshData = true;
    }
  
    if ((reportingStrategyDHT22 == 0) and (bFreshData)) {
      bFreshData = false;
      #ifdef DEBUG_GENERAL
      if ((temp_c_newDHT22 != temp_c_oldDHT22)     ||
          (humidity_newDHT22 != humidity_oldDHT22) ||
          (hic_newDHT22 != hic_oldDHT22)) {
        Serial.println("Rep Strat ==0");
        Serial.print("Humidity: ");
        Serial.print(humidity_newDHT22);
        Serial.print(" %\t");
        Serial.print("Temperature: ");
        Serial.print(temp_c_newDHT22);
        Serial.print(" *C\t");
        Serial.print("Heat index: ");
        Serial.print(hic_newDHT22);
        Serial.println(" *C");
      } 
      #endif
      
      if (temp_c_newDHT22 != temp_c_oldDHT22)
      {
        s1 = String(temp_c_newDHT22);
        if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          MQTTclient.publish(temperatureDHT22Topic, s1.c_str());  
        temp_c_oldDHT22 = temp_c_newDHT22;
      }
  
      if (humidity_newDHT22 != humidity_oldDHT22)
      {
        s1 = String(humidity_newDHT22);
        if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          MQTTclient.publish(humidityDHT22Topic, s1.c_str());        
        humidity_oldDHT22 = humidity_newDHT22;
      }
  
      if (hic_newDHT22 != hic_oldDHT22)
      {
        s1 = String(hic_newDHT22);
        if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          MQTTclient.publish(heatIndexDHT22Topic, s1.c_str());        
        hic_oldDHT22 = hic_newDHT22;
      }
  
    } else  {
      if (sendTHUpdateDHT22 == true) {
        sendTHUpdateDHT22 = false;
        
        if (!isnan(temp_c_newDHT22)) {
          if (temp_c_newDHT22 != temp_c_oldDHT22)
          {
            s1 = String(temp_c_newDHT22);
            if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
              MQTTclient.publish(temperatureDHT22Topic, s1.c_str());        
            temp_c_oldDHT22 = temp_c_newDHT22;
          }
        } else {
          s1 = String(temp_c_oldDHT22);
          if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
            MQTTclient.publish(temperatureDHT22Topic, s1.c_str());        
        }
        
        if (!isnan(humidity_newDHT22)) {
          if (humidity_newDHT22 != humidity_oldDHT22)
          {
            s1 = String(humidity_newDHT22);
            if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
              MQTTclient.publish(humidityDHT22Topic, s1.c_str());        
            humidity_oldDHT22 = humidity_newDHT22;
          }
        } else {
          s1 = String(humidity_oldDHT22);
          if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
            MQTTclient.publish(humidityDHT22Topic, s1.c_str());        
        }
        
        if ((!isnan(humidity_newDHT22)) && (!isnan(temp_c_newDHT22))) {
          hic_newDHT22 = dht.computeHeatIndex(temp_c_newDHT22, humidity_newDHT22, false);   // Compute heat index in Celsius 
          if (hic_newDHT22 != hic_oldDHT22)
          {
            s1 = String(hic_newDHT22);
            if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
              MQTTclient.publish(heatIndexDHT22Topic, s1.c_str());        
            hic_oldDHT22 = hic_newDHT22;
          }
        } else {
            s1 = String(hic_oldDHT22);
            if (eSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
              MQTTclient.publish(heatIndexDHT22Topic, s1.c_str());        
        }
        #ifdef DEBUG_GENERAL
        Serial.println("DHT22 Rep Strat <> 0");
        Serial.print("DHT22 Humidity: ");
        Serial.print(humidity_newDHT22);
        Serial.print(" %\t");
        Serial.print("DHT22 Temperature: ");
        Serial.print(temp_c_newDHT22);
        Serial.print(" *C\t");
        Serial.print("DHT22 Heat index: ");
        Serial.print(hic_newDHT22);
        Serial.println(" *C");
        #endif
      }
    }
  }
}




void callback(char* topic, byte* payload, unsigned int length) {

  //convert topic to string to make it easier to work with
  String topicStr = topic; 
  char tmpCharBuf[length+1];
  char tmpCharBuf1[10];
  int tmpInt  = 0;
  int tmpInt1 = 0;
  int tmpInt2 = 0;

  for (int i=0;i<length;i++) 
    tmpCharBuf[i] = ((char)payload[i]);
  tmpCharBuf[length] = 0x00;
  
  //Print out some debugging info
  #ifdef DEBUG_GENERAL  
  Serial.println("Callback update.");
  Serial.print("Topic: ");
  Serial.println(topicStr);
  Serial.print("Payload: ");
  Serial.println(tmpCharBuf);
  #endif

  if ((SpeechProcessorCurrentState != eSPEECH_SPEAKING_HEX_PHONEMES) && // Only handle callback functionality if the ESP8266-12E is not in the process of speaking
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_PHONEMES) && 
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_FIXED_PHRASE) &&
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_WORDS) &&
      (SpeechProcessorCurrentState != eSPEECH_HTTP))
  {
    //Print out some debugging info
    #ifdef DEBUG_MQTT 
    Serial.println("Callback update.");
    Serial.print("Topic: ");
    Serial.println(topicStr);
    Serial.print("Payload: ");
    Serial.println(tmpBuf);
    #endif

    //turn the light off if the payload is '0' and publish to the MQTT server a confirmation message
    if (strcmp(lightSystemTopic,topic)== 0) {
      if(payload[0] == '1'){ //turn the light on if the payload is '1' and publish the confirmation 
        mcp23017.digitalWrite(lightPin, LOW);
        //digitalWrite(lightPin, LOW);
        MQTTclient.publish(lightSystemConfirmTopic, "On");
      } else if (payload[0] == '0'){ //turn the light off if the payload is '0' and publish the confirmation
        mcp23017.digitalWrite(lightPin, HIGH);
        //digitalWrite(lightPin, HIGH);
        MQTTclient.publish(lightSystemConfirmTopic, "Off");
      } else {
        MQTTclient.publish(lightSystemConfirmTopic, "Err");
      }
      return;
    }

    
    if (strcmp(swVerTopic,topic)== 0) {
      MQTTclient.publish(swVerConfirmTopic, swVersion.c_str());
      return;
    }  
  
    if (strcmp(swVerThisDeviceTopic,topic)== 0) {
      MQTTclient.publish(swVerConfirmTopic, swVersion.c_str());
      return;
    }  
  
    // handle red led topic, send MQTT confirmation via lightConfirmTopic  
    if (strcmp(redLightTopic,topic)== 0) { // Note inverted logic as this led is a pull down
      if(payload[0] == '1'){ //turn the red led on if the payload is '1' and publish the confirmation 
        // Call to I2C device Red Led on
        mcp23017.digitalWrite(RED_LED, LOW);
        MQTTclient.publish(lightConfirmTopic, "1");
      } else if (payload[0] == '0'){ //turn the red led off if the payload is '0' and publish the confirmation
        // Call to I2C device Red Led off
        mcp23017.digitalWrite(RED_LED, HIGH); 
        MQTTclient.publish(lightConfirmTopic, "0");
      } else {
        MQTTclient.publish(lightConfirmTopic, "5");
      }
      return;
    }
  
    // handle blue led topic, send MQTT confirmation via lightConfirmTopic
    if (strcmp(blueLightTopic,topic)== 0) {  // Note inverted logic as this led is a pull down
      if(payload[0] == '1'){ //turn the blue led on if the payload is '1' and publish the confirmation 
        // Call to I2C device Blue Led on
        mcp23017.digitalWrite(BLUE_LED, LOW);
        MQTTclient.publish(lightConfirmTopic, "2");
      } else if (payload[0] == '0'){ //turn the blue led off if the payload is '0' and publish the confirmation
        // Call to I2C device Blue Led off
        mcp23017.digitalWrite(BLUE_LED, HIGH);
        MQTTclient.publish(lightConfirmTopic, "0");
      } else {
        MQTTclient.publish(lightConfirmTopic, "5");
      }
      return;
    }

  
    // handle RSSI topic, send MQTT confirmation via rssiConfirmTopic  
    if (strcmp(rssiTopic,topic)== 0) {
      int32_t rssi = WiFi.RSSI();
      sprintf(tmpCharBuf,"%ld",rssi);
      MQTTclient.publish(rssiConfirmTopic, tmpCharBuf);
      return;
    }    
  
  /*
    // SD Handlers
    // Re-initialise the filing system
    if (strcmp(spiffsInitTopic,topic)== 0) {
      if (SD.format())
        MQTTclient.publish(spiffsConfirmTopic, "0");
      else
        MQTTclient.publish(spiffsConfirmTopic, "1");
      return;
    }  
  */
    // Re-read all stored calibration values
    if (strcmp(sdReadAM2320Topic,topic)== 0) {
      String s;
      // open file for readting
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_AM2320, SD_FILE_READ_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "1");
        return;
      } else {
        fileRead(f, CalibrationVarArrayAM2320,(int)(sizeof(CalibrationVarArrayAM2320)/sizeof(tsFileVarInstance)));
        f.close();
        MQTTclient.publish(sdConfirmTopic, "0");
        return;
      }
    }  
    
    // Re-read all stored calibration values
    if (strcmp(sdReadDHT22Topic,topic)== 0) {
      String s;
      // open file for readting
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_DHT22, SD_FILE_READ_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "1");
        return;
      } else {
        fileRead(f, CalibrationVarArrayDHT22,(int)(sizeof(CalibrationVarArrayDHT22)/sizeof(tsFileVarInstance)));
        f.close();
        MQTTclient.publish(sdConfirmTopic, "0");
        return;
      }
    }  
    
  
    // Write Humidity calibration value to cal file and update local Humidity Cal offset
    if (strcmp(sdHumidityZeroOffsetAM2320Topic,topic)== 0) {
      // test to see value is a float
      String s;
      String tmpHumidityCalVal = tmpCharBuf;
      if (!isFloat(tmpHumidityCalVal))
      {
        MQTTclient.publish(sdConfirmTopic, "2");
        return;
      }
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_AM2320, SD_FILE_WRITE_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "1");
        return;
      } else {
        humCalOffsetAM2320 = tmpHumidityCalVal.toFloat();
        fileWrite(f, CalibrationVarArrayAM2320,(int)(sizeof(CalibrationVarArrayAM2320)/sizeof(tsFileVarInstance)));
        f.close();
        MQTTclient.publish(sdConfirmTopic, "0");
        return;
      }
    }  
    
    // Write Humidity calibration value to cal file and update local Humidity Cal offset
    if (strcmp(sdHumidityZeroOffsetDHT22Topic,topic)== 0) {
      // test to see value is a float
      String s;
      String tmpHumidityCalVal = tmpCharBuf;
      if (!isFloat(tmpHumidityCalVal))
      {
        MQTTclient.publish(sdConfirmTopic, "2");
        return;
      }
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_DHT22, SD_FILE_WRITE_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "1");
        return;
      } else {
        humCalOffsetDHT22 = tmpHumidityCalVal.toFloat();
        fileWrite(f, CalibrationVarArrayDHT22,(int)(sizeof(CalibrationVarArrayDHT22)/sizeof(tsFileVarInstance)));
        f.close();
        MQTTclient.publish(sdConfirmTopic, "0");
        return;
      }
    }  
    
    // Write Humidity calibration value to cal file and update local Humidity Cal scaling factor
    if (strcmp(sdHumidityScalingFactorAM2320Topic,topic)== 0) {
      // test to see value is a float
      String s;
      String tmpHumidityCalVal = tmpCharBuf;
      if (!isFloat(tmpHumidityCalVal))
      {
        MQTTclient.publish(sdConfirmTopic, "4");
        return;
      }
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_AM2320, SD_FILE_WRITE_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "1");
        return;
      } else {
        humCalScaleAM2320 = tmpHumidityCalVal.toFloat();
        fileWrite(f, CalibrationVarArrayAM2320,(int)(sizeof(CalibrationVarArrayAM2320)/sizeof(tsFileVarInstance)));
        f.close();
        MQTTclient.publish(sdConfirmTopic, "0");
        return;
      }
    }  
    
    // Write Humidity calibration value to cal file and update local Humidity Cal scaling factor
    if (strcmp(sdHumidityScalingFactorDHT22Topic,topic)== 0) {
      // test to see value is a float
      String s;
      String tmpHumidityCalVal = tmpCharBuf;
      if (!isFloat(tmpHumidityCalVal))
      {
        MQTTclient.publish(sdConfirmTopic, "4");
        return;
      }
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_DHT22, SD_FILE_WRITE_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "1");
        return;
      } else {
        humCalScaleDHT22 = tmpHumidityCalVal.toFloat();
        fileWrite(f, CalibrationVarArrayDHT22,(int)(sizeof(CalibrationVarArrayDHT22)/sizeof(tsFileVarInstance)));
        f.close();
        MQTTclient.publish(sdConfirmTopic, "0");
        return;
      }
    }  
    
  
    // Write Temperature calibration value to cal file and update local Temperature Cal offset
    if (strcmp(sdTemperatureZeroOffsetAM2320Topic,topic)== 0) {
      // test to see value is a float
      String s;
      String tmpTemperatureCalVal = tmpCharBuf;
      if (!isFloat(tmpTemperatureCalVal))
      {
        MQTTclient.publish(sdConfirmTopic, "3");
        return;
      }
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_AM2320, SD_FILE_WRITE_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "1");
        return;
      } else {
        tempCalOffsetAM2320 = tmpTemperatureCalVal.toFloat();
        fileWrite(f, CalibrationVarArrayAM2320,(int)(sizeof(CalibrationVarArrayAM2320)/sizeof(tsFileVarInstance)));
        f.close();
        MQTTclient.publish(sdConfirmTopic, "0");
        return;
      }
    }  
  
    // Write Temperature calibration value to cal file and update local Temperature Cal offset
    if (strcmp(sdTemperatureZeroOffsetDHT22Topic,topic)== 0) {
      // test to see value is a float
      String s;
      String tmpTemperatureCalVal = tmpCharBuf;
      if (!isFloat(tmpTemperatureCalVal))
      {
        MQTTclient.publish(sdConfirmTopic, "3");
        return;
      }
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_DHT22, SD_FILE_WRITE_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "1");
        return;
      } else {
        tempCalOffsetDHT22 = tmpTemperatureCalVal.toFloat();
        fileWrite(f, CalibrationVarArrayDHT22,(int)(sizeof(CalibrationVarArrayDHT22)/sizeof(tsFileVarInstance)));
        f.close();
        MQTTclient.publish(sdConfirmTopic, "0");
        return;
      }
    }  
  
    // Write Temperature calibration value to cal file and update local Temperature Cal scaling factor
    if (strcmp(sdTemperatureScalingFactorAM2320Topic,topic)== 0) {
      // test to see value is a float
      String s;
      String tmpTemperatureCalVal = tmpCharBuf;
      if (!isFloat(tmpTemperatureCalVal))
      {
        MQTTclient.publish(sdConfirmTopic, "5");
        return;
      }
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_AM2320, SD_FILE_WRITE_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "1");
        return;
      } else {
        tempCalScaleAM2320 = tmpTemperatureCalVal.toFloat();
        fileWrite(f, CalibrationVarArrayAM2320,(int)(sizeof(CalibrationVarArrayAM2320)/sizeof(tsFileVarInstance)));
        f.close();
        MQTTclient.publish(sdConfirmTopic, "0");
        return;
      }
    }  
  
    // Write Temperature calibration value to cal file and update local Temperature Cal scaling factor
    if (strcmp(sdTemperatureScalingFactorDHT22Topic,topic)== 0) {
      // test to see value is a float
      String s;
      String tmpTemperatureCalVal = tmpCharBuf;
      if (!isFloat(tmpTemperatureCalVal))
      {
        MQTTclient.publish(sdConfirmTopic, "5");
        return;
      }
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_DHT22, SD_FILE_WRITE_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "1");
        return;
      } else {
        tempCalScaleDHT22 = tmpTemperatureCalVal.toFloat();
        fileWrite(f, CalibrationVarArrayDHT22,(int)(sizeof(CalibrationVarArrayDHT22)/sizeof(tsFileVarInstance)));
        f.close();
        MQTTclient.publish(sdConfirmTopic, "0");
        return;
      }
    }  
  
    // Write Reporting Strategy value to cal file and update local Reporting Strategy variable
    if (strcmp(reportingStrategyAM2320Topic,topic)== 0) {
      // test to see value is a float
      String s;
      String tmpreportingStrategyAM2320Val = tmpCharBuf;
      #ifdef DEBUG_SD
      Serial.println(tmpreportingStrategyAM2320Val);
      #endif
      int tmpVal = tmpreportingStrategyAM2320Val.toInt();
      if ((tmpVal < LOWER_REPORTING_STRATEGY_VALUE) || (tmpVal > UPPER_REPORTING_STRATEGY_VALUE))
      {
        MQTTclient.publish(reportingStrategyConfirmAM2320DHT22Topic, "2");
        return;
      }
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_AM2320, SD_FILE_WRITE_MODE);
      if (!f) {
        MQTTclient.publish(reportingStrategyConfirmAM2320DHT22Topic, "1");
        return;
      } else {
        reportingStrategyAM2320=tmpVal;
        fileWrite(f, CalibrationVarArrayAM2320,(int)(sizeof(CalibrationVarArrayAM2320)/sizeof(tsFileVarInstance)));
        if (reportingStrategyAM2320 == 0)
          timer_stop(PERIODIC_UPDATE_TIMER_AM2320);
        else {
          timer_change_period(PERIODIC_UPDATE_TIMER_AM2320, TOTAL_TIME_IN_MILLISECONDS(0, reportingStrategyAM2320, 0));  
          timer_start(PERIODIC_UPDATE_TIMER_AM2320);                      
        }
        sendTHUpdateAM2320 = false;
        f.close();
        MQTTclient.publish(reportingStrategyConfirmAM2320DHT22Topic, "0");
        return;
      }
    }

    // Write Reporting Strategy value to cal file and update local Reporting Strategy variable
    if (strcmp(reportingStrategyDHT22Topic,topic)== 0) {
      // test to see value is a float
      String s;
      String tmpreportingStrategyDHT22Val = tmpCharBuf;
      #ifdef DEBUG_SD
      Serial.println(tmpreportingStrategyDHT22Val);
      #endif
      int tmpVal = tmpreportingStrategyDHT22Val.toInt();
      if ((tmpVal < LOWER_REPORTING_STRATEGY_VALUE) || (tmpVal > UPPER_REPORTING_STRATEGY_VALUE))
      {
        MQTTclient.publish(reportingStrategyConfirmAM2320DHT22Topic, "2");
        return;
      }
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_DHT22, SD_FILE_WRITE_MODE);
      if (!f) {
        MQTTclient.publish(reportingStrategyConfirmAM2320DHT22Topic, "1");
        return;
      } else {
        reportingStrategyDHT22=tmpVal;
        fileWrite(f, CalibrationVarArrayDHT22,(int)(sizeof(CalibrationVarArrayDHT22)/sizeof(tsFileVarInstance)));
        if (reportingStrategyDHT22 == 0)
          timer_stop(PERIODIC_UPDATE_TIMER_DHT22);
        else {
          timer_change_period(PERIODIC_UPDATE_TIMER_DHT22, TOTAL_TIME_IN_MILLISECONDS(0, reportingStrategyDHT22, 0));  
          timer_start(PERIODIC_UPDATE_TIMER_DHT22);                      
        }
        sendTHUpdateDHT22 = false;
        f.close();
        MQTTclient.publish(reportingStrategyConfirmAM2320DHT22Topic, "0");
        return;
      }
    }

  
    // Query a stored calibration value and publish this value
    if (strcmp(sdSendAM2320Topic,topic)== 0) {
      String s;
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_AM2320, SD_FILE_READ_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "Err no file");
        return;
      } else {
        //int x = os_scanf(tmpCharBuf, "%d", &tmpInt);
        tmpInt = String(tmpCharBuf).toInt();
        int iMaxCalParms = (int)(sizeof(CalibrationVarArrayAM2320)/sizeof(tsFileVarInstance));
        for (int i = 0; i <= iMaxCalParms; i++){
          s=f.readStringUntil('\n');
          s.trim();
          if (i == (tmpInt-1)) {
            MQTTclient.publish(sdConfirmTopic, s.c_str());
            return;
          }
          if (f.position()>= f.size()) break;
        }
        f.close();
        MQTTclient.publish(sdConfirmTopic, "Err Parm Not Found");
        return;
      }
    }   

    // Query a stored calibration value and publish this value
    if (strcmp(sdSendDHT22Topic,topic)== 0) {
      String s;
      // open file for writing
      File f = SD.open(CALIBRATION_PARAMETERS_FILE_DHT22, SD_FILE_READ_MODE);
      if (!f) {
        MQTTclient.publish(sdConfirmTopic, "Err no file");
        return;
      } else {
        //int x = os_scanf(tmpCharBuf, "%d", &tmpInt);
        tmpInt = String(tmpCharBuf).toInt();
        int iMaxCalParms = (int)(sizeof(CalibrationVarArrayDHT22)/sizeof(tsFileVarInstance));
        for (int i = 0; i <= iMaxCalParms; i++){
          s=f.readStringUntil('\n');
          s.trim();
          if (i == (tmpInt-1)) {
            MQTTclient.publish(sdConfirmTopic, s.c_str());
            return;
          }
          if (f.position()>= f.size()) break;
        }
        f.close();
        MQTTclient.publish(sdConfirmTopic, "Err Parm Not Found");
        return;
      }
    }   

  
    // Write new network security values to file and restart IoT device
    if (strcmp(sdNewSecValsTopic,topic)== 0) {
      char  *StrPtr = tmpCharBuf;
      char   tmp_mqtt_broker_ip[MQTT_BROKER_IP_STRING_MAX_LEN];
      int    tmp_mqtt_broker_port;
      int    tmp_mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
      char   tmp_sta_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
      char   tmp_sta_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
      int    tmp_network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
      String strMQTTBrokerIPAddress;
      String strMQTTBrokerPort;
      String strMQTTBrokerConnectionAttempts;
      String strNetworkSSID;
      String strNetworkPassword;
      String strNetworkConnectionAttempts;
  
      grabParm(&StrPtr,&strMQTTBrokerIPAddress);
      grabParm(&StrPtr,&strMQTTBrokerPort);
      grabParm(&StrPtr,&strMQTTBrokerConnectionAttempts);
      grabParm(&StrPtr,&strNetworkSSID);
      grabParm(&StrPtr,&strNetworkPassword);
      grabParm(&StrPtr,&strNetworkConnectionAttempts);
  
  /*    
      //sscanf(tmpCharBuf,"%s,%d,%d,%s,%s,%d",tmp_mqtt_broker_ip,&tmp_mqtt_broker_port,&tmp_mqtt_broker_connection_attempts,tmp_sta_network_ssid,tmp_sta_network_password,&tmp_network_connection_attempts);
      os_sprintf(tmpCharBuf,"%s,%d,%d,%s,%s,%d",tmp_mqtt_broker_ip,&tmp_mqtt_broker_port,&tmp_mqtt_broker_connection_attempts,tmp_sta_network_ssid,tmp_sta_network_password,&tmp_network_connection_attempts);
      strMQTTBrokerIPAddress = tmp_mqtt_broker_ip;
      strMQTTBrokerPort = tmp_mqtt_broker_port;
      strMQTTBrokerConnectionAttempts = tmp_mqtt_broker_connection_attempts;
      strNetworkSSID = tmp_sta_network_ssid;
      strNetworkPassword = tmp_sta_network_password;
      strNetworkConnectionAttempts = tmp_network_connection_attempts;
  */    
      #ifdef DEBUG_SECVALS
      Serial.print("SecValsMQTTBrokerIPAddress : "); Serial.println(strMQTTBrokerIPAddress);
      Serial.print("SecValsMQTTBrokerPort : "); Serial.println(strMQTTBrokerPort);
      Serial.print("SecValsMQTTBrokerConnectionAttempts : "); Serial.println(strMQTTBrokerConnectionAttempts);
      Serial.print("SecValsSTANetworkSSID : "); Serial.println(strNetworkSSID);
      Serial.print("SecValsSTANetworkPassword : "); Serial.println(strNetworkPassword);
      Serial.print("SecValsNetworkConnectionAttempts : "); Serial.println(strNetworkConnectionAttempts);
      #endif
  
      strMQTTBrokerIPAddress.trim();
      strMQTTBrokerPort.trim();
      strMQTTBrokerConnectionAttempts.trim();
      strNetworkSSID.trim();
      strNetworkPassword.trim();
      strNetworkConnectionAttempts.trim();
  
      if ((strMQTTBrokerIPAddress.length()          == 0) || 
          (strMQTTBrokerPort.length()               == 0) || 
          (strMQTTBrokerConnectionAttempts.length() == 0) || 
          (strNetworkSSID.length()                  == 0) || 
          (strNetworkPassword.length()              == 0) || 
          (strNetworkConnectionAttempts.length()    == 0)) {
        MQTTclient.publish(sdConfirmTopic, "13");
        return;
      }
      
      strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
      if (! isValidIpv4Address((char *)strMQTTBrokerIPAddress.c_str())) {
          MQTTclient.publish(sdConfirmTopic, "6");
          return;
      } else {
        //strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
        if (! isValidNumber(strMQTTBrokerPort)) {
          MQTTclient.publish(sdConfirmTopic, "7");
          return;
        } else {
          tmp_mqtt_broker_port = strMQTTBrokerPort.toInt();
          if (((strNetworkSSID.length() == 0)     || (strNetworkSSID.length() >= NETWORK_SSID_STRING_MAX_LEN)) || 
              ((strNetworkPassword.length() == 0) || (strNetworkPassword.length() >= NETWORK_PASSWORD_STRING_MAX_LEN))) {
              MQTTclient.publish(sdConfirmTopic, "8");
              return;
          } else {
            strcpy(tmp_sta_network_ssid,strNetworkSSID.c_str());
            strcpy(tmp_sta_network_password,strNetworkPassword.c_str());
    
            if (! isValidNumber(strMQTTBrokerConnectionAttempts)) {
              MQTTclient.publish(sdConfirmTopic, "9");
              return;
            } else {
              tmp_mqtt_broker_connection_attempts = strMQTTBrokerConnectionAttempts.toInt();
              if ((tmp_mqtt_broker_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_mqtt_broker_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
                MQTTclient.publish(sdConfirmTopic, "10");
                return;
              } else {
                if (! isValidNumber(strNetworkConnectionAttempts)) {
                  MQTTclient.publish(sdConfirmTopic, "11");
                  return;
                } else {
                  tmp_network_connection_attempts = strNetworkConnectionAttempts.toInt();
                  if ((tmp_network_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_network_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
                    MQTTclient.publish(sdConfirmTopic, "12");
                    return;
                  } else {
                    strcpy(mqtt_broker_ip,tmp_mqtt_broker_ip);
                    mqtt_broker_port = tmp_mqtt_broker_port;
                    mqtt_broker_connection_attempts = tmp_mqtt_broker_connection_attempts;
                    strcpy(sta_network_ssid,tmp_sta_network_ssid);
                    strcpy(sta_network_password,tmp_sta_network_password);
                    network_connection_attempts = tmp_network_connection_attempts;
                    // Save new network parameters
                    File f = SD.open(SECURITY_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
                    if (!f) {
                      MQTTclient.publish(sdConfirmTopic, "1");
                      return;
                    } else {
                      fileWrite(f, SecurityVarArray,(int)(sizeof(SecurityVarArray)/sizeof(tsFileVarInstance)));
                      f.close();
                      MQTTclient.publish(sdConfirmTopic, "0");
                      bBrokerPresent = true;
                      #ifdef DEBUG_STATE_CHANGE
                      SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_INIT,"callback, sdNewSecValsTopic");
                      #endif
                      eSENSORSTATE_STATE = eSENSORSTATE_INIT;
                      #ifdef DEBUG_SECVALS
                      Serial.print("SecValsMQTTBrokerIPAddress : "); Serial.println(mqtt_broker_ip);
                      Serial.print("SecValsMQTTBrokerPort : "); Serial.println(mqtt_broker_port);
                      Serial.print("SecValsMQTTBrokerConnectionAttempts : "); Serial.println(mqtt_broker_connection_attempts);
                      Serial.print("SecValsSTANetworkSSID : "); Serial.println(sta_network_ssid);
                      Serial.print("SecValsSTANetworkPassword : "); Serial.println(sta_network_password);
                      Serial.print("SecValsNetworkConnectionAttempts : "); Serial.println(network_connection_attempts);
                      #endif
                      return;
                    }      
                  }
                }
              }
            }
          }
        }
      }
    }  


    if (strcmp(volumeTopic,topic)== 0) {
      String newPotValue(tmpCharBuf);
      if (((uint16_t)newPotValue.toInt()) <= DEFAULT_DIGITAL_POT_SETTING_MAX) {
        int failure = digiPot.writeVal(MCP4561_VOL_WIPER_0,(uint16_t) newPotValue.toInt());
        //digiPot.enableOutput();
        if (!failure) 
          MQTTclient.publish(volumeConfirmTopic, "0");
        else  
          MQTTclient.publish(volumeConfirmTopic, "2");
      } else
        MQTTclient.publish(volumeConfirmTopic, "1");
      return;
    }  
  }

  // handle Hex Phonemes topic, send MQTT confirmation via phonemesConfirmTopic  
  if (strcmp(hexPhonemesTopic,topic)== 0) {
    if (SpeechProcessorCurrentState == eSPEECH_SPEAKING_HEX_PHONEMES)
    {
      MQTTclient.publish(hexPhonemesConfirmTopic, "2");
      return;
    } else {
      if ((length % 2) == 0) {
        char tmpChar;
        tmpInt = 0;
        tmpInt1 = length / 2;
        for(; tmpInt < tmpInt1; tmpInt++)
        {
          phonemeBufferStore[tmpInt] = (((hexVal(tmpCharBuf[tmpInt*2]))<<4) & 0xF0) | ((hexVal(tmpCharBuf[(tmpInt*2)+1])) & 0x0F);
        }
        if (phonemeBufferStore[tmpInt-1] != SP0256_PA1)
          phonemeBufferStore[tmpInt++] = SP0256_PA1;
        phonemeBufferStore[tmpInt] = PHONEME_END_MARKER;
        SpeechProcessorCurrentState = eSPEECH_SPEAKING_HEX_PHONEMES;
        currentPhoneme = 0;
        mcp23017.digitalWrite(BLUE_LED, LOW);
        testSP0256SpeechProcessor();
        writeByteSP0256SpeechProcessor(phonemeBufferStore[currentPhoneme++]);
        return;
      } else {
        MQTTclient.publish(hexPhonemesConfirmTopic, "6");    
        return;
      }
    }
  }

  // handle Phonemes topic, send MQTT confirmation via phonemesConfirmTopic  
  if (strcmp(phonemesTopic,topic)== 0) {
    if (SpeechProcessorCurrentState == eSPEECH_SPEAKING_PHONEMES)
    {
      MQTTclient.publish(phonemesConfirmTopic, "2");
      return;
    } else {
        uint8_t phonemeVal = 0x00;
        tmpInt = 0;
        tmpInt1 = 0;
        tmpInt2 = 0;
        while (tmpCharBuf[tmpInt]) {
          tmpCharBuf1[tmpInt1++] = tmpCharBuf[tmpInt];
          if ((tmpCharBuf[tmpInt + 1] == ',') || (tmpCharBuf[tmpInt + 1] == (char)0x00)) {
            if (tmpCharBuf[tmpInt + 1] == ',') 
              tmpInt++;
            tmpCharBuf1[tmpInt1] = 0x00;
            phonemeVal = getPhoneme(tmpCharBuf1);
            if (phonemeVal == PHONEME_UNRECOGNISED) {
                MQTTclient.publish(phonemesConfirmTopic, "5");
                return;          
            } else {
              phonemeBufferStore[tmpInt2++] = phonemeVal;
              tmpInt1 = 0;
            }   
          }
          tmpInt++;
        }
        
        if (phonemeBufferStore[tmpInt2-1] != SP0256_PA1)
          phonemeBufferStore[tmpInt2++] = SP0256_PA1;
        phonemeBufferStore[tmpInt2] = PHONEME_END_MARKER;
        SpeechProcessorCurrentState = eSPEECH_SPEAKING_PHONEMES;
        currentPhoneme = 0;
        mcp23017.digitalWrite(BLUE_LED, LOW);
        testSP0256SpeechProcessor();
        writeByteSP0256SpeechProcessor(phonemeBufferStore[currentPhoneme++]);
        return;
    }
  }

  // handle Fixed Phrases topic, send MQTT confirmation via fixedPhraseConfirmTopic  
  if (strcmp(fixedPhraseTopic,topic)== 0) {
    if (SpeechProcessorCurrentState == eSPEECH_SPEAKING_FIXED_PHRASE)
    {
      MQTTclient.publish(fixedPhraseConfirmTopic, "3");
      return;
    }
    tmpInt = String(tmpCharBuf).toInt();
    //sscanf(tmpBuf, "%d", &tmpInt);

    if ((tmpInt) < maxFixedPhrases)
    {
      handleFixedPhrase(tmpInt, eSPEECH_SPEAKING_FIXED_PHRASE);
    } else { // Error case as an unknown fixed phrase number was received
      MQTTclient.publish(fixedPhraseConfirmTopic, "5");
    }
    return;
  }

  // handle Word topic, send MQTT confirmation via wordConfirmTopic  
  // TODO
  if (strcmp(wordTopic,topic)== 0) {
    MQTTclient.publish(wordConfirmTopic, "1");
    return;
  }  
}



void grabParm(char **ptrToParmString, String *recipientString){
  #ifdef DEBUG_PARMGRAB
  Serial.print("**ptrToParmString : "); 
  #endif
  while (**ptrToParmString)
  {
    #ifdef DEBUG_PARMGRAB
    Serial.print(**ptrToParmString);
    #endif
    *recipientString += **ptrToParmString;
    (*ptrToParmString)++;
    if ((**ptrToParmString=='\0') || (**ptrToParmString==','))
    {
      if (**ptrToParmString==',')
        (*ptrToParmString)++;
      #ifdef DEBUG_PARMGRAB
      Serial.println();
      #endif
      return;
    }
  }
}


int fileWrite(File f, FileVarInstance *fviArray, int iTotalParametersToWrite) {
    #ifdef DEBUG_FILE_RW
    Serial.print("fileWrite, Filename : "); Serial.println(f.name());
    #endif
    for (int i = 0; i < iTotalParametersToWrite; i++){
      switch (fviArray[i].iVarType){
        case FILE_VAR_INSTANCE_TYPE_STRING :
                f.println((char *)(fviArray[i].ptrVar));
                break;
        case FILE_VAR_INSTANCE_TYPE_FLOAT :
                char tmpStr[10];
                dtostrf(*((float *)(fviArray[i].ptrVar)),5,2,tmpStr);
                f.println(tmpStr);
                break;
        case FILE_VAR_INSTANCE_TYPE_INT :
                f.println(*((int *)(fviArray[i].ptrVar)));
                break;
        case FILE_VAR_INSTANCE_TYPE_BOOL :
                f.println( ((*((int *)(fviArray[i].ptrVar)))?"1":"0") );
                break;
        default :
                return 1;
      }
  }
  return 0;
}



int fileRead(File f, FileVarInstance *fviArray, int iTotalParametersToRead) {
    String s;
    #ifdef DEBUG_FILE_RW
    Serial.print("fileRead, Filename : "); Serial.println(f.name());
    #endif
    for (int i = 0; i < iTotalParametersToRead; i++){
      s=f.readStringUntil('\n');
      s.trim();
      #ifdef DEBUG_FILE_RW
      Serial.print("Val ("); Serial.print(i); Serial.print(") : ");  Serial.println(s);
      #endif
      switch (fviArray[i].iVarType){
        case FILE_VAR_INSTANCE_TYPE_STRING :
                strcpy((char *)(fviArray[i].ptrVar),s.c_str());
                break;
        case FILE_VAR_INSTANCE_TYPE_FLOAT :
                *((float *)(fviArray[i].ptrVar)) = s.toFloat();
                break;
        case FILE_VAR_INSTANCE_TYPE_INT :
                *((int *)(fviArray[i].ptrVar)) = s.toInt();
                break;
        case FILE_VAR_INSTANCE_TYPE_BOOL :
                *((bool *)(fviArray[i].ptrVar)) = (s.toInt()==0?false:true);
                break;
        default : // Unknown data type
                return 1;
      }
  }
  return 0; // Successful completion
}




void readCalibrationValuesAM2320(void)
{
  // open file for reading
  String s;
  File f = SD.open(CALIBRATION_PARAMETERS_FILE_AM2320, SD_FILE_READ_MODE);
  if (!f) {
    tempCalOffsetAM2320      = TEMPERATURE_CALIBRATION_OFFSET_AM2320_DEFAULT;
    tempCalScaleAM2320       = TEMPERATURE_CALIBRATION_SCALE_AM2320_DEFAULT;
    humCalOffsetAM2320  = HUMIDITY_CALIBRATION_OFFSET_AM2320_DEFAULT;
    humCalScaleAM2320   = HUMIDITY_CALIBRATION_SCALE_AM2320_DEFAULT;
    reportingStrategyAM2320  = REPORTING_STRATEGY_AM2320_DEFAULT;
    #ifdef DEBUG_GENERAL
    Serial.println("Failed to read SD Cal Vals");
    #endif
  } else {
    fileRead(f, CalibrationVarArrayAM2320,(int)(sizeof(CalibrationVarArrayAM2320)/sizeof(tsFileVarInstance)));
    f.close();
  }
  #ifdef DEBUG_GENERAL
  Serial.println("readCalibrationValuesAM2320");
  s=tempCalOffsetAM2320;
  Serial.print("Temp Cal Offset : ");    Serial.println(s);
  s=tempCalScaleAM2320;
  Serial.print("Temp Cal Scale : ");    Serial.println(s);
  s=humCalOffsetAM2320;
  Serial.print("Humi Cal Offset : ");    Serial.println(s);
  s=humCalScaleAM2320;
  Serial.print("Humi Cal Scale : ");    Serial.println(s);
  s=reportingStrategyAM2320;
  Serial.print("Reporting Strategy : "); Serial.println(s);
  #endif
}


void readCalibrationValuesDHT22(void)
{
  // open file for reading
  String s;
  File f = SD.open(CALIBRATION_PARAMETERS_FILE_DHT22, SD_FILE_READ_MODE);
  if (!f) {
    tempCalOffsetDHT22     = TEMPERATURE_CALIBRATION_OFFSET_DHT22_DEFAULT;
    tempCalScaleDHT22      = TEMPERATURE_CALIBRATION_SCALE_DHT22_DEFAULT;
    humCalOffsetDHT22      = HUMIDITY_CALIBRATION_OFFSET_DHT22_DEFAULT;
    humCalScaleDHT22       = HUMIDITY_CALIBRATION_SCALE_DHT22_DEFAULT;
    reportingStrategyDHT22 = REPORTING_STRATEGY_DHT22_DEFAULT;
    #ifdef DEBUG_GENERAL
    Serial.println("Failed to read SD Cal Vals");
    #endif
  } else {
    fileRead(f, CalibrationVarArrayDHT22,(int)(sizeof(CalibrationVarArrayDHT22)/sizeof(tsFileVarInstance)));
    f.close();
  }
  #ifdef DEBUG_GENERAL
  Serial.println("readCalibrationValuesDHT22");
  s=tempCalOffsetDHT22;
  Serial.print("Temp Cal Offset : ");    Serial.println(s);
  s=tempCalScaleDHT22;
  Serial.print("Temp Cal Scale : ");    Serial.println(s);
  s=humCalOffsetDHT22;
  Serial.print("Humi Cal Offset : ");    Serial.println(s);
  s=humCalScaleDHT22;
  Serial.print("Humi Cal Scale : ");    Serial.println(s);
  s=reportingStrategyDHT22;
  Serial.print("Reporting Strategy : "); Serial.println(s);
  #endif
}



void readNetworkSecurityParameters(void){
  // open file for reading
  String s;
  File f = SD.open(SECURITY_PARAMETERS_FILE, SD_FILE_READ_MODE);
  if (!f) {
    strcpy(mqtt_broker_ip, MQTT_BROKER_IP_DEFAULT);
    mqtt_broker_port = MQTT_BROKER_PORT_DEFAULT;
    mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
    strcpy(sta_network_ssid, STA_NETWORK_SSID_DEFAULT);
    strcpy(sta_network_password, STA_NETWORK_PASSWORD_DEFAULT);
    network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
    #ifdef DEBUG_GENERAL
    Serial.println("Failed to read SD Sec Vals. Using defaults");
    #endif
  } else {
    fileRead(f, SecurityVarArray,(int)(sizeof(SecurityVarArray)/sizeof(tsFileVarInstance)));
    f.close();
  }
  strcpy(ap_network_ssid,AP_NETWORK_SSID_DEFAULT);
  strcat(ap_network_ssid,macStrForAPSSID.c_str());
  strcpy(ap_network_password,AP_NETWORK_PASSWORD_DEFAULT);
  #ifdef DEBUG_GENERAL
  Serial.println("readNetworkSecurityParameters");
  Serial.print("Broker IP : ");            Serial.println(mqtt_broker_ip);
  Serial.print("Broker Port : ");          Serial.println(mqtt_broker_port);
  Serial.print("Max MQTT Conn Atmpts : "); Serial.println(mqtt_broker_connection_attempts);
  Serial.print("STA SSID : ");             Serial.println(sta_network_ssid);
  Serial.print("STA PW : ");               Serial.println(sta_network_password);
  Serial.print("Max NW Conn Atmpts : ");   Serial.println(network_connection_attempts);
  Serial.print("AP SSID : ");              Serial.println(ap_network_ssid);
  Serial.print("AP PW : ");                Serial.println(ap_network_password);
  #endif
}


void readConfigurationParameters(void)
{
  // open file for reading
  String s;
  File f = SD.open(CONFIGURATION_PARAMETERS_FILE, SD_FILE_READ_MODE);
  if (!f) {
    bDigitalPotPresence  = DEFAULT_DIGITAL_POT_PRESENCE;
    iDigitalPotSetting   = DEFAULT_DIGITAL_POT_SETTING;
    bAnnounceSystemReady = DEFAULT_ANNOUNCE_SYSTEM_READY;
    #ifdef DEBUG_GENERAL
    Serial.println("Failed to read SD Conf Vals");
    #endif
  } else {
    fileRead(f, ConfigurationVarArray,(int)(sizeof(ConfigurationVarArray)/sizeof(tsFileVarInstance)));
    f.close();
  }
  #ifdef DEBUG_GENERAL
  Serial.println("readConfigurationValues");
  s=bDigitalPotPresence;
  Serial.print("Digital Pot Presence : ");        Serial.println(s);
  s=iDigitalPotSetting;
  Serial.print("Digital Pot Setting : ");        Serial.println(s);
  s=bAnnounceSystemReady;
  Serial.print("Announce System Ready : ");    Serial.println(s);
  #endif
}


void connectMQTT(void) {
  int connection_counts = 0;
  eSENSORSTATE tmpeSENSORSTATE_STATE;
  bBrokerPresent = true;
  #ifdef DEBUG_GENERAL
  conDotCountMQTT = 0;
  #endif
  tmpeSENSORSTATE_STATE = eSENSORSTATE_STATE; // Record the state connectMQTT was entered from. 
  // Make sure we are connected to WIFI before attemping to reconnect to MQTT
  eLEDFLASHSTATE_STATE = eLEDFLASH_PENDING_MQTT;
  timer_update(); // Update timers
  if(WiFi.status() == WL_CONNECTED){
    // Loop until we're reconnected to the MQTT server
    #ifdef DEBUG_STATE_CHANGE
    SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_PENDING_MQTT,"connectMQTT");
    #endif
    eSENSORSTATE_STATE = eSENSORSTATE_PENDING_MQTT;
    #ifdef DEBUG_GENERAL
    Serial.print("Attempting MQTT connection");
    #endif
    while (!MQTTclient.connected()) {
      #ifdef DEBUG_GENERAL
      if (conDotCountMQTT > 50)
          conDotCountMQTT = 0;
      if (conDotCountMQTT == 0)
        Serial.print(".");
      conDotCountMQTT++;  
      #endif
    
      timer_update(); // Update timers
      server.handleClient();      
      //if connected, subscribe to the topic(s) we want to be notified about
      if (MQTTclient.connect((char*) clientName.c_str())) {
        // Start wifi subsystem
        WiFi.mode(WIFI_STA);  // Switch off access point and turn into station only
        //WiFi.begin((const char *)sta_network_ssid, (const char *)sta_network_password);
        #ifdef DEBUG_GENERAL
        Serial.println();
        Serial.println("Switching to STA Mode. Now MQTT is connected.");
        #endif
        MQTTclient.publish(swVerConfirmTopic, swVersion.c_str());        
        makeSubscriptions();
        #ifdef DEBUG_STATE_CHANGE
        SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_ACTIVE,"connectMQTT");
        #endif

        // Set up HTTP server
        strIndexFile = SPEECH_SNYTH_WEB_PAGE;
        WebServerHttpGetHandlerCurrentState = eWEBSVR_SPEECH_SNYTH_WEB_PAGE;
        #ifdef DEBUG_WEB
        Serial.println("handleButtonPresses");
        //Serial.println("HTTP server started");
        #endif
        
        eSENSORSTATE_STATE = eSENSORSTATE_ACTIVE;
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
        if (bAnnounceSystemReady) 
          handleFixedPhrase((int) eSPEECH_FIXED_PHRASE_SYSTEM_READY, eSPEECH_SPEAKING_FIXED_PHRASE);

      } else { //otherwise print failed for debugging
        #ifdef DEBUG_GENERAL
        Serial.println("\tFailed."); 
        #endif
        //abort();
      }
      
      if(WiFi.status() != WL_CONNECTED) { // Catches a lost NW whilst looking for MQTT broker
        #ifdef DEBUG_STATE_CHANGE
        SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_INIT,"connectMQTT WiFi lost");
        #endif
        eSENSORSTATE_STATE = eSENSORSTATE_INIT;
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
        return;
      }
      
      if (eSENSORSTATE_STATE == eSENSORSTATE_INIT) // Catches the state where device is hung in MQTT pending mode with mqtt_broker_connection_attempts==0 and user sets new config via handleNetworkConfig
      {
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
        timer_update();
        return;
      }  
      
      if ((connection_counts >= mqtt_broker_connection_attempts) && (mqtt_broker_connection_attempts > 0))
      {
        #ifdef DEBUG_STATE_CHANGE
        SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_INIT,"connectMQTT con count");
        #endif
        eSENSORSTATE_STATE = eSENSORSTATE_INIT;
        eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
        timer_update();
        if (tmpeSENSORSTATE_STATE == eSENSORSTATE_ACTIVE)
          bBrokerPresent = true; // Force programme to go back to eSENSORSTATE_INIT state if MQTT Broker conn lost after having connected to the nw and broker at least once
        else
          bBrokerPresent = false; // Force programme to go to eSENSORSTATE_NO_CONFIG State if after MQTT connection attempts made and never having made an MQTT on this nw before
        return;
      }
      
      if (mqtt_broker_connection_attempts > 0)
        connection_counts++;
      delay(10);
    }
  } else { // catches a lost NW as the cause for an MQTT broker connection failure
    #ifdef DEBUG_STATE_CHANGE
    SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_INIT,"connectMQTT no WiFi at start");
    #endif
    eSENSORSTATE_STATE = eSENSORSTATE_INIT;
    eLEDFLASHSTATE_STATE = eLEDFLASH_SEQUENCE_END;
  }
}


void makeSubscriptions(void)
{
// Fixes these : https://github.com/knolleary/pubsubclient/issues/141
//             : https://github.com/knolleary/pubsubclient/issues/98
  for (int index = 0; index < maxSubscriptions; index++)
  {
    MQTTclient.subscribe(subscriptionsArray[index]);
    for (int i=0;i<10;i++) {
      MQTTclient.loop();
      delay(10);
    }
  }
}


//generate unique name from MAC addr
String macToStr(const uint8_t* mac, boolean addColons){

  String result;

  for (int i = 0; i < 6; ++i) {
    if ((mac[i] & 0xF0) == 0)
      result += String(0, HEX); // stop suppression of leading zero
    result += String(mac[i], HEX);

    if (addColons && (i < 5)){
      result += ':';
    }
  }
  
  return result;
}


void timer_create(int iTimerNumber, unsigned long ulTimerPeriod, void (*callbackfn)(void))
{
  if (iTimerNumber <= MAX_TIMERS)
  {
    stiTimerArray[iTimerNumber].tmrcallback = callbackfn;
    stiTimerArray[iTimerNumber].bRunning = false;
    stiTimerArray[iTimerNumber].ulTimerPeriod = ulTimerPeriod;
    stiTimerArray[iTimerNumber].ulStartValue = 0;
    #ifdef DEBUG_TIMER
    Serial.print(F("T Create, TNum : "));
    Serial.print(iTimerNumber);
    Serial.print(F(", TPeriod : "));
    Serial.println(ulTimerPeriod);
    #endif
  }
}



void timer_update(void)
{
  unsigned long ulCurrentTime = millis();
  unsigned long ulElapsedTime = 0;
  
  for (int iIndex = 0; iIndex < MAX_TIMERS; iIndex++)
  {
    if (stiTimerArray[iIndex].bRunning)
    {
      ulElapsedTime = ulCurrentTime - stiTimerArray[iIndex].ulStartValue;
      /* // Argh! twos complement arithmetic, I hate it...
      if (ulCurrentTime < stiTimerArray[iIndex].ulStartValue) // Cater for UL counter wrap ~every day
        ulElapsedTime = ulCurrentTime - stiTimerArray[iIndex].ulStartValue;
      else  
        ulElapsedTime = ulCurrentTime + (ULONG_MAX - stiTimerArray[iIndex].ulStartValue);
      */
      #ifdef DEBUG_TIMER
      Serial.print(F("T Up, TNum : "));
      Serial.print(iIndex);
      Serial.print(F(", T Elapsed : "));
      Serial.println(ulElapsedTime);
      #endif
        
      if (ulElapsedTime >= stiTimerArray[iIndex].ulTimerPeriod)
      {
        stiTimerArray[iIndex].bRunning = false;
        stiTimerArray[iIndex].tmrcallback();
      }
    }
  }
}


void timer_start(int iTimerNumber)
{
  if (iTimerNumber <= MAX_TIMERS)
  {
    stiTimerArray[iTimerNumber].ulStartValue = millis();
    stiTimerArray[iTimerNumber].bRunning = true;
    #ifdef DEBUG_TIMER
    Serial.print(F("T Start , TNum : "));
    Serial.print(iTimerNumber);
    Serial.print(F(", TStart : "));
    Serial.println(stiTimerArray[iTimerNumber].ulStartValue);
    #endif
  }
}


void timer_stop(int iTimerNumber)
{
  if (iTimerNumber <= MAX_TIMERS)
    stiTimerArray[iTimerNumber].bRunning = false;
  #ifdef DEBUG_TIMER
  Serial.print(F("T Stop : "));
  Serial.println(iTimerNumber);
  #endif
}


void timer_reset(int iTimerNumber)
{
  if (iTimerNumber <= MAX_TIMERS)
    stiTimerArray[iTimerNumber].ulStartValue = millis();
  #ifdef DEBUG_TIMER
  Serial.print(F("T Reset : "));
  Serial.println(iTimerNumber);
  #endif
}


boolean timer_isRunning(int iTimerNumber)
{
  return stiTimerArray[iTimerNumber].bRunning;
}


void timer_change_period(int iTimerNumber, unsigned long ulTimerPeriod)
{
  boolean bTmpRunning;
  if (iTimerNumber <= MAX_TIMERS)
  {
    bTmpRunning = stiTimerArray[iTimerNumber].bRunning;
    stiTimerArray[iTimerNumber].bRunning = false;
    stiTimerArray[iTimerNumber].ulTimerPeriod = ulTimerPeriod;
    stiTimerArray[iTimerNumber].bRunning = bTmpRunning;
    #ifdef DEBUG_TIMER
    Serial.print(F("T Change Period, TNum : "));
    Serial.print(iTimerNumber);
    Serial.print(F(", TPeriod : "));
    Serial.println(ulTimerPeriod);
    #endif
  }
}


  
void ledFlashTimerCallback(void)
{
  // This is called if the led flash timer has timed out. 
  #ifdef DEBUG_TIMER
  Serial.println("In ledFlashTimerCallback()");
  #endif


  #ifdef DEBUG_LEDFLASH
  SHOW_UPDATED_LED_STATE(eLEDFLASHSTATE_STATE,eLEDFLASHSTATE_STATE,"ledFlashTimerCallback");
  Serial.print("Led Flash : ");
  Serial.print(cFlashProfiles[eLEDFLASHSTATE_STATE][iFlashSequenceIndex]);
  Serial.print(", Led Flash Ind : ");
  Serial.print(iFlashSequenceIndex);
  Serial.print(", Led Flash State : ");
  Serial.println(eLEDFLASHSTATE_STATE);
  #endif


  switch (eLEDFLASHSTATE_STATE){
    case eLEDFLASH_NO_CONFIG    :
    case eLEDFLASH_PENDING_NW   :
    case eLEDFLASH_PENDING_MQTT :
        if (cFlashProfiles[eLEDFLASHSTATE_STATE][iFlashSequenceIndex] == '1')
          mcp23017.digitalWrite(lightPin, LOW);  // Led on
          //digitalWrite(lightPin, LOW); // Led on
        else  
          mcp23017.digitalWrite(lightPin, HIGH); // Led off
          //digitalWrite(lightPin, HIGH); // Led off
        break;
        
    case eLEDFLASH_SEQUENCE_END : 
        mcp23017.digitalWrite(lightPin, HIGH); // Led off
        //digitalWrite(lightPin, HIGH); // Led off
        eLEDFLASHSTATE_STATE = eLEDFLASH_OFF;
        break;
        
    case eLEDFLASH_OFF : 
        iFlashSequenceIndex = 0;
        break;
        
    default : 
        break;
  }

  iFlashSequenceIndex++;
  if (iFlashSequenceIndex >= (FLASH_SEQUENCE_MAX-1))
    iFlashSequenceIndex = 0;
  
  timer_start(LED_FLASH_TIMER);
  #ifdef DEBUG_SD
  Serial.println("In ledFlashTimerCallback");
  #endif
}


void periodicUpdateTimerCallbackAM2320(void)
{
  // This is called if the Periodic Update timer has timed out. 
  #ifdef DEBUG_TIMER
  Serial.println("In periodicUpdateTimerCallbackAM2320()");
  #endif

  sendTHUpdateAM2320 = true;
  timer_start(PERIODIC_UPDATE_TIMER_AM2320);
  #ifdef DEBUG_SD
  Serial.println("In periodicUpdateTimerCallbackAM2320");
  #endif
}


void periodicUpdateTimerCallbackDHT22(void)
{
  // This is called if the Periodic Update timer has timed out. 
  #ifdef DEBUG_TIMER
  Serial.println("In periodicUpdateTimerCallbackDHT22()");
  #endif

  sendTHUpdateDHT22 = true;
  timer_start(PERIODIC_UPDATE_TIMER_DHT22);
  #ifdef DEBUG_SD
  Serial.println("In periodicUpdateTimerCallbackDHT22");
  #endif
}




void returnOK(String mess) {
  #ifdef DEBUG_WEB
  Serial.println("returnOK");  
  #endif
  if (mess.length() > 0)
    server.send(200, "text/html", mess);
  else  
    server.send(200, "text/plain", "\n\r");
}

void returnFail(String mess) {
  #ifdef DEBUG_WEB
  Serial.println("returnFail");  
  #endif
  if (mess.length() > 0)
    server.send(500, "text/html", mess);
  else  
    server.send(500, "text/plain", "\n\r");
}


bool loadFromSD(String path){
  String dataType = "text/plain";
  #ifdef DEBUG_WEB
  Serial.println("loadFromSD");  
  Serial.print("path : ");  
  Serial.println(path);  
  Serial.print("strIndexFile : ");  
  Serial.println(strIndexFile);  
  #endif
//  if(path.endsWith("/")) path += "index.htm";
  if(path.endsWith("/")) path += strIndexFile;

  if(path.endsWith(".src")) path = path.substring(0, path.lastIndexOf("."));
  else if(path.endsWith(".htm")) dataType = "text/html";
  else if(path.endsWith(".css")) dataType = "text/css";
  else if(path.endsWith(".js")) dataType = "application/javascript";
  else if(path.endsWith(".json")) dataType = "application/json";
  else if(path.endsWith(".png")) dataType = "image/png";
  else if(path.endsWith(".gif")) dataType = "image/gif";
  else if(path.endsWith(".jpg")) dataType = "image/jpeg";
  else if(path.endsWith(".ico")) dataType = "image/x-icon";
  else if(path.endsWith(".xml")) dataType = "text/xml";
  else if(path.endsWith(".pdf")) dataType = "application/pdf";
  else if(path.endsWith(".zip")) dataType = "application/zip";
  else if(path.endsWith(".png")) dataType = "image/png";

  File dataFile = SD.open(path.c_str(),SD_FILE_READ_MODE);

  if (!dataFile)
    return false;

  if (server.hasArg("download")) dataType = "application/octet-stream";

  if (server.streamFile(dataFile, dataType) != dataFile.size()) {
    #ifdef DEBUG_WEB
    Serial.println("Sent less data than expected!");
    #endif
  }

  dataFile.close();
  return true;
}


void handleFileUpload(void){
  #ifdef DEBUG_WEB
  Serial.println("handleFileUpload");  
  #endif
  if(server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    if(SD.exists((char *)upload.filename.c_str())) SD.remove((char *)upload.filename.c_str());
    //uploadFile = SD.open(upload.filename.c_str(), FILE_WRITE); 
    uploadFile = SD.open(upload.filename.c_str(), SD_FILE_WRITE_MODE); 
    #ifdef DEBUG_WEB
    Serial.print("Upload: START, filename: "); Serial.println(upload.filename);
    #endif
  } else if(upload.status == UPLOAD_FILE_WRITE){
    if(uploadFile) uploadFile.write(upload.buf, upload.currentSize);
    #ifdef DEBUG_WEB
    Serial.print("Upload: WRITE, Bytes: "); Serial.println(upload.currentSize);
    #endif
  } else if(upload.status == UPLOAD_FILE_END){
    if(uploadFile) uploadFile.close();
    #ifdef DEBUG_WEB
    Serial.print("Upload: END, Size: "); Serial.println(upload.totalSize);
    #endif
  }
}


void deleteRecursive(String path){
  #ifdef DEBUG_WEB
  Serial.println("deleteRecursive");  
  #endif
  File file = SD.open((char *)path.c_str());
  if(!file.isDirectory()){
    file.close();
    SD.remove((char *)path.c_str());
    return;
  }

  file.rewindDirectory();
  while(true) {
    File entry = file.openNextFile();
    if (!entry) break;
    String entryPath = path + "/" +entry.name();
    if(entry.isDirectory()){
      entry.close();
      deleteRecursive(entryPath);
    } else {
      entry.close();
      SD.remove((char *)entryPath.c_str());
    }
    yield();
  }

  SD.rmdir((char *)path.c_str());
  file.close();
}


void handleDelete(void){
  String pass_response;
  String fail_response;

  pass_response  = "<html>";
  pass_response += "  <head>";
  pass_response += "   <title>Handle Deleted</title>";
  pass_response += " </head>";
  pass_response += " <body>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Speech Server </b> </font></p>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif'>Handle Deleted</font></p>";
  pass_response += " </body>";
  pass_response += "</html>";  

  fail_response  = "<html>";
  fail_response += "  <head>";
  fail_response += "   <title>Handle Not Deleted</title>";
  fail_response += " </head>";
  fail_response += " <body>";
  fail_response += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Speech Server </b> </font></p>";
  fail_response += "   <p><font face='Helvetica, Arial, sans-serif'>Handle Not Deleted</font></p>";
  fail_response += " </body>";
  fail_response += "</html>";  
  
  #ifdef DEBUG_WEB
  Serial.println("handleDelete");  
  #endif
  if(server.args() == 0) return returnFail("BAD ARGS");
  String path = server.arg(0);
  if(path == "/" || !SD.exists((char *)path.c_str())) {
    //returnFail("BAD PATH");
    returnFail(fail_response);
    return;
  }
  deleteRecursive(path);
  //returnOK();
  returnOK(pass_response);
}


void handleCreate(void){
  String pass_response;
  String fail_response_ba;
  String fail_response_bp;

  pass_response  = "<html>";
  pass_response += "  <head>";
  pass_response += "   <title>Handle Create</title>";
  pass_response += " </head>";
  pass_response += " <body>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Speech Server </b> </font></p>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif'>Handle Created</font></p>";
  pass_response += " </body>";
  pass_response += "</html>";  

  fail_response_ba  = "<html>";
  fail_response_ba += "  <head>";
  fail_response_ba += "   <title>Handle Created</title>";
  fail_response_ba += " </head>";
  fail_response_ba += " <body>";
  fail_response_ba += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Speech Server </b> </font></p>";
  fail_response_ba += "   <p><font face='Helvetica, Arial, sans-serif'>File Not Created : Bad Args</font></p>";
  fail_response_ba += " </body>";
  fail_response_ba += "</html>";  
  
  fail_response_bp  = "<html>";
  fail_response_bp += "  <head>";
  fail_response_bp += "   <title>Handle Create</title>";
  fail_response_bp += " </head>";
  fail_response_bp += " <body>";
  fail_response_bp += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Speech Server </b> </font></p>";
  fail_response_bp += "   <p><font face='Helvetica, Arial, sans-serif'>File Not Created : Bad Path</font></p>";
  fail_response_bp += " </body>";
  fail_response_bp += "</html>";  
  
  #ifdef DEBUG_WEB
  Serial.println("handleCreate");
  #endif
  //if(server.args() == 0) return returnFail("BAD ARGS");
  if(server.args() == 0) return returnFail(fail_response_ba);
  String path = server.arg(0);
  if(path == "/" || SD.exists((char *)path.c_str())) {
    //returnFail("BAD PATH");
    returnFail(fail_response_bp);
    return;
  }

  if(path.indexOf('.') > 0){
    //File file = SD.open((char *)path.c_str(), FILE_WRITE); 
    File file = SD.open((char *)path.c_str(), SD_FILE_WRITE_MODE); 
    if(file){
      file.write((const char *)0);
      file.close();
    }
  } else {
    SD.mkdir((char *)path.c_str());
  }
  //returnOK();
  returnOK(pass_response);
}


void printDirectory(void) {
  String fail_response_ba;
  String fail_response_bp;
  String fail_response_nd;

  fail_response_ba  = "<html>";
  fail_response_ba += "  <head>";
  fail_response_ba += "   <title>Print Directory</title>";
  fail_response_ba += " </head>";
  fail_response_ba += " <body>";
  fail_response_ba += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Speech Server </b> </font></p>";
  fail_response_ba += "   <p><font face='Helvetica, Arial, sans-serif'>Print Directory : Bad Args</font></p>";
  fail_response_ba += " </body>";
  fail_response_ba += "</html>";  
  
  fail_response_bp  = "<html>";
  fail_response_bp += "  <head>";
  fail_response_bp += "   <title>Print Directory</title>";
  fail_response_bp += " </head>";
  fail_response_bp += " <body>";
  fail_response_bp += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Speech Server </b> </font></p>";
  fail_response_bp += "   <p><font face='Helvetica, Arial, sans-serif'>Print Directory : Bad Path</font></p>";
  fail_response_bp += " </body>";
  fail_response_bp += "</html>";    
  
  fail_response_nd  = "<html>";
  fail_response_nd += "  <head>";
  fail_response_nd += "   <title>Print Directory</title>";
  fail_response_nd += " </head>";
  fail_response_nd += " <body>";
  fail_response_nd += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Speech Server </b> </font></p>";
  fail_response_nd += "   <p><font face='Helvetica, Arial, sans-serif'>Print Directory : Not Dir</font></p>";
  fail_response_nd += " </body>";
  fail_response_nd += "</html>";  
  
  #ifdef DEBUG_WEB
  Serial.println("printDirectory");
  #endif
  //if(!server.hasArg("dir")) return returnFail("BAD ARGS");
  if(!server.hasArg("dir")) return returnFail(fail_response_ba);
  String path = server.arg("dir");
  //if(path != "/" && !SD.exists((char *)path.c_str())) return returnFail("BAD PATH");
  if(path != "/" && !SD.exists((char *)path.c_str())) return returnFail(fail_response_bp);
  File dir = SD.open((char *)path.c_str());
  path = String();
  if(!dir.isDirectory()){
    dir.close();
    //return returnFail("NOT DIR");
    return returnFail(fail_response_nd);
  }
  dir.rewindDirectory();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/json", "");
  WiFiClient client = server.client();

  server.sendContent("[");
  for (int cnt = 0; true; ++cnt) {
    File entry = dir.openNextFile();
    if (!entry)
    break;

    String output;
    if (cnt > 0)
      output = ',';

    output += "{\"type\":\"";
    output += (entry.isDirectory()) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += entry.name();
    output += "\"";
    output += "}";
    server.sendContent(output);
    entry.close();
 }
 server.sendContent("]");
 dir.close();
}


void handleNetworkConfig(void)
{
  String pass_response;
  String fail_response;
  char tmp_mqtt_broker_ip[MQTT_BROKER_IP_STRING_MAX_LEN];
  int  tmp_mqtt_broker_port;
  int  tmp_mqtt_broker_connection_attempts = MQTT_BROKER_CONNECTION_ATTEMPTS_DEFAULT;
  char tmp_sta_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
  char tmp_sta_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];
  int  tmp_network_connection_attempts = NETWORK_CONNECTION_ATTEMPTS_DEFAULT;
  //char tmp_ap_network_ssid[NETWORK_SSID_STRING_MAX_LEN];
  //char tmp_ap_network_password[NETWORK_PASSWORD_STRING_MAX_LEN];

  pass_response  = "<html>";
  pass_response += "  <head>";
  pass_response += "   <title>Form submitted</title>";
  pass_response += " </head>";
  pass_response += " <body>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Speech Synth Configuration Home Page </b> </font></p>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif'>New configuration details now submitted</font></p>";
  pass_response += "   <p><font face='Helvetica, Arial, sans-serif'><a href='index.htm'>Return to main page</a></font></p>";
  pass_response += " </body>";
  pass_response += "</html>";  

  fail_response  = "<html>";
  fail_response += "  <head>";
  fail_response += "   <title>Form not submitted</title>";
  fail_response += " </head>";
  fail_response += " <body>";
  fail_response += "   <p><font face='Helvetica, Arial, sans-serif' size='5' color='#3366ff'> <b> Speech Synth Configuration Home Page </b> </font></p>";
  fail_response += "   <p><font face='Helvetica, Arial, sans-serif'>Return to main page and re-submit details</font></p>";
  fail_response += "   <p><font face='Helvetica, Arial, sans-serif'><a href='index.htm'>Return to main page</a></font></p>";
  fail_response += " </body>";
  fail_response += "</html>";  

  #ifdef DEBUG_WEB
  Serial.println("In handleNetworkConfig");
  #endif
  String strMQTTBrokerIPAddress=server.arg("MQTTBrokerIPAddress");
  String strMQTTBrokerPort=server.arg("MQTTBrokerPort");
  String strMQTTBrokerConnectionAttempts=server.arg("MQTTBrokerConnectionAttempts");
  String strNetworkSSID=server.arg("NetworkSSID");
  String strNetworkPassword=server.arg("NetworkPassword");
  String strNetworkConnectionAttempts=server.arg("NetworkConnectionAttempts");

  strMQTTBrokerIPAddress.trim();
  strMQTTBrokerPort.trim();
  strMQTTBrokerConnectionAttempts.trim();
  strNetworkSSID.trim();
  strNetworkPassword.trim();
  strNetworkConnectionAttempts.trim();

  strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
  if (! isValidIpv4Address((char *)strMQTTBrokerIPAddress.c_str())) {
    returnOK(fail_response);
  } else {
    //strcpy(tmp_mqtt_broker_ip,strMQTTBrokerIPAddress.c_str());
    if (! isValidNumber(strMQTTBrokerPort)) {
      returnOK(fail_response);
    } else {
      tmp_mqtt_broker_port = strMQTTBrokerPort.toInt();
      if (((strNetworkSSID.length() == 0)     || (strNetworkSSID.length() >= NETWORK_SSID_STRING_MAX_LEN)) || 
          ((strNetworkPassword.length() == 0) || (strNetworkPassword.length() >= NETWORK_PASSWORD_STRING_MAX_LEN))) {
        returnOK(fail_response);
      } else {
        strcpy(tmp_sta_network_ssid,strNetworkSSID.c_str());
        strcpy(tmp_sta_network_password,strNetworkPassword.c_str());

        if (! isValidNumber(strMQTTBrokerConnectionAttempts)) {
          returnOK(fail_response);
        } else {
          tmp_mqtt_broker_connection_attempts = strMQTTBrokerConnectionAttempts.toInt();
          if ((tmp_mqtt_broker_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_mqtt_broker_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
            returnOK(fail_response);
          } else {
            if (! isValidNumber(strNetworkConnectionAttempts)) {
              returnOK(fail_response);
            } else {
              tmp_network_connection_attempts = strNetworkConnectionAttempts.toInt();
              if ((tmp_network_connection_attempts < CONNECTION_ATTEMPTS_MIN) || (tmp_network_connection_attempts > CONNECTION_ATTEMPTS_MAX)) {
                returnOK(fail_response);
              } else {
                strcpy(mqtt_broker_ip,tmp_mqtt_broker_ip);
                mqtt_broker_port = tmp_mqtt_broker_port;
                mqtt_broker_connection_attempts = tmp_mqtt_broker_connection_attempts;
                strcpy(sta_network_ssid,tmp_sta_network_ssid);
                strcpy(sta_network_password,tmp_sta_network_password);
                network_connection_attempts = tmp_network_connection_attempts;
                bBrokerPresent = true;
                // Save new network parameters
                File f = SD.open(SECURITY_PARAMETERS_FILE, SD_FILE_WRITE_MODE);
                if (f) {
                  fileWrite(f, SecurityVarArray,(int)(sizeof(SecurityVarArray)/sizeof(tsFileVarInstance)));
                  f.close();
                }      
                returnOK(pass_response);
                #ifdef DEBUG_STATE_CHANGE
                SHOW_UPDATED_STATE(eSENSORSTATE_STATE,eSENSORSTATE_INIT,"handleNetworkConfig");
                #endif
                eSENSORSTATE_STATE = eSENSORSTATE_INIT;
              }
            }
          }
        }
      }
    }
  }
 
  #ifdef DEBUG_WEB
  Serial.print("MQTTBrokerIPAddress : "); Serial.println(mqtt_broker_ip);
  Serial.print("MQTTBrokerPort : "); Serial.println(mqtt_broker_port);
  Serial.print("MQTTBrokerConnectionAttempts : "); Serial.println(mqtt_broker_connection_attempts);
  Serial.print("STANetworkSSID : "); Serial.println(sta_network_ssid);
  Serial.print("STANetworkPassword : "); Serial.println(sta_network_password);
  Serial.print("NetworkConnectionAttempts : "); Serial.println(network_connection_attempts);
  #endif
  return;
}



/*
 * http://www.esp8266.com/viewtopic.php?f=29&t=2153
 * 
Processing arguments of GET and POST requests is also easy enough. Let's make our sketch turn a led on or off depending on the value of a request argument.
http://<ip address>/led?state=on will turn the led ON
http://<ip address>/led?state=off will turn the led OFF
CODE: SELECT ALL
server.on("/led", []() {
  String state=server.arg("state");
  if (state == "on") digitalWrite(13, LOW);
  else if (state == "off") digitalWrite(13, HIGH);
  server.send(200, "text/plain", "Led is now " + state);
});
- See more at: http://www.esp8266.com/viewtopic.php?f=29&t=2153#sthash.7O0kU5VW.dpuf
 */

void handleNotFound(void){
  #ifdef DEBUG_WEB
  Serial.println("handleNotFound");
  #endif
  if(hasSD && loadFromSD(server.uri())) return;
  String message = " Not Detected\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " NAME:"+server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  #ifdef DEBUG_WEB
  Serial.print(message);
  #endif
}



boolean isFloat(String tString) {
  String tBuf;
  boolean decPt = false;
  
  if(tString.charAt(0) == '+' || tString.charAt(0) == '-') tBuf = &tString[1];
  else tBuf = tString;  

  for(int x=0;x<tBuf.length();x++)
  {
    if(tBuf.charAt(x) == '.') {
      if(decPt) return false;
      else decPt = true;  
    }    
    else if(tBuf.charAt(x) < '0' || tBuf.charAt(x) > '9') return false;
  }
  return true;
}



boolean isValidNumber(String str){
   for(byte i=0;i<str.length();i++)
   {
      if(!isDigit(str.charAt(i))) return false;
   }
   return true;
} 

/*
boolean isValidNumber(String str){
   for(byte i=0;i<str.length();i++)
   {
      if(isDigit(str.charAt(i))) return true;
   }
   return false;
} 
*/

bool isValidIpv4Address(char *st)
{
    int num, i, len;
    char *ch;

    //counting number of quads present in a given IP address
    int quadsCnt=0;

    #ifdef DEBUG_VALIDATION
    Serial.print("Split IP: ");
    Serial.println(st);
    #endif
    len = strlen(st);

    //  Check if the string is valid
    if(len<7 || len>15)
        return false;

    ch = strtok(st, ".");

    while (ch != NULL) 
    {
        quadsCnt++;
        #ifdef DEBUG_VALIDATION
        Serial.print("Quald ");
        Serial.print(quadsCnt);
        Serial.print(" is ");
        Serial.println(ch);
        #endif

        num = 0;
        i = 0;

        //  Get the current token and convert to an integer value
        while(ch[i]!='\0')
        {
            num = num*10;
            num = num+(ch[i]-'0');
            i++;
        }

        if(num<0 || num>255)
        {
            #ifdef DEBUG_VALIDATION
            Serial.println("Not a valid ip");
            #endif
            return false;
        }

        if( (quadsCnt == 1 && num == 0) || (quadsCnt == 4 && num == 0))
        {
            #ifdef DEBUG_VALIDATION
            Serial.print("Not a valid ip, quad: ");
            Serial.print(quadsCnt);
            Serial.print(" AND/OR quad: ");
            Serial.print(quadsCnt);
            Serial.println(" is zero");
            #endif
            return false;
        }

        ch = strtok(NULL, ".");
    }

    //  Check the address string, should be n.n.n.n format
    if(quadsCnt!=4)
    {
        return false;
    }

    //  Looks like a valid IP address
    return true;
}


void initSP0256SpeechProcessor(void)
{
  mcp23017.pinMode(SPEECH_PROCESSOR_RESET, OUTPUT);
  mcp23017.digitalWrite(SPEECH_PROCESSOR_RESET, HIGH);  
  delay(10);
  mcp23017.digitalWrite(SPEECH_PROCESSOR_RESET, LOW);  
  mcp23017.pinMode(ADDR_LOAD, OUTPUT);
  mcp23017.digitalWrite(ADDR_LOAD, HIGH);  // Inactive
  mcp23017.pinMode(SPBit0, OUTPUT);
  mcp23017.digitalWrite(SPBit0, LOW);  
  mcp23017.pinMode(SPBit1, OUTPUT);
  mcp23017.digitalWrite(SPBit1, LOW);  
  mcp23017.pinMode(SPBit2, OUTPUT);
  mcp23017.digitalWrite(SPBit2, LOW);  
  mcp23017.pinMode(SPBit3, OUTPUT);
  mcp23017.digitalWrite(SPBit3, LOW);  
  mcp23017.pinMode(SPBit4, OUTPUT);
  mcp23017.digitalWrite(SPBit4, LOW);  
  mcp23017.pinMode(SPBit5, OUTPUT);
  mcp23017.digitalWrite(SPBit5, LOW);  
  mcp23017.pinMode(LOAD_REQUEST, INPUT);
  
  for (int x = 0; x < 5; x++)
  {
    while (mcp23017.digitalRead(LOAD_REQUEST) != 0) {delay(10);};
    mcp23017.digitalWrite(ADDR_LOAD, LOW);  // Do one speak or a PA0 or pause, in case the speech processor is noisy
    mcp23017.digitalWrite(ADDR_LOAD, HIGH);  
  }
  #ifdef DEBUG_SPEECH
  Serial.println(F("S Init : "));
  #endif
}


void testSP0256SpeechProcessor(void)
{
  mcp23017.digitalWrite(ADDR_LOAD, HIGH);  
  #ifdef DEBUG_SPEECH
  Serial.println("testSP0256SpeechProcessor");
  #endif
  delay(10);
  if (mcp23017.digitalRead(LOAD_REQUEST) == HIGH) {
    #ifdef DEBUG_SPEECH
    Serial.println("LRQ == HIGH, resetSP0256SpeechProcessor");
    #endif
    resetSP0256SpeechProcessor();
  } 
}



void resetSP0256SpeechProcessor(void)
{
  mcp23017.digitalWrite(SPEECH_PROCESSOR_RESET, HIGH);  
  delay(2);
  mcp23017.digitalWrite(SPEECH_PROCESSOR_RESET, LOW);  
  delay(100);
}


void presentByteSP0256SpeechProcessor(uint8_t uPhoneme)
{
  // Present data to SP0256 inputs
  #ifdef DEBUG_SPEECH
  Serial.println("Setting Port Bits");
  #endif
  if (BIT_IS_SET(uPhoneme,SPBit0))
    mcp23017.digitalWrite(SPBit0,HIGH);  
  else
    mcp23017.digitalWrite(SPBit0,LOW);  

  if (BIT_IS_SET(uPhoneme,SPBit1))
    mcp23017.digitalWrite(SPBit1,HIGH);  
  else
    mcp23017.digitalWrite(SPBit1,LOW);  
    
  if (BIT_IS_SET(uPhoneme,SPBit2))
    mcp23017.digitalWrite(SPBit2,HIGH);  
  else
    mcp23017.digitalWrite(SPBit2,LOW);  
    
  if (BIT_IS_SET(uPhoneme,SPBit3))
    mcp23017.digitalWrite(SPBit3,HIGH);  
  else
    mcp23017.digitalWrite(SPBit3,LOW);  
    
  if (BIT_IS_SET(uPhoneme,SPBit4))
    mcp23017.digitalWrite(SPBit4,HIGH);  
  else
    mcp23017.digitalWrite(SPBit4,LOW);  
    
  if (BIT_IS_SET(uPhoneme,SPBit5))
    mcp23017.digitalWrite(SPBit5,HIGH);  
  else
    mcp23017.digitalWrite(SPBit5,LOW);  
    
  #ifdef DEBUG_SPEECH
  char tmpStrDebug[10];
  sprintf(tmpStrDebug,"S Write : 0x%02X,",uPhoneme);
  Serial.println(tmpStrDebug);
  #endif
}


void writeByteSP0256SpeechProcessor(uint8_t uPhoneme)
{
  presentByteSP0256SpeechProcessor(uPhoneme);
  
  while (mcp23017.digitalRead(LOAD_REQUEST) != 0) {}
  // Clock data into SP0256
  mcp23017.digitalWrite(ADDR_LOAD, LOW);  
  mcp23017.digitalWrite(ADDR_LOAD, HIGH);  

  #ifdef DEBUG_SPEECH
  char tmpStrDebug[10];
  sprintf(tmpStrDebug,"S Write : 0x%02X,",uPhoneme);
  Serial.println(tmpStrDebug);
  #endif
}


boolean isBusySP0256SpeechProcessor(void)
{
  return (mcp23017.digitalRead(LOAD_REQUEST) != 0);
}



boolean isNum(char n)
{
  return ((((uint8_t)n) >= ((uint8_t)'0')) && (((uint8_t)n) <= ((uint8_t)'9')));
}


boolean isLetter(char l)
{
  return (((((uint8_t)l) >= ((uint8_t)'a')) && (((uint8_t)l) <= ((uint8_t)'z'))) || ((((uint8_t)l) >= ((uint8_t)'A')) && (((uint8_t)l) <= ((uint8_t)'Z'))));
}


boolean isHex(char h)
{
  return ((((((uint8_t)h) >= ((uint8_t)'a')) && (((uint8_t)h) <= ((uint8_t)'f')))) || (((((uint8_t)h) >= ((uint8_t)'A')) && (((uint8_t)h) <= ((uint8_t)'F')))) || isNum(h) );
}



uint8_t hexVal(char v)
{
  if (isHex(v))
  {
    if (!isNum(v))
      v &= TO_UPPER;
    return ((uint8_t) (strchr((const char *)hexString,v) - hexString));
  } else {
    return ((uint8_t)0);
  }
}


uint8_t getPhoneme(const char* phonemeString)
{
  char charBuff[10];
  int iCount = 0;
  while (phonemeString[iCount])
  {
    if (isNum(phonemeString[iCount]))
      charBuff[iCount] = phonemeString[iCount];
    else
      if (isLetter(phonemeString[iCount]))
        charBuff[iCount] = MAKE_UPPER_CASE(phonemeString[iCount]);
      else
        return ((uint8_t) PHONEME_UNRECOGNISED);
    iCount++;        
  }
  charBuff[iCount] = (char) 0x00;
  
  for (int Index = 0; Index < maxPhonemes; Index++)
    if (strcmp(charBuff, allophoneAddressTable[Index].Allophone) == 0)
      return ((uint8_t)allophoneAddressTable[Index].hexAddress);

  return ((uint8_t) PHONEME_UNRECOGNISED);
}


void handleFixedPhrase(int whichPhrase, eSpeechProcessorState nextState)
{
  int tmpInt  = 0;
  int tmpInt1 = 0;

  tmpInt = whichPhrase;
  if ((tmpInt) < maxFixedPhrases)
  {
    tmpInt1 = 0;
    // Copy fixed phrase to buffer and initiate send to the SP0256
    #ifdef DEBUG_SPEECH
    Serial.print("Copying Phonemes :");
    #endif
    while ((PHONEME_FIXED_PHRASE_STORE[tmpInt])[tmpInt1] != PHONEME_END_MARKER)
    {
          phonemeBufferStore[tmpInt1] = (PHONEME_FIXED_PHRASE_STORE[tmpInt])[tmpInt1];
          tmpInt1++;
    }
    phonemeBufferStore[tmpInt1] = (PHONEME_FIXED_PHRASE_STORE[tmpInt])[tmpInt1];
    
    currentPhoneme = 0;
    switch (nextState){
      case eSPEECH_SPEAKING_FIXED_PHRASE : mcp23017.digitalWrite(BLUE_LED, LOW);
                                           SpeechProcessorCurrentState = eSPEECH_SPEAKING_FIXED_PHRASE;
                                           break;
      case eSPEECH_HTTP                  : mcp23017.digitalWrite(RED_LED, LOW);
                                           SpeechProcessorCurrentState = eSPEECH_HTTP;
                                           break;
                                 default : ;
    }
    testSP0256SpeechProcessor();
    writeByteSP0256SpeechProcessor(phonemeBufferStore[currentPhoneme++]);
  }
}


void handleHttpGet(void)
{
    switch (WebServerHttpGetHandlerCurrentState){
      case eWEBSVR_SPEECH_SNYTH_CONFIG_HOME_PAGE : handleNetworkConfig();
                                                   break;
      case eWEBSVR_SPEECH_SNYTH_WEB_PAGE         : handleButtonPresses();
                                                   break;
                                         default : ;
    }
}


void handleButtonPresses(void)
{
  #ifdef DEBUG_WEB
  Serial.println("In handleButtonPresses");
  #endif
  if ((SpeechProcessorCurrentState != eSPEECH_SPEAKING_HEX_PHONEMES) && // Only handle callback functionality if the ESP8266-12E is not in the process of speaking
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_PHONEMES) && 
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_FIXED_PHRASE) &&
      (SpeechProcessorCurrentState != eSPEECH_SPEAKING_WORDS) &&
      (SpeechProcessorCurrentState != eSPEECH_HTTP)) {
    String phrase=server.arg("PHRASE");
   
    if (phrase == "0") {
      handleFixedPhrase(0, eSPEECH_HTTP);  
    } else {
      if (phrase == "1") {
        handleFixedPhrase(1, eSPEECH_HTTP);  
      } else {
        if (phrase == "2") {
          handleFixedPhrase(2, eSPEECH_HTTP);  
        } else {
          String led=server.arg("LED");
          if (led == "BLUEN") {
            mcp23017.digitalWrite(BLUE_LED, LOW);
          } else {
            if (led == "BLUEF") {
              mcp23017.digitalWrite(BLUE_LED, HIGH);
            } else {
              if (led == "REDN") {
                mcp23017.digitalWrite(RED_LED, LOW);
              } else {
                if (led == "REDF") {
                  mcp23017.digitalWrite(RED_LED, HIGH);
                } else {
                  String reset=server.arg("RESET");
                  if (reset == "1") {
                    resetSP0256SpeechProcessor();
                  } else {
                    returnFail("UNKOWN COMMAND");
                    return;
                  }
                }
              }
            }
          }
        } 
      }
    }
    returnOK("SPEECH OK");
    return;
  
  } else {
    returnFail("SPEECH BUSY");
    return;
  }
}


float convertCtoF(float c) {
  return c * 1.8 + 32;
}

float convertFtoC(float f) {
  return (f - 32) * 0.55555;
}


//boolean isFahrenheit: True == Fahrenheit; False == Celcius
float computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float hi;

  if (!isFahrenheit)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * percentHumidity +
            -0.22475541 * temperature*percentHumidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature*pow(percentHumidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isFahrenheit ? hi : convertFtoC(hi);
}


