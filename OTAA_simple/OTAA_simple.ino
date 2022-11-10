/*
 * HelTec Automation(TM) LoRaWAN 1.0.2 OTAA example use OTAA, CLASS A
 *
 * Function summary:
 *
 * - use internal RTC(150KHz);
 *
 * - Include stop mode and deep sleep mode;
 *
 * - 15S data send cycle;
 *
 * - Informations output via serial(115200);
 *
 * - Only ESP32 + LoRa series boards can use this library, need a license
 *   to make the code run(check you license here: http://www.heltec.cn/search/);
 *
 * You can change some definition in "Commissioning.h" and "LoRaMac-definitions.h"
 *
 * HelTec AutoMation, Chengdu, China.
 * 成都惠利特自动化科技有限公司
 * https://heltec.org
 * support@heltec.cn
 *
 *this project also release in GitHub:
 *https://github.com/HelTecAutomation/ESP32_LoRaWAN
*/

#include <ESP32_LoRaWAN.h>
#include "Arduino.h"
#include <DHT.h>
#include <SPI.h>
#include <Wire.h>

/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */


//DEFINE
#define DHTPIN 13//13

//INIT
DHT dht(DHTPIN, DHT11);

//FLAG
int d_dht = 0;
//DHT11

//GLOBAL
float humidity, temperature;
float humidity_new = -1, temperature_new = -1;
//back


uint32_t  license[4] = {0x2DDE4B60,0xDFE5A404,0x86F11611,0x8B477132};
/* OTAA para*/
//uint8_t DevEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x04, 0x59, 0x9E };
//uint8_t AppEui[] = { 0xC0, 0xEE, 0x40, 0xFF, 0xFF, 0x29, 0x34, 0xE3 };
//uint8_t AppKey[] = { 0x00, 0x28, 0x83, 0xA5, 0x05, 0x7A, 0x73, 0x62, 0xF2, 0x9E, 0x90, 0x20, 0xF4, 0xD5, 0x79, 0x8E };
uint8_t DevEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0x76, 0x5B };
uint8_t AppEui[] = { 0xC0, 0xEE, 0x40, 0xFF, 0xFF, 0x29, 0x34, 0xE3 };
uint8_t AppKey[] = { 0x24, 0xC1, 0xB4, 0xFC, 0x2B, 0x79, 0xD6, 0xB6, 0x89, 0xFD, 0xCA, 0x74, 0xAD, 0x99, 0x23, 0x4E };

/* ABP para*/
uint8_t NwkSKey[] = { 0x7D, 0x1B, 0x76, 0xF1, 0x69, 0xCC, 0xAE, 0x3E, 0xA1, 0x10, 0x7C, 0x1B, 0x24, 0x5A, 0x9E, 0x92 };
uint8_t AppSKey[] = { 0x69, 0x46, 0x6A, 0xBF, 0x64, 0xE8, 0x77, 0x2E, 0x1F, 0x01, 0xB3, 0x46, 0x1A, 0xC0, 0xE2, 0x51 };
uint32_t DevAddr =  ( uint32_t )0x260DD938;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0xFF00,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 60000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true; //TROCAR PARA FALSE ?

/* Application port */
uint8_t appPort = 2;

/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/*LoraWan debug level, select in arduino IDE tools.
* None : print basic info.
* Freq : print Tx and Rx freq, DR info.
* Freq && DIO : print Tx and Rx freq, DR, DIO0 interrupt and DIO1 interrupt info.
* Freq && DIO && PW: print Tx and Rx freq, DR, DIO0 interrupt, DIO1 interrupt, MCU sleep and MCU wake info.
*/
uint8_t debugLevel = LoRaWAN_DEBUG_LEVEL;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION; //TEM QUE USAR US915 EM FERRAMENTAS -> LORAWANREGION!





void readDHT(){  
  humidity_new = dht.readHumidity();
  delay(50);
  temperature_new = dht.readTemperature();
  delay(50);
  if((isnan(humidity_new) || isnan(temperature_new) || humidity_new == -1 || temperature_new == -1 )){
    Serial.println("Não foi possivel ler o sensor:"); 
    d_dht = 0;     
  }else if(humidity_new != humidity || temperature_new != temperature){
    humidity = humidity_new;
    temperature = temperature_new;
    d_dht = 1;
  }
}
static void prepareTxFrame( uint8_t port )
{
  //int8 -> int16
  uint16_t temp = (uint16_t) (temperature * 100);
  uint16_t hum = (uint16_t) (humidity * 100);
  
  //envio
    appDataSize = 10;                 //AppDataSize max value is 64
    appData[0] = temp >> 8;
    appData[1] = temp & 0xFF;
    appData[2] = hum >> 8;
    appData[3] = hum & 0xFF;
    appData[4] = d_dht;
}

// Add your initialization code here
void setup()
{
  Serial.begin(115200);
  dht.begin();

  while (!Serial);
  SPI.begin(SCK,MISO,MOSI,SS);
  Mcu.init(SS,RST_LoRa,DIO0,DIO1,license);
  deviceState = DEVICE_STATE_INIT;
}

// The loop function is called in an endless loop
void loop()
{
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
      LoRaWAN.init(loraWanClass,loraWanRegion);
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      readDHT();
      prepareTxFrame( appPort );
      LoRaWAN.send(loraWanClass);
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep(loraWanClass,debugLevel);
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
