#include <ESP32_LoRaWAN.h>
#include "OTAA_OLED_KEYS.h"
#include "Arduino.h"
#include <DHT.h>
#include <SPI.h>
#include <Wire.h>

#define DHTPIN 13//13

DHT dht(DHTPIN, DHT11);

int d_dht = 0;

float humidity, temperature;
float humidity_new = -1, temperature_new = -1;

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