#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>      // SPI Library used by the radio
#include <RH_RF95.h>  // Library to control the radio
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SparkFun_VL53L1X.h>   //Sensor de distancia
#include <vl53l1x_class.h>      //Sensor de distancia
#include <vl53l1_error_codes.h> //Sensor de distancia
#include <Wire.h>               //Protocolos de comunciación serial: I2C
#include "SparkFun_VL53L1X.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <math.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RF95_FREQ 915.0
#define SHUTDOWN_PIN 11
#define INTERRUPT_PIN 12
#define DHTPIN 13
#define DHTTYPE DHT22

RH_RF95 rf95(RFM95_CS, RFM95_INT);
OneWire ds(5);
DallasTemperature sensors(&ds);
SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
DHT_Unified dht(DHTPIN, DHTTYPE);

int counter = 0;

struct data{
  float mean = 0.0;
  float sd = 0.0;
};

struct packet_t{
  int id = 1;
  int p_count = 1;
  data temp;
  data humidity;
  data distance;
};

packet_t packet;

float mean(float actual_mean, float new_data, int counter){
  actual_mean += (new_data - actual_mean)/counter;
  return actual_mean;
}

float standard_d(float actual_sd, float last_mean, float new_data, int counter){
  float actual_variance = pow(actual_sd, 2);
  if (counter <= 1){
    return 0;
  }
  actual_variance += pow(new_data - last_mean, 2)/counter - actual_variance/(counter - 1);
  return pow(actual_variance, 0.5);
}

float hum(){
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  return event.relative_humidity;
}

float temp(){
  float t;
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

int dist(){
  int x;
  
  distanceSensor.startRanging(); 
  while (!distanceSensor.checkForDataReady()) {
    delay(1);
  }
  x=distanceSensor.getDistance();

  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  return x;
}

void processing(packet_t *packet){
  int counter = packet->p_count++;
  float temperature = temp();
  float humidity = hum();
  int distance = dist();
  packet->temp.sd = standard_d(packet->temp.sd, packet->temp.mean, temperature, counter);
  packet->temp.mean = mean(packet->temp.mean, temperature, counter);
  packet->humidity.sd = standard_d(packet->humidity.sd, packet->humidity.mean, humidity, counter);
  packet->humidity.mean = mean(packet->humidity.mean, humidity, counter);
  packet->distance.sd = standard_d(packet->distance.sd, packet->distance.mean, float(distance), counter);
  packet->distance.mean = mean(packet->distance.mean, float(distance), counter);
}

void send_data(packet_t *packet){  
  String radiopacket = String(packet->id);
  
  radiopacket += ",";
  radiopacket += String(packet->temp.mean,2);
  radiopacket += ",";
  radiopacket += String(packet->temp.sd,2);
  radiopacket += ",";
  radiopacket += String(packet->humidity.mean,2);
  radiopacket += ",";
  radiopacket += String(packet->humidity.sd,2);
  radiopacket += ",";
  radiopacket += String(packet->distance.mean,2);
  radiopacket += ",";
  radiopacket += String(packet->distance.sd,2);
  
  //radiopacket[27] = 0;  // Check if can be removed
  Serial.println(radiopacket);
  
  Serial.println("Sending packet ...");
  delay(10);
  rf95.send((uint8_t *)radiopacket.c_str(), sizeof(radiopacket.c_str()));
  Serial.println("Waiting for packet to complete..."); 
  delay(10);
  rf95.waitPacketSent();

  // Confirmation reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  int ack = 0;
  do
  {
    ack = rf95.waitAvailableTimeout(1000);
    Serial.println("Waiting for reply...");
    if (ack)
    { 
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len))
      {
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);    
      }
      else
      {
        Serial.println("Receive failed");
      }
    }
    else
    {
      Serial.println("Ack Failed: Re-sending packet ...");
      delay(10);
      rf95.send((uint8_t *)radiopacket.c_str(), sizeof(radiopacket.c_str()));
      Serial.println("Waiting for packet to complete..."); 
      delay(10);
      rf95.waitPacketSent();
    }
  } while(!ack);
  

  Serial.println("Packet delivered successfully!");
  // Restart all the data in the packet
  packet->p_count = 1;
  packet->temp.sd = 0.0;
  packet->temp.mean = 0.0;
  packet->humidity.sd = 0.0;
  packet->humidity.mean = 0.0;
  packet->distance.sd = 0.0;
  packet->distance.mean = 0.0;
}

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  delay(100);
  Serial.println("GWSat Remote Sensing Node");
  
  Wire.begin();
  sensors.begin();
  dht.begin();
  distanceSensor.begin();

  // radio manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
  rf95.sleep(); // Put the radio to sleep to to decrease energy consumption
}

void loop(){
  unsigned long time1;
  unsigned long time2;
  
  time1 = millis();
  
  /*
  if (counter == 0){
    packet = eraser;
  }
  counter++;
  processing(&packet, counter);
  */
  processing(&packet);
  Serial.println("Data computed");
  time2=millis();
  if (packet.p_count > 2){
    //counter = 0;
    Serial.println("Sendind the following data ...");
    Serial.print("Temp mean: ");
    Serial.print(packet.temp.mean);
    Serial.print("\tDist mean: ");
    Serial.print(packet.distance.mean);
    Serial.print("\tHum mean: ");
    Serial.println(packet.humidity.mean);
    Serial.print("Temp standar d: ");
    Serial.print(packet.temp.sd);
    Serial.print("\tDist standar d: ");
    Serial.print(packet.distance.sd);
    Serial.print("\tHum standar d: ");
    Serial.println(packet.humidity.sd);
    Serial.println("------------------------------------------");
    send_data(&packet);
    rf95.sleep(); 
  }
  else{
    delay(1000);
    //delay(60000-int(time2-time1));
  };
  /*
  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.print("\t\tDistance: ");
  Serial.print(distance);
  Serial.print("\t\tHumidity: ");
  Serial.println(humidity);
  */
}
