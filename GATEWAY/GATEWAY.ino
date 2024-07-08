#include <SPI.h>              // include libraries
#include <LoRa.h>
#include "EspMQTTClient.h"
#include <ArduinoJson.h>

#define TINY_GSM_MODEM_SIM7600

#define SerialMon Serial
#define TINY_GSM_DEBUG SerialMon

#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

#define TINY_GSM_USE_GPRS true

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT details
const char* broker = "54.249.187.78";

const char* topicState  = "hust/HUSTGW0001/cmd";
const char* topicStatus = "hust/HUSTGW0001/report";

#include <TinyGsmClient.h>

#include <PubSubClient.h>

#include <ArduinoJson.h>

TinyGsm        modem(Serial2);  

TinyGsmClient client(modem);
PubSubClient  mqtt(client);

int ledStatus = LOW;

uint32_t lastReconnectAttempt = 0;

#define SerialAT  Serial2

const long frequency = 915E6;  // LoRa Frequency
String jsonString;
String data_public;
String data_ACK;
byte* data_transmit;

unsigned int len_data = 0;
JsonDocument doc;
JsonDocument doc1;
String ID;

uint8_t data = 0;
uint8_t count200ms = 0, count500ms = 0, count1s = 0;
uint8_t flag200ms = 0, flag500ms = 0, flag1s = 0;
uint8_t flagReceiveFromMqtt = 0;
uint8_t flagSendToMqtt = 0;
uint8_t flagSendAckToNema = 0;
#define ss 19
#define rst 21 
#define dio0 22



void mqttCallback(char* topic, byte* payload, unsigned int len) {
  data_transmit = payload;
  len_data = len;
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  flagReceiveFromMqtt = 1;

  // Only proceed if incoming message's topic matches
  // if (String(topic) == topicLed) {
  //   ledStatus = !ledStatus;
  //   mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");
  // }
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  boolean status = mqtt.connect("DATN", "device", "w5DirAKQDUhDcAh");

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  mqtt.publish(topicStatus, "DATN started");
  mqtt.subscribe(topicState);
  return mqtt.connected();
}



void setup() {
  SerialMon.begin(115200);
  delay(10);
    //     void begin(int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);
  SPI.begin(17, 5, 18, 19); 
  LoRa.setPins(ss, rst, dio0);

    if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  Serial.println();
  Serial.println("LoRa Simple Gateway");
  Serial.println("Only receive messages from nodes");
  Serial.println("Tx: invertIQ enable");
  Serial.println("Rx: invertIQ disable");
  Serial.println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();

  SerialMon.println("Wait...");
  TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
    // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
    // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) { SerialMon.println("Network connected"); }

  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
  if (modem.isGprsConnected()) { SerialMon.println("GPRS connected"); }

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
}

void loop() {
  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, true)) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected()) {
      SerialMon.println("Network re-connected");
    }

    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      SerialMon.println("GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected()) { SerialMon.println("GPRS reconnected"); }
    }
  }

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) { lastReconnectAttempt = 0; }
    }
    delay(100);
    return;
  }

  if(flagSendToMqtt == 1)
  {
    deserializeJson(doc, data_public);
    if(doc["STATUS"] == "ON" || doc["STATUS"] == "OFF")
    {
      mqtt.publish(topicStatus, (const char*)data_public.c_str());
      doc1["ID"] = doc["ID"];
      doc1["ACK"] = "OK";
      flagSendAckToNema = 1;
    }

    flagSendToMqtt = 0;
  }

  
  if(runEvery(100))
  {
    count200ms ++;
    count500ms ++;
    count1s ++;
    if(count200ms > 1)
    {
      flag200ms = 1;
      count200ms = 0;
    }

    if(count500ms > 4)
    {
      flag500ms = 1;
      count500ms = 0;
    }

    if(count1s > 9)
    {
      flag1s = 1;
      count1s = 0;
    }
  }

  // Do Nothing!
  if(flag200ms == 1)
  {
//    Serial.println("task 200ms");
    flag200ms = 0;

    if(flagReceiveFromMqtt == 1)
    {
//      LoRa_sendMessage(data_transmit, len_data);
      SerialMon.write(data_transmit, len_data);
      SerialMon.println();
      LoRa_txMode();                        // set tx mode
      LoRa.beginPacket();                   // start packet
    //  LoRa.print(message);                  // add payload
      LoRa.write(data_transmit, len_data);
      LoRa.endPacket(true); 
      flagReceiveFromMqtt = 0;
    }

    if(flagSendAckToNema == 1)
    {
      serializeJson(doc1, data_ACK);
      Serial.println(data_ACK);
      LoRa_txMode();                        // set tx mode
      LoRa.beginPacket();                   // start packet
      LoRa.print(data_ACK);                  // add payload
      LoRa.endPacket(true);                 // finish packet and send it
      
      flagSendAckToNema = 0;
    }
  }

      // Do Nothing!
  if(flag500ms == 1)
  {
    flag500ms = 0;
  }


      // Do Nothing!
  if(flag1s == 1)
  {
    flag1s = 0;
//    mqtt.publish(topicStatus, (const char*)message.c_str());
//    mqtt.publish(topicStatus, "hello");
  }

  delay(100);


  mqtt.loop();
}



void LoRa_rxMode(){
  LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();               
           // set standby mode
  LoRa.enableInvertIQ();                // active invert I and Q signals
}

void LoRa_sendMessage(byte* payload, unsigned int len) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
//  LoRa.print(message);                  // add payload
  LoRa.write(payload, len);
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }
  Serial.print("Gateway Receive: ");
  Serial.println(message);
  data_public = message;
  flagSendToMqtt = 1;
}

void onTxDone() {
  Serial.println("TxDone");
  LoRa_rxMode();
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}





















