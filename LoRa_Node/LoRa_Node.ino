#include <SPI.h>              // include libraries
#include <LoRa.h>
#include "HLW8012.h"
#include <ArduinoJson.h>

const long frequency = 915E6;  // LoRa Frequency

// GPIOs
#define RELAY_PIN                       13
#define SEL_PIN                         14
#define CF1_PIN                         27
#define CF_PIN                          26

//define the pins used by the transceiver module
#define ss 21
#define rst 23 
#define dio0 5

// Check values every 2 seconds
#define UPDATE_TIME                     2000

#define CURRENT_MODE                    HIGH

// These are the nominal values for the resistors in the circuit
#define CURRENT_RESISTOR                0.001
#define VOLTAGE_RESISTOR_UPSTREAM       ( 4 * 300000 + 4500) 
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 510 ) // Real 1.009k

HLW8012 hlw8012;
String dataFromGateway = "";
String dataToGateway = "";
JsonDocument doc;

int PWM_FREQUENCY = 10000; 
int PWM_CHANNEL = 0; 
int PWM_RESOUTION = 8; 
int GPIOPIN = 32 ; 
int val;


uint8_t receive_flag = 0;
uint8_t handle_flag = 0;
uint8_t feedback_flag = 0;
uint8_t trans_feedback_flag = 0;
uint8_t count200ms = 0, count300ms = 0, count500ms = 0, count1s = 0, count5s = 0;

uint8_t cnt_send = 0;

uint8_t state_bef = 0;
uint8_t state_cur = 0;
uint8_t flag200ms = 0, flag300ms = 0, flag500ms = 0, flag1s = 0, flag5s = 0;
void unblockingDelay(unsigned long mseconds) {
    unsigned long timeout = millis();
    while ((millis() - timeout) < mseconds) delay(1);
}

void calibrate() {

    hlw8012.getActivePower();

    hlw8012.setMode(MODE_CURRENT);
    unblockingDelay(2000);
    hlw8012.getCurrent();

    hlw8012.setMode(MODE_VOLTAGE);
    unblockingDelay(2000);
    hlw8012.getVoltage();

    // Calibrate using a 60W bulb (pure resistive) on a 230V line
    hlw8012.expectedActivePower(25.0);
    hlw8012.expectedVoltage(220.0);
    hlw8012.expectedCurrent(25.0 / 220.0);

    // Show corrected factors
    Serial.print("[HLW] New current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
    Serial.print("[HLW] New voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
    Serial.print("[HLW] New power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
    Serial.println();

}

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  SPI.begin(18, 19, 22, 21); 
  LoRa.setPins(ss, rst, dio0);
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOUTION);

  ledcAttachPin(GPIOPIN, PWM_CHANNEL);

  if (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  Serial.println();
  Serial.println("LoRa Simple Node");
  Serial.println("Only receive messages from gateways");
  Serial.println("Tx: invertIQ disable");
  Serial.println("Rx: invertIQ enable");
  Serial.println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();

  // Close the relay to switch on the load
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
  hlw8012.begin(CF_PIN, CF1_PIN, SEL_PIN, CURRENT_MODE, false, 500000);

  hlw8012.setResistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);

  
}

void loop() {

  if(runEvery(100))
  {
    count200ms ++;
    count300ms ++;
    count500ms ++;
    count1s ++;
    count5s ++;
    if(count200ms > 1)
    {
      flag200ms = 1;
      count200ms = 0;
    }

    if(count300ms > 2)
    {
      flag300ms = 1;
      count300ms = 0;
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
    if(count5s > 29)
    {
      flag5s = 1;
      count5s = 0;
    }
  }

  if(receive_flag == 1)
  {
    deserializeJson(doc, dataFromGateway);
    // "ID":"30187"
    if((doc["ID"] == "30205") || (doc["ID"] == "ALL"))
    {
      handle_flag = 1;
      receive_flag = 0;
      cnt_send = 0;
    }
  }

  if(handle_flag == 1)
  {
    if(doc["STATE"] == "ON")
    {
      Serial.println("denon");
      digitalWrite(RELAY_PIN, HIGH);
      val = doc["DIM"];
      val = map(val, 100, 0, 0, 255);
      analogWrite(GPIOPIN, val);
    }
    else if (doc["STATE"] == "OFF")
    {
      Serial.println("denoff");
      digitalWrite(RELAY_PIN, LOW);
    }

    feedback_flag = 1;
    handle_flag = 0;
    if((doc["ID"] == "30205"))
    {
      if((doc["ACK"] == "OK"))
      {
        feedback_flag = 0;
      }
    }
  }


  if(flag200ms == 1)
  {
    flag200ms = 0;
  }

  if(flag300ms == 1)
  {
    hlw8012.toggleMode();
    flag300ms = 0;
  }


  if (flag500ms == 1) { // repeat every 1000 millis

    flag500ms = 0;
  }

  if(flag1s == 1)
  {
    if(hlw8012.getActivePower() > 10)
    {
      Serial.println("den da on");
      dataToGateway = "{\"ID\":\"30205\",\"STATUS\":\"ON\"}";
      state_cur = 1;
    }
    else
    {
      Serial.println("den da off");
      dataToGateway = "{\"ID\":\"30205\",\"STATUS\":\"OFF\"}";
      state_cur = 0;
    }
    
    if(state_cur != state_bef)
    {
      feedback_flag = 1;
      state_bef = state_cur;
    }

    flag1s = 0;

  }
  if(flag5s == 1)
  {
    if(feedback_flag == 1)
    {
      if(hlw8012.getActivePower() > 10)
      {
        Serial.println("den da on");
        dataToGateway = "{\"ID\":\"30205\",\"STATUS\":\"ON\"}";
      }
      else
      {
        Serial.println("den da off");
        dataToGateway = "{\"ID\":\"30205\",\"STATUS\":\"OFF\"}";
      }
      LoRa_sendMessage(dataToGateway);
      cnt_send ++;
      if(cnt_send > 10)
      {
        feedback_flag = 0;
      }
    }
    Serial.println("task 5s");
    flag5s = 0;
  }

  delay(100);
}

void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode(){
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize) {
  String message = "";

  while (LoRa.available()) {
    message += (char)LoRa.read();
  }
  dataFromGateway = message;
  Serial.print("Node Receive: ");
  Serial.println(dataFromGateway);
  receive_flag = 1;
}

void onTxDone() {
  Serial.println("TxDone");
  trans_feedback_flag = 0;
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

