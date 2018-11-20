/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Server.h"

// Supports interrupts on ESP32
#define SENSORPIN 25 

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
// uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

# define LED 2

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
      deviceConnected = true;
      digitalWrite(LED, HIGH);
    };

    void onDisconnect(BLEServer *pServer) {
      deviceConnected = false;
      digitalWrite(LED, LOW);
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

volatile uint8_t ticks;

//DEBUG: Remove in PROD
static uint8_t sendticks;

void Flow_ISR(){
  // Increment ticks with every trigger
  ticks ++ ;
  }
  
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  pinMode(2, OUTPUT);

  // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);
                      
  pTxCharacteristic->addDescriptor(new BLE2902());
  

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  //Attaching Interrupt
  attachInterrupt(digitalPinToInterrupt(SENSORPIN), Flow_ISR, RISING);

  //Enable interrupts
  interrupts();

  //DEBUG: Remove in PROD
  sendticks = 0;
}

void loop() {
  //Reset ticks to zero with every loop
    ticks = 0;

    //DEBUG: ACTIVATE in PROD
    //uint8_t sendTicks = 0;
    
  //Keep interrupts on for 10s
  delay(10000);

  //Turn off interrupts to offload counted values 
  //TODO: Investigate whether turning off these interrupts disables BlueTooth ?
  noInterrupts();

 // sendTicks = ticks;
    sendticks ++;
    
  //Turn interrupts back on. This operation should take microseconds
  interrupts();

  //Debug Point
  Serial.print("Virtual Ticks: ");
  Serial.println(sendticks);
  
    if (deviceConnected) {
       // digitalWrite(2, HIGH);
        Serial.println(ticks);
       // pTxCharacteristic->setValue(&sendTicks, 1);
       pTxCharacteristic->setValue(String(sendticks).c_str());
       pTxCharacteristic->notify();
        
		delay(30); // bluetooth stack will go into congestion, if too many packets are sent
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
      //  digitalWrite(2, LOW);
        Serial.println("Disconnecting");
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
    Serial.println("Checking for Connection");
        oldDeviceConnected = deviceConnected;
    }
}
