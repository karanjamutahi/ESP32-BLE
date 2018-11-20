/** * A BLE client example that is rich in capabilities. */ 
#include "BLEDevice.h" 
#include "BLEScan.h" 
#include <HardwareSerial.h>
//#include <ArduinoTrace.h>

#define LED 2

// The remote service we wish to connect to. 
static BLEUUID serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"); 

// The characteristic of the remote service we are interested in. 
static BLEUUID charUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

static BLEAddress *pServerAddress; 
static boolean doConnect = false; 
static boolean connected = false; 
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEClient* pClient; 
static uint8_t BLEConnect = 0;


class MyClientCallbacks: public BLEClientCallbacks 
  {
    
    void onDisconnect(BLEClient *pClient)
    {
      BLEConnect = 0;
    }

    void onConnect(BLEClient *pClient)
    {
      BLEConnect = 1;
    }
    
  };


static void notifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) 
{ 
  blinker(350);
  Serial.print("Notify callback for characteristic ");     
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str()); 
  Serial.print(" of data length "); 
  Serial.println(length); 
  
  //Serial.print("Pulled Data: ");
  //Serial.println(pClient->getValue(serviceUUID, charUUID).c_str());
  
  //Serial.println(pRemoteCharacteristic->readValue().c_str());
  //TRACE();
} 

bool connectToServer(BLEAddress pAddress) {
  Serial.print("Forming a connection to "); 
  Serial.println(pAddress.toString().c_str()); 
  BLEClient* pClient = BLEDevice::createClient(); 
  Serial.println(" - Created client"); 

  //Set CBs
  pClient->setClientCallbacks(new MyClientCallbacks() ); 
  
  // Connect to the remote BLE Server. 
  pClient->connect(pAddress); 
  Serial.println(" - Connected to server"); 
  
  // Obtain a reference to the service we are after in the remote BLE server. 
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID); 
  if (pRemoteService == nullptr) 
  { 
    Serial.print("Failed to find our service UUID: "); 
    Serial.println(serviceUUID.toString().c_str()); 
    return false; 
  } 
  
  Serial.println(" - Found our service"); 
  
  // Obtain a reference to the characteristic in the service of the remote BLE server. 
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID); 
  if (pRemoteCharacteristic == nullptr) 
  {
    Serial.print("Failed to find our characteristic UUID: "); 
    Serial.println(charUUID.toString().c_str()); 
    return false; 
  } 
  
  Serial.println(" - Found our characteristic"); 
  
  // Read the value of the characteristic. 
  std::string value = pRemoteCharacteristic->readValue(); 
  Serial.print("The characteristic value was: "); 
  Serial.println(value.c_str()); 
  
  pRemoteCharacteristic->registerForNotify(notifyCallback); 
  doConnect = false ;
} 

/** * Scan for BLE servers and find the first one that advertises the service we are looking for. */ 

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks 
{ 
  /** * Called for each advertising BLE server. */ 
  void onResult(BLEAdvertisedDevice advertisedDevice) 
  { 
  Serial.print("BLE Advertised Device found: ");
  Serial.println(advertisedDevice.toString().c_str()); 
  
    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) 
    { 
    // 
    Serial.print("Found our device! address: "); 
    advertisedDevice.getScan()->stop(); 
    pServerAddress = new BLEAddress(advertisedDevice.getAddress()); 
    doConnect = true; 
    } // Found our server 
  } // onResult 
}; // MyAdvertisedDeviceCallbacks 

void blinker(uint16_t msecs)
{
  digitalWrite(LED, HIGH);
  delay(msecs);
  digitalWrite(LED, LOW);
  delay(msecs);
}

void setup() 
{ 

  //Set Up GPS & GSM Serial
  HardwareSerial GSM_Serial(0);
  HardwareSerial GPS_Serial(2);
  GSM_Serial.begin(9600, SERIAL_8N1, 1, 1);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  
  Serial.begin(115200); 
  Serial.setDebugOutput(true);
  Serial.println("Starting Arduino BLE Client application..."); 
  BLEDevice::init(""); 
  // Retrieve a Scanner and set the callback we want to use to be informed when we // have detected a new device. Specify that we want active scanning and start the // scan to run for 30 seconds. 
  BLEScan* pBLEScan = BLEDevice::getScan(); 
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); 
  pBLEScan->setActiveScan(true); 
  pBLEScan->start(15);
  
  if(connectToServer((BLEAddress) "30:ae:a4:27:78:12"))
  {
    blinker(1000);

    //Dont try to connect in Loop
    doConnect = false;
  };
} // End of setup. 

// This is the Arduino main loop function. 
void loop() 
{ 
  // If the flag "doConnect" is true then we have scanned for and found the desired // BLE Server with which we wish to connect. Now we connect to it. Once we are // connected we set the connected flag to be true. 
  
  //TRACE();

  if (doConnect == true) 
  { 
    if (connectToServer(*pServerAddress)) 
    { 
      Serial.println("We are now connected to the BLE Server."); 
      connected = true; 
    } 
    else 
    { 
      Serial.println("We have failed to connect to the server; there is nothin more we will do."); 
    } 
    doConnect = false; 
  } 
  
  // If we are connected to a peer BLE Server, update the characteristic each time we are reached // with the current time since boot. 
  if (connected) { 
    
   // TRACE();
   
   // String newValue = "Time since boot: " + String(millis()/1000); 
   // Serial.println("Setting new characteristic value to \"" + newValue + "\""); 
    // Set the characteristic's value to be the array of bytes that is actually a string. 
  //  pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length()); 
  } 

  // BLE is not connected
  if (BLEConnect == 0)
  {
    connectToServer((BLEAddress) "30:ae:a4:27:78:12");
  }
  
  // BLE is connected
  else if(BLEConnect == 1)
  {
    // don't try to reconnect
    blinker(200);
  }

  //Send GPS & Flow via MQTT
  blinker (500);
  delay(1000); // Delay a second between loops. 
} // End of loop
