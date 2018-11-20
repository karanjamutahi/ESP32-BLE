  /*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <HardwareSerial.h>
//#include <string.h>

// #define UART_DEBUG_PORT   1
#define UART_GPS_PORT     0
#define UART_GSM_PORT     2

#define UART_GPS_TX       1
#define UART_GPS_RX       3

#define UART_GSM_TX       17
#define UART_GSM_RX       16

//#define UART_DEBUG_TX     10
//#define UART_DEBUG_RX     9

#define LED 2

//HardwareSerial Serial(UART_DEBUG_PORT);

HardwareSerial GPS_Serial(UART_GPS_PORT);
HardwareSerial GSM_Serial(UART_GSM_PORT);


int scanTime = 15; //In seconds

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    }
};


void setup() {
  GSM_Serial.begin(9600,SERIAL_8N1, 9, 10 );
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Scanning...");
  
  pinMode(LED, OUTPUT);
  
  //Debug Point
  blink();
  
  BLEDevice::init("Cloud Logger");
  BLEScan* pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  BLEScanResults foundDevices = pBLEScan->start(scanTime);
  GPS_Serial.print("Devices found: ");
  GPS_Serial.println(foundDevices.getCount());
  Serial.print("Devices found: ");
  Serial.println(foundDevices.getCount());
  Serial.println("Scan done! Connecting");
 }

void blink(){
  digitalWrite(LED, HIGH);
  delay(350);
  digitalWrite(LED, LOW);
  delay(300);
  }
  
void loop() {
  Serial.print("Pulling Values");
  //String ticks = 
  
  Serial.print("Ticks: ");
  Serial.println(BLEDevice::getValue((BLEAddress)"30:ae:a4:27:78:12" ,(BLEUUID)"6E400001-B5A3-F393-E0A9-E50E24DCCA9E", (BLEUUID)"6E400003-B5A3-F393-E0A9-E50E24DCCA9E").c_str());
  delay(300);
// GPS_Serial.print ("Ticks: ");
// GPS_Serial.println (ticks);
  
  // put your main code here, to run repeatedly:
  blink();
}
