/*
  ESP32_ArmCrawler_BluefruitConnect.ino by hikabec
  
  Bluefruit Connect
  https://apps.apple.com/jp/app/bluefruit-connect/id830125974
  https://play.google.com/store/apps/details?id=com.adafruit.bluefruit.le.connect
  
*/

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

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Add
#define PIN_L1 25
#define PIN_L2 26
#define PIN_R1 32
#define PIN_R2 33

// Add
void motor_stop(int pin1, int pin2) {  
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
}

// Add
void motor_forward(int pin1, int pin2) {
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
}

// Add
void motor_back(int pin1, int pin2) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        // Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        
        // UP
        if (rxValue == "!B516") {
          motor_forward(PIN_L1, PIN_L2);
          motor_forward(PIN_R1, PIN_R2);

        } else if (rxValue == "!B507") {
          motor_stop(PIN_L1, PIN_L2);
          motor_stop(PIN_R1, PIN_R2);
        }

        // DOWN
        if (rxValue == "!B615") {
          motor_back(PIN_L1, PIN_L2);
          motor_back(PIN_R1, PIN_R2);
        } else if (rxValue == "!B606") {
          motor_stop(PIN_L1, PIN_L2);
          motor_stop(PIN_R1, PIN_R2);
        }

        // LEFT
        if (rxValue == "!B714") {
          motor_back(PIN_L1, PIN_L2);
          motor_forward(PIN_R1, PIN_R2);
        } else if (rxValue == "!B705") {
          motor_stop(PIN_L1, PIN_L2);
          motor_stop(PIN_R1, PIN_R2);
        }

        // RIGHT
        if (rxValue == "!B813") {
           motor_forward(PIN_L1, PIN_L2);
           motor_back(PIN_R1, PIN_R2);
        } else if (rxValue == "!B804") {
          motor_stop(PIN_L1, PIN_L2);
          motor_stop(PIN_R1, PIN_R2);
        }
        
        Serial.println();
        // Serial.println("*********");
      }
    }
};


void setup() {
  Serial.begin(115200);
  
  // Add
  pinMode(PIN_L1, OUTPUT);
  pinMode(PIN_L2, OUTPUT);
  pinMode(PIN_R1, OUTPUT);
  pinMode(PIN_R2, OUTPUT);
  
  // Create the BLE Device
  BLEDevice::init("ESP32 ArmCrawler");

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
}

void loop() {

    if (deviceConnected) {
        pTxCharacteristic->setValue(&txValue, 1);
        pTxCharacteristic->notify();
        txValue++;
		    delay(10); // bluetooth stack will go into congestion, if too many packets are sent
	  }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
