#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <Arduino.h>
#include "ble_manager.h"
#include "configuration.h"


// BLE Objects
BLECharacteristic *pCharacteristic;

bool deviceConnected = false;

// Helper function to parse command string to CommandType enum
CommandType parseCommand(String command) {
  command.trim();
  command.toUpperCase();
  if (command.startsWith("LED")) return CMD_LED;
  if (command.startsWith("SPI")) return CMD_SPI;
  if (command.startsWith("BUZZER")) return CMD_BUZZER;
  if (command.startsWith("I2C")) return CMD_I2C;
  if (command.startsWith("CTN")) return CMD_CTN;
  if (command.startsWith("INA")) return CMD_INA;
  return CMD_NONE;
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { 
    deviceConnected = true; 
    Serial.println("Central Connected.");
  }
  void onDisconnect(BLEServer* pServer) {
     deviceConnected = false; 
     Serial.println("Central Disconnected.");
    }


};


void setup_ble() {
  BLEDevice::init("ESP32-Yaxsomo");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
      BLE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY
  );

  // Add a descriptor to enable notifications
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  Serial.println("BLE initialized and advertising as ESP32-Yaxsomo");
}

void test_bluetooth() {
  Serial.println("Testing BLE (notify)...");
  pCharacteristic->setValue("Hello from ESP32 via BLE!");
  pCharacteristic->notify();
  Serial.println("BLE notification sent.");
}