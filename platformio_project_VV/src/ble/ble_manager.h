#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include "configuration.h"


// BLE Configuration
#define BLE_CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-123456789abc" // Custom Characteristic UUID
#define BLE_SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"

CommandType parseCommand(String command);
void test_bluetooth();
void setup_ble();

#endif // BLE_MANAGER_H