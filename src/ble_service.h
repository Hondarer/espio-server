#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

// BLE サービス初期化
void ble_service_init(bool auth_enabled);

// BLE アドバタイズ開始
void ble_service_start_advertising(void);

// BLE ホストタスク
void ble_service_host_task(void *param);

#endif // BLE_SERVICE_H
