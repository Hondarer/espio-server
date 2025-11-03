#ifndef GPIO_BASIC_H
#define GPIO_BASIC_H

#include <stdint.h>
#include "esp_err.h"

// GPIO 基本制御
esp_err_t gpio_basic_set_mode(uint8_t pin, uint8_t command, uint8_t latch_mode);
esp_err_t gpio_basic_write_level(uint8_t pin, uint8_t command);
esp_err_t gpio_basic_start_blink(uint8_t pin, uint8_t command);
esp_err_t gpio_basic_set_disconnect_behavior(uint8_t pin, uint8_t behavior);

// タイマーコールバックから呼び出される更新処理
void gpio_basic_update_blink_outputs(uint8_t global_blink_counter);
void gpio_basic_update_input_latches(void);

// 初期化
void gpio_basic_init(void);

#endif // GPIO_BASIC_H
