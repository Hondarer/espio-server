#ifndef GPIO_PWM_H
#define GPIO_PWM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// PWM 設定
#define BLEIO_LEDC_CHANNEL_MAX 8                     // Low Speed モードで使用可能なチャネル数
#define LEDC_TIMER_RESOLUTION LEDC_TIMER_8_BIT // 8 ビット分解能 (0-255)
#define LEDC_TIMER_NUM LEDC_TIMER_0            // 使用するタイマー番号
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE

// PWM 設定保持用構造体
typedef struct
{
    uint8_t duty_cycle;  // デューティサイクル (0-255)
    uint8_t freq_preset; // 周波数プリセット (0-7)
    int8_t channel;      // 割り当てられた LEDC チャネル (-1: 未割り当て)
} pwm_config_t;

// LEDC チャネル管理用構造体
typedef struct
{
    bool in_use;      // チャネルが使用中か
    uint8_t gpio_num; // このチャネルを使用している GPIO 番号
} ledc_channel_info_t;

// PWM 制御
esp_err_t gpio_pwm_set(uint8_t pin, uint8_t duty_cycle, uint8_t freq_preset);
void gpio_pwm_stop(uint8_t pin);

// 初期化
void gpio_pwm_init(void);

// 状態アクセサ
pwm_config_t* gpio_pwm_get_config(uint8_t pin);
ledc_channel_info_t* gpio_pwm_get_ledc_channels(void);

#endif // GPIO_PWM_H
