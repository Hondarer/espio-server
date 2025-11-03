#ifndef GPIO_ADC_H
#define GPIO_ADC_H

#include <stdint.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

// ADC 設定保持用構造体
typedef struct
{
    int8_t channel;                // ADC1 チャネル (-1: 未設定)
    adc_atten_t attenuation;       // 減衰率
    bool calibrated;               // キャリブレーション済みか
    adc_cali_handle_t cali_handle; // キャリブレーションハンドル
} adc_config_t;

// ADC 制御
esp_err_t gpio_adc_enable(uint8_t pin, uint8_t atten_param);
esp_err_t gpio_adc_disable(uint8_t pin);
uint16_t gpio_adc_read_value(uint8_t pin);
void gpio_adc_stop(uint8_t pin);

// 初期化
void gpio_adc_init(void);

// 状態アクセサ
adc_config_t* gpio_adc_get_config(uint8_t pin);

#endif // GPIO_ADC_H
