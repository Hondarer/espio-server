#ifndef WS28XX_H
#define WS28XX_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/rmt_tx.h"

// WS2812B 設定
#define WS2812B_MAX_LEDS 256                   // 1つの GPIO あたり最大 LED 数
#define WS2812B_RMT_RESOLUTION_HZ 10000000     // RMT 解像度 10MHz (0.1us 刻み)
#define WS2812B_T0H_TICKS 4                    // 0 ビット HIGH 時間 0.4us (4 × 0.1us)
#define WS2812B_T0L_TICKS 9                    // 0 ビット LOW 時間 0.9us (9 × 0.1us)
#define WS2812B_T1H_TICKS 8                    // 1 ビット HIGH 時間 0.8us (8 × 0.1us)
#define WS2812B_T1L_TICKS 5                    // 1 ビット LOW 時間 0.5us (5 × 0.1us)
#define WS2812B_RESET_US 50                    // リセット信号時間 50us

// WS2812B パターン定義
#define WS2812B_PATTERN_ON 0            // 常時点灯 (デフォルト)
#define WS2812B_PATTERN_BLINK_250MS 1   // 250ms 点灯 / 250ms 消灯
#define WS2812B_PATTERN_BLINK_500MS 2   // 500ms 点灯 / 500ms 消灯
#define WS2812B_PATTERN_RAINBOW 3       // 虹色パターン
#define WS2812B_PATTERN_FLICKER1 4      // 炎のゆらめきパターン (GPIO 共有明度、色相ゆらぎなし)
#define WS2812B_PATTERN_FLICKER2 5      // 炎のゆらめきパターン (LED 個別明度、色相ゆらぎなし)
#define WS2812B_PATTERN_FLICKER3 6      // 炎のゆらめきパターン (LED 個別明度、色相ゆらぎあり)
#define WS2812B_PATTERN_UNSET 0xFF      // 未設定 (個別 LED パターン用)

// FLICKER パターンの色相ゆらぎ係数
// 0 で色相ゆらぎ無効、32 で range=128 時に約 ±2000 (色相環の約 ±3%) のゆらぎ
#define WS2812B_FLICKER_HUE_VARIATION 8

// シリアル LED 種別 (カラーオーダー)
typedef enum {
    LED_TYPE_WS2812B_RGB = 0,   // WS2812B バリアント (RGB 順、デフォルト)
    LED_TYPE_WS2812B_GRB = 1,   // WS2812B 標準仕様 (GRB 順)
    LED_TYPE_WS2811 = 2         // WS2811 (RGB 順)
} serial_led_type_t;

// WS2812B LED 個別パターン設定構造体
typedef struct
{
    uint8_t pattern_type;   // パターンタイプ (0-4, 0xFF=未設定)
    uint8_t pattern_param1; // パラメータ1 (RAINBOW: 色相が一周する LED 個数、FLICKER: ゆらめきの速度)
    uint8_t pattern_param2; // パラメータ2 (RAINBOW: 変化スピード、FLICKER: ゆらめきの変化幅)
    uint16_t hue;           // RAINBOW 用の現在色相 (0-65535)
} ws2812b_led_pattern_t;

// WS2812B FLICKER パターン用のデータ構造
typedef struct
{
    uint16_t base_hue;      // 基準色の色相 (キャッシュ)
    uint8_t base_sat;       // 基準色の彩度 (キャッシュ)
    uint8_t base_val;       // 基準色の明度 (キャッシュ)
    uint32_t seed;          // 疑似乱数の状態
    uint8_t val_ema;        // 明度の平滑値 (現在値) 0-255
    uint16_t hue_ema;       // 色相の平滑値 (現在値) 0-65535
    float x;                // 間欠カオス法の状態変数 (明度用) (0.0-1.0)
    uint8_t count;          // 明度移行カウンタ
    uint8_t maxloop;        // 明度移行目標値
    float hue_offset;       // 色相オフセットの平滑値 (-1.0-1.0)
    float hue_offset_target; // 色相オフセットの目標値 (-1.0-1.0)
} led_flicker_data_t;

// WS2812B 設定保持用構造体
typedef struct
{
    uint16_t num_leds;                  // LED 個数
    uint8_t brightness;                 // 基準輝度 (0-255)
    serial_led_type_t led_type;         // LED 種別 (カラーオーダー)
    uint8_t *led_data;                  // LED データバッファ (RGB 形式、3 バイト × LED 個数)
    rmt_channel_handle_t rmt_channel;   // RMT チャネルハンドル
    rmt_encoder_handle_t rmt_encoder;   // RMT エンコーダハンドル

    // パターン関連
    ws2812b_led_pattern_t gpio_pattern; // GPIO 全体のデフォルトパターン (LED 番号 0)
    ws2812b_led_pattern_t *led_patterns; // LED 個別のパターン配列 (num_leds 個、動的割り当て)
    uint8_t *base_colors;               // LED ごとのベースカラー (RGB 形式、3 バイト × num_leds)
    led_flicker_data_t *flicker_data;   // FLICKER2/3 パターン用のデータ (num_leds 個、動的割り当て)
    led_flicker_data_t gpio_flicker1_shared; // FLICKER1 用の GPIO 共有状態
} ws2812b_config_t;

// WS2812B 制御
esp_err_t ws2812b_enable(uint8_t pin, uint16_t num_leds, uint8_t brightness, serial_led_type_t led_type);
esp_err_t ws2812b_set_color(uint8_t pin, uint16_t led_index, uint8_t r, uint8_t g, uint8_t b);
esp_err_t ws2812b_set_pattern(uint8_t pin, uint8_t led_index, uint8_t pattern_type,
                               uint8_t param1, uint8_t param2);
esp_err_t ws2812b_turn_off_all_leds(uint8_t pin);
void ws2812b_stop(uint8_t pin);

// 初期化
void ws2812b_init(void);

// パターン更新 (タイマーから呼び出される)
void ws2812b_update_patterns(uint8_t blink_counter);

// 状態アクセサ
ws2812b_config_t* ws2812b_get_config(uint8_t pin);

#endif // WS28XX_H
