#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

// GPIO コマンド定義
#define CMD_SET_OUTPUT_LOW 0x01
#define CMD_SET_OUTPUT_HIGH 0x02
#define CMD_SET_OUTPUT_BLINK_250MS 0x03
#define CMD_SET_OUTPUT_BLINK_500MS 0x04
#define CMD_SET_OUTPUT_PWM 0x05
#define CMD_SET_OUTPUT_ON_DISCONNECT 0x09
#define CMD_SET_OUTPUT_WS2812B_ENABLE 0x11
#define CMD_SET_OUTPUT_WS2812B_BASECOLOR 0x12
#define CMD_SET_OUTPUT_WS2812B_PATTERN 0x13
#define CMD_SET_INPUT_FLOATING 0x81
#define CMD_SET_INPUT_PULLUP 0x82
#define CMD_SET_INPUT_PULLDOWN 0x83
#define CMD_SET_ADC_ENABLE 0x91
#define CMD_SET_ADC_DISABLE 0x92

// 内部用途予約 GPIO
#define GPIO_AUTH_ENABLE 5  // 認証機能有効/無効 (LOW: 有効, HIGH: 無効)
#define GPIO_PAIRING_MODE 4 // ペアリングモード (LOW: ペアリングモード, HIGH: 通常モード)

// GPIO 最大数
#define MAX_USABLE_GPIO 22 // 使用可能な GPIO の総数 (GPIO4, GPIO5 を除く)

// 入力ラッチモード定義
#define LATCH_MODE_NONE 0 // ラッチなし
#define LATCH_MODE_LOW 1  // LOWラッチ
#define LATCH_MODE_HIGH 2 // HIGHラッチ

// ポーリング設定
#define INPUT_POLL_INTERVAL_MS 10 // 入力ポーリング間隔 (ms)
#define LATCH_STABLE_COUNT 2      // ラッチ判定に必要な連続安定回数

// GPIO モード状態の定義
typedef enum
{
    BLEIO_MODE_UNSET = 0,      // モード未設定 (初期状態)
    BLEIO_MODE_INPUT_FLOATING, // ハイインピーダンス入力モード
    BLEIO_MODE_INPUT_PULLUP,   // 内部プルアップ付き入力モード
    BLEIO_MODE_INPUT_PULLDOWN, // 内部プルダウン付き入力モード
    BLEIO_MODE_OUTPUT_LOW,     // LOW (0V) 出力モード
    BLEIO_MODE_OUTPUT_HIGH,    // HIGH (3.3V) 出力モード
    BLEIO_MODE_BLINK_250MS,    // 250ms 点滅出力モード
    BLEIO_MODE_BLINK_500MS,    // 500ms 点滅出力モード
    BLEIO_MODE_PWM,            // PWM 出力モード
    BLEIO_MODE_ADC,            // ADC 入力モード
    BLEIO_MODE_WS2812B         // WS2812B シリアル LED 出力モード
} bleio_mode_state_t;

// GPIO ごとの状態管理
typedef struct
{
    bleio_mode_state_t mode;       // 現在のモード
    uint8_t current_level;         // 現在の出力レベル (点滅時に使用)
    uint8_t latch_mode;            // 入力ラッチモード (0-2)
    bool is_latched;               // ラッチ済みフラグ
    uint8_t stable_counter;        // 安定カウンタ
    uint8_t last_level;            // 前回の読み取り値
    uint8_t disconnect_behavior;   // BLE 切断時の振る舞い (0: 維持, 1: LOW, 2: HIGH)
} bleio_gpio_state_t;

// グローバル状態へのアクセサ
bleio_gpio_state_t* main_get_gpio_state(uint8_t pin);
portMUX_TYPE* main_get_gpio_states_mutex(void);
uint16_t main_get_conn_handle(void);
void main_set_conn_handle(uint16_t handle);

// 点滅カウンタ
uint8_t main_get_global_blink_counter(void);
uint8_t main_get_prev_blink_counter(void);
void main_set_prev_blink_counter(uint8_t value);

// GPIO 検証
bool main_is_valid_gpio(uint8_t pin);
bool main_is_valid_output_pin(uint8_t pin);

#endif // MAIN_H
