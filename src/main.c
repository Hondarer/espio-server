#include "main.h"
#include "gpio_basic.h"
#include "gpio_pwm.h"
#include "gpio_adc.h"
#include "ws28xx.h"
#include "ble_service.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "esp_bt.h"
#include "store/config/ble_store_config.h"

static const char *TAG = "BLEIO-MAIN";

// REQUIRED_MTU と PAYLOAD_SIZE の定義
#define REQUIRED_MTU 136
#define PAYLOAD_SIZE 133

// グローバル変数
static uint16_t conn_handle = 0;
static bleio_gpio_state_t gpio_states[40] = {0};
static portMUX_TYPE gpio_states_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t global_blink_counter = 0;
static uint8_t prev_blink_counter = 0xFF;
static esp_timer_handle_t periodic_timer = NULL;

// アクセサ関数の実装
bleio_gpio_state_t* main_get_gpio_state(uint8_t pin)
{
    return &gpio_states[pin];
}

portMUX_TYPE* main_get_gpio_states_mutex(void)
{
    return &gpio_states_mux;
}

uint16_t main_get_conn_handle(void)
{
    return conn_handle;
}

void main_set_conn_handle(uint16_t handle)
{
    conn_handle = handle;
}

uint8_t main_get_global_blink_counter(void)
{
    return global_blink_counter;
}

uint8_t main_get_prev_blink_counter(void)
{
    return prev_blink_counter;
}

void main_set_prev_blink_counter(uint8_t value)
{
    prev_blink_counter = value;
}

// GPIO 検証関数
bool main_is_valid_gpio(uint8_t pin)
{
    // GPIO4, GPIO5 は内部用途に予約されているため除外
    if (pin == GPIO_AUTH_ENABLE || pin == GPIO_PAIRING_MODE)
    {
        return false;
    }

    // 使用可能な GPIO ピン
    if (pin == 2 || (pin >= 12 && pin <= 19) ||
        (pin >= 21 && pin <= 27) || (pin >= 32 && pin <= 36) || pin == 39)
    {
        return true;
    }
    return false;
}

bool main_is_valid_output_pin(uint8_t pin)
{
    // 入力専用ピン (GPIO34, 35, 36, 39) を除外
    if (pin >= 34 && pin <= 39)
    {
        return false;
    }
    return main_is_valid_gpio(pin);
}

// 周期タイマコールバック (10ms 周期)
static void periodic_timer_callback(void *arg)
{
    // 10ms カウンタ (0-24 で 250ms 周期)
    static uint8_t tick_counter = 0;

    tick_counter++;

    // 250ms ごとに点滅カウンタを更新 (25 ticks = 250ms)
    if (tick_counter >= 25)
    {
        tick_counter = 0;

        // グローバルカウンタを更新 (0-3 の範囲)
        global_blink_counter++;
        if (global_blink_counter >= 4)
        {
            global_blink_counter = 0;
        }

        // GPIO 基本モジュールの点滅出力を更新
        gpio_basic_update_blink_outputs(global_blink_counter);
    }

    // GPIO 基本モジュールの入力ラッチを更新 (10ms ごと)
    gpio_basic_update_input_latches();

    // WS2812B パターンを更新 (10ms ごと、点滅カウンタを渡す)
    ws2812b_update_patterns(global_blink_counter);
}

// 認証機能とペアリングモードの判定
static bool is_auth_enabled(void)
{
    // GPIO5 をプルアップ付き入力として設定
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_AUTH_ENABLE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    vTaskDelay(pdMS_TO_TICKS(10)); // 安定待ち

    int level = gpio_get_level(GPIO_AUTH_ENABLE);
    return (level == 0); // LOW の場合 true (認証有効)
}

static bool is_pairing_mode_requested(void)
{
    // GPIO4 をプルアップ付き入力として設定
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_PAIRING_MODE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    vTaskDelay(pdMS_TO_TICKS(10)); // 安定待ち

    int level = gpio_get_level(GPIO_PAIRING_MODE);
    return (level == 0); // LOW の場合 true (ペアリングモード)
}

static void clear_bonding_info(void)
{
    ESP_LOGI(TAG, "Clearing bonding information...");

    // NimBLE のボンディング情報をクリア
    ble_store_clear();

    ESP_LOGI(TAG, "Bonding information cleared");
}

// メイン関数
void app_main(void)
{
    ESP_LOGI(TAG, "Starting BLEIO-ESP32 Service");

    // GPIO 状態の初期化
    for (int i = 0; i < 40; i++)
    {
        gpio_states[i].mode = BLEIO_MODE_UNSET;
        gpio_states[i].current_level = 0;
        gpio_states[i].latch_mode = LATCH_MODE_NONE;
        gpio_states[i].is_latched = false;
        gpio_states[i].stable_counter = 0;
        gpio_states[i].last_level = 0;
        gpio_states[i].disconnect_behavior = 0;
    }

    // 各モジュールの初期化
    gpio_basic_init();
    gpio_pwm_init();
    gpio_adc_init();
    ws2812b_init();

    // 周期タイマの初期化 (10ms 周期: 入力ポーリング、点滅生成、LED パターン更新)
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "periodic_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, INPUT_POLL_INTERVAL_MS * 1000)); // 10ms = 10000us
    ESP_LOGI(TAG, "Periodic timer started (%dms interval)", INPUT_POLL_INTERVAL_MS);

    // 認証機能の有効/無効をチェック
    bool auth_mode_enabled = is_auth_enabled();

    if (auth_mode_enabled)
    {
        ESP_LOGI(TAG, "Authentication enabled (GPIO%d = LOW)", GPIO_AUTH_ENABLE);

        // ペアリングモードのチェック (認証有効時のみ)
        bool pairing_mode = is_pairing_mode_requested();

        if (pairing_mode)
        {
            ESP_LOGI(TAG, "Starting in pairing mode (GPIO%d = LOW) - clearing bonding information", GPIO_PAIRING_MODE);
        }
        else
        {
            ESP_LOGI(TAG, "Starting in authentication mode (GPIO%d = HIGH)", GPIO_PAIRING_MODE);
        }
    }
    else
    {
        ESP_LOGI(TAG, "Authentication disabled (GPIO%d = HIGH)", GPIO_AUTH_ENABLE);
    }

    // BLE サービス初期化
    ble_service_init(auth_mode_enabled);

    // 認証有効時のみボンディング情報のクリア
    if (auth_mode_enabled && is_pairing_mode_requested())
    {
        clear_bonding_info();
    }

    // BLE 送信電力を設定 (電力変動を抑制)
    // ESP_PWR_LVL_N12 ~ ESP_PWR_LVL_P9 の範囲で設定可能
    // ESP_PWR_LVL_P3 (3dBm) 程度が安定性と到達距離のバランスが良い
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P3);
    ESP_LOGI(TAG, "BLE TX power set to +3dBm for stable power consumption");

    // ATT MTU を設定 (最大 22 コマンドまで送信可能)
    // 必要 MTU = ATT ヘッダ (3) + コマンド個数 (1) + コマンド (22 * 6) = 136 バイト
    ble_att_set_preferred_mtu(REQUIRED_MTU);
    ESP_LOGI(TAG, "Set preferred MTU to %d bytes (max %d commands, payload %d bytes)",
             REQUIRED_MTU, MAX_USABLE_GPIO, PAYLOAD_SIZE);

    // BLE ホストタスク起動
    nimble_port_freertos_init(ble_service_host_task);

    ESP_LOGI(TAG, "BLEIO-ESP32 Service started successfully");
}
