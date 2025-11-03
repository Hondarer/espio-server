#include "gpio_pwm.h"
#include "gpio_adc.h"
#include "main.h"
#include "esp_log.h"
#include "driver/ledc.h"

static const char *TAG = "BLEIO-PWM";

// 周波数プリセットテーブル (Hz)
static const uint32_t pwm_freq_table[] = {
    1000,  // 0: 1 kHz (デフォルト)
    5000,  // 1: 5 kHz
    10000, // 2: 10 kHz
    25000, // 3: 25 kHz
    50,    // 4: 50 Hz (サーボモーター)
    100,   // 5: 100 Hz
    500,   // 6: 500 Hz
    20000  // 7: 20 kHz
};
#define PWM_FREQ_TABLE_SIZE (sizeof(pwm_freq_table) / sizeof(pwm_freq_table[0]))

// 内部データ
static pwm_config_t pwm_configs[40] = {0};
static ledc_channel_info_t ledc_channels[BLEIO_LEDC_CHANNEL_MAX] = {0};

// 内部関数のプロトタイプ
static int8_t allocate_ledc_channel(uint8_t gpio_num);
static void free_ledc_channel(uint8_t gpio_num);

void gpio_pwm_init(void)
{
    // LEDC タイマー設定
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_RESOLUTION,
        .timer_num = LEDC_TIMER_NUM,
        .freq_hz = pwm_freq_table[0], // デフォルト周波数 (1 kHz)
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return;
    }

    // チャネル管理テーブルの初期化
    for (int i = 0; i < BLEIO_LEDC_CHANNEL_MAX; i++)
    {
        ledc_channels[i].in_use = false;
        ledc_channels[i].gpio_num = 0;
    }

    // PWM 設定の初期化
    for (int i = 0; i < 40; i++)
    {
        pwm_configs[i].duty_cycle = 0;
        pwm_configs[i].freq_preset = 0;
        pwm_configs[i].channel = -1;
    }

    ESP_LOGI(TAG, "PWM initialized (timer=%d, resolution=%d bit, default freq=%lu Hz)",
             LEDC_TIMER_NUM, (1 << LEDC_TIMER_RESOLUTION) - 1, pwm_freq_table[0]);
}

static int8_t allocate_ledc_channel(uint8_t gpio_num)
{
    // 既にこの GPIO に割り当てられているチャネルがあれば再利用
    for (int i = 0; i < BLEIO_LEDC_CHANNEL_MAX; i++)
    {
        if (ledc_channels[i].in_use && ledc_channels[i].gpio_num == gpio_num)
        {
            return (int8_t)i;
        }
    }

    // 空きチャネルを検索
    for (int i = 0; i < BLEIO_LEDC_CHANNEL_MAX; i++)
    {
        if (!ledc_channels[i].in_use)
        {
            ledc_channels[i].in_use = true;
            ledc_channels[i].gpio_num = gpio_num;
            return (int8_t)i;
        }
    }

    // チャネルが不足
    return -1;
}

static void free_ledc_channel(uint8_t gpio_num)
{
    for (int i = 0; i < BLEIO_LEDC_CHANNEL_MAX; i++)
    {
        if (ledc_channels[i].in_use && ledc_channels[i].gpio_num == gpio_num)
        {
            ledc_stop(LEDC_SPEED_MODE, (ledc_channel_t)i, 0);
            ledc_channels[i].in_use = false;
            ledc_channels[i].gpio_num = 0;
            pwm_configs[gpio_num].channel = -1;
            ESP_LOGI(TAG, "Freed LEDC channel %d for GPIO%d", i, gpio_num);
            break;
        }
    }
}

void gpio_pwm_stop(uint8_t pin)
{
    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);

    portENTER_CRITICAL(mutex);
    bleio_mode_state_t mode = state->mode;
    portEXIT_CRITICAL(mutex);

    if (mode == BLEIO_MODE_PWM)
    {
        free_ledc_channel(pin);
        ESP_LOGI(TAG, "Stopped PWM on GPIO%d", pin);
    }
}

esp_err_t gpio_pwm_set(uint8_t pin, uint8_t duty_cycle, uint8_t freq_preset)
{
    // パラメータ検証
    if (!main_is_valid_output_pin(pin))
    {
        ESP_LOGE(TAG, "GPIO%d does not support PWM output", pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (freq_preset >= PWM_FREQ_TABLE_SIZE)
    {
        ESP_LOGE(TAG, "Invalid frequency preset: %d (max: %d)", freq_preset, PWM_FREQ_TABLE_SIZE - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // ADC が有効な場合は停止
    gpio_adc_stop(pin);

    // 既存のモードをクリーンアップ
    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);

    portENTER_CRITICAL(mutex);
    bleio_mode_state_t prev_mode = state->mode;
    portEXIT_CRITICAL(mutex);

    if (prev_mode == BLEIO_MODE_BLINK_250MS || prev_mode == BLEIO_MODE_BLINK_500MS)
    {
        // 点滅モードは特にクリーンアップ不要 (タイマーコールバックで自動スキップされる)
    }

    if (prev_mode == BLEIO_MODE_PWM)
    {
        free_ledc_channel(pin);
    }

    // LEDC チャネルを割り当て
    int8_t channel = allocate_ledc_channel(pin);
    if (channel < 0)
    {
        ESP_LOGE(TAG, "Insufficient LEDC channels (max: %d channels)", BLEIO_LEDC_CHANNEL_MAX);
        return ESP_ERR_NO_MEM;
    }

    // 周波数を設定 (タイマー 0 を使用)
    uint32_t freq_hz = pwm_freq_table[freq_preset];
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_RESOLUTION,
        .timer_num = LEDC_TIMER_NUM,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        ledc_channels[channel].in_use = false;
        ledc_channels[channel].gpio_num = 0;
        return ret;
    }

    // チャネルを設定
    ledc_channel_config_t channel_conf = {
        .gpio_num = pin,
        .speed_mode = LEDC_SPEED_MODE,
        .channel = (ledc_channel_t)channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_NUM,
        .duty = duty_cycle, // 8 ビット値 (0-255)
        .hpoint = 0
    };
    ret = ledc_channel_config(&channel_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(ret));
        ledc_channels[channel].in_use = false;
        ledc_channels[channel].gpio_num = 0;
        return ret;
    }

    // 設定を保存
    pwm_configs[pin].duty_cycle = duty_cycle;
    pwm_configs[pin].freq_preset = freq_preset;
    pwm_configs[pin].channel = channel;

    portENTER_CRITICAL(mutex);
    state->mode = BLEIO_MODE_PWM;
    portEXIT_CRITICAL(mutex);

    ESP_LOGI(TAG, "Set GPIO%d to PWM output (duty: %d/255 = %.1f%%, freq: %lu Hz, channel: %d)",
             pin, duty_cycle, (duty_cycle * 100.0) / 255.0, freq_hz, channel);

    return ESP_OK;
}

pwm_config_t* gpio_pwm_get_config(uint8_t pin)
{
    return &pwm_configs[pin];
}

ledc_channel_info_t* gpio_pwm_get_ledc_channels(void)
{
    return ledc_channels;
}
