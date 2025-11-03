#include "gpio_adc.h"
#include "main.h"
#include "esp_log.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "BLEIO-ADC";

// GPIO から ADC1 チャネルへのマッピングテーブル
static const struct
{
    uint8_t gpio_num;
    adc_channel_t channel;
} adc1_gpio_map[] = {
    {32, ADC_CHANNEL_4},
    {33, ADC_CHANNEL_5},
    {34, ADC_CHANNEL_6},
    {35, ADC_CHANNEL_7},
    {36, ADC_CHANNEL_0},
    {39, ADC_CHANNEL_3}
};
#define ADC1_GPIO_MAP_SIZE (sizeof(adc1_gpio_map) / sizeof(adc1_gpio_map[0]))

// 減衰率マッピングテーブル
static const adc_atten_t adc_atten_map[] = {
    ADC_ATTEN_DB_0,   // 0: 0 dB (0-1.1V)
    ADC_ATTEN_DB_2_5, // 1: 2.5 dB (0-1.5V)
    ADC_ATTEN_DB_6,   // 2: 6 dB (0-2.2V)
    ADC_ATTEN_DB_12   // 3: 12 dB (0-3.3V、旧 11dB)
};
#define ADC_ATTEN_MAP_SIZE (sizeof(adc_atten_map) / sizeof(adc_atten_map[0]))

// 内部データ
static adc_config_t adc_configs[40] = {0};
static adc_oneshot_unit_handle_t adc1_handle = NULL;

// 内部関数のプロトタイプ
static adc_channel_t gpio_to_adc1_channel(uint8_t gpio_num);

void gpio_adc_init(void)
{
    // ADC1 の初期化
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize ADC1 unit: %s", esp_err_to_name(ret));
        return;
    }

    // ADC 設定の初期化
    for (int i = 0; i < 40; i++)
    {
        adc_configs[i].channel = -1;
        adc_configs[i].attenuation = ADC_ATTEN_DB_12;
        adc_configs[i].calibrated = false;
        adc_configs[i].cali_handle = NULL;
    }

    ESP_LOGI(TAG, "ADC module initialized (12-bit resolution, ADC1 only)");
}

static adc_channel_t gpio_to_adc1_channel(uint8_t gpio_num)
{
    for (int i = 0; i < ADC1_GPIO_MAP_SIZE; i++)
    {
        if (adc1_gpio_map[i].gpio_num == gpio_num)
        {
            return adc1_gpio_map[i].channel;
        }
    }
    return -1; // ADC1 に対応していない
}

esp_err_t gpio_adc_enable(uint8_t pin, uint8_t atten_param)
{
    // パラメータ検証
    adc_channel_t channel = gpio_to_adc1_channel(pin);
    if (channel < 0)
    {
        ESP_LOGE(TAG, "GPIO%d does not support ADC1 (supported pins: 32, 33, 34, 35, 36, 39)", pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (atten_param >= ADC_ATTEN_MAP_SIZE)
    {
        ESP_LOGE(TAG, "Invalid attenuation parameter: %d (max: %d)", atten_param, ADC_ATTEN_MAP_SIZE - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // 既存のキャリブレーションハンドルを削除
    if (adc_configs[pin].cali_handle != NULL)
    {
        adc_cali_delete_scheme_line_fitting(adc_configs[pin].cali_handle);
        adc_configs[pin].cali_handle = NULL;
        adc_configs[pin].calibrated = false;
    }

    // ADC チャネルの設定
    adc_atten_t attenuation = adc_atten_map[atten_param];
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = attenuation,
    };
    esp_err_t ret = adc_oneshot_config_channel(adc1_handle, channel, &config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // キャリブレーションの設定
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = attenuation,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc_configs[pin].cali_handle);
    bool calibrated = (ret == ESP_OK);

    // 設定を保存
    adc_configs[pin].channel = channel;
    adc_configs[pin].attenuation = attenuation;
    adc_configs[pin].calibrated = calibrated;

    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);
    portENTER_CRITICAL(mutex);
    state->mode = BLEIO_MODE_ADC;
    portEXIT_CRITICAL(mutex);

    const char *range_str =
        (atten_param == 0) ? "0-1.1V" : (atten_param == 1) ? "0-1.5V"
                                    : (atten_param == 2)   ? "0-2.2V"
                                                           : "0-3.3V";

    ESP_LOGI(TAG, "Set GPIO%d to ADC mode (channel: %d, atten: %d dB, range: %s, calibration: %s)",
             pin, channel, atten_param, range_str, calibrated ? "success" : "failed");

    return ESP_OK;
}

esp_err_t gpio_adc_disable(uint8_t pin)
{
    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);

    portENTER_CRITICAL(mutex);
    bleio_mode_state_t mode = state->mode;
    portEXIT_CRITICAL(mutex);

    if (mode != BLEIO_MODE_ADC)
    {
        ESP_LOGW(TAG, "GPIO%d is not in ADC mode", pin);
        return ESP_OK;
    }

    // キャリブレーションハンドルを削除
    if (adc_configs[pin].cali_handle != NULL)
    {
        adc_cali_delete_scheme_line_fitting(adc_configs[pin].cali_handle);
        adc_configs[pin].cali_handle = NULL;
    }

    // 設定をクリア
    adc_configs[pin].channel = -1;
    adc_configs[pin].calibrated = false;

    portENTER_CRITICAL(mutex);
    state->mode = BLEIO_MODE_UNSET;
    portEXIT_CRITICAL(mutex);

    ESP_LOGI(TAG, "Disabled ADC mode on GPIO%d", pin);
    return ESP_OK;
}

uint16_t gpio_adc_read_value(uint8_t pin)
{
    adc_config_t *config = &adc_configs[pin];

    if (config->channel < 0)
    {
        ESP_LOGW(TAG, "GPIO%d is not in ADC mode", pin);
        return 0;
    }

    // 生の ADC 値を読み取り
    int raw = 0;
    esp_err_t ret = adc_oneshot_read(adc1_handle, config->channel, &raw);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read ADC on GPIO%d: %s", pin, esp_err_to_name(ret));
        return 0;
    }

    // キャリブレーションが有効な場合は補正された値を使用
    if (config->calibrated && config->cali_handle != NULL)
    {
        int voltage = 0;
        ret = adc_cali_raw_to_voltage(config->cali_handle, raw, &voltage);
        if (ret == ESP_OK)
        {
            // 電圧 (mV) を ADC 値に逆変換 (0-4095)
            // 減衰率に応じた最大電圧で正規化
            uint32_t max_voltage =
                (config->attenuation == ADC_ATTEN_DB_0) ? 1100 : (config->attenuation == ADC_ATTEN_DB_2_5) ? 1500
                                                             : (config->attenuation == ADC_ATTEN_DB_6)     ? 2200
                                                                                                           : 3300;

            uint16_t normalized = (voltage * 4095) / max_voltage;
            return (normalized > 4095) ? 4095 : normalized;
        }
    }

    return (uint16_t)raw;
}

void gpio_adc_stop(uint8_t pin)
{
    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);

    portENTER_CRITICAL(mutex);
    bleio_mode_state_t mode = state->mode;
    portEXIT_CRITICAL(mutex);

    if (mode == BLEIO_MODE_ADC)
    {
        // キャリブレーションハンドルを削除
        if (adc_configs[pin].cali_handle != NULL)
        {
            adc_cali_delete_scheme_line_fitting(adc_configs[pin].cali_handle);
            adc_configs[pin].cali_handle = NULL;
        }

        // 設定をクリア
        adc_configs[pin].channel = -1;
        adc_configs[pin].calibrated = false;

        portENTER_CRITICAL(mutex);
        state->mode = BLEIO_MODE_UNSET;
        portEXIT_CRITICAL(mutex);

        ESP_LOGI(TAG, "Stopped ADC on GPIO%d", pin);
    }
}

adc_config_t* gpio_adc_get_config(uint8_t pin)
{
    return &adc_configs[pin];
}
