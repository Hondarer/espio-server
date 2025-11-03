#include "gpio_basic.h"
#include "gpio_pwm.h"
#include "gpio_adc.h"
#include "ws2812b.h"
#include "main.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "BLEIO-GPIO";

void gpio_basic_init(void)
{
    ESP_LOGI(TAG, "GPIO basic module initialized");
}

esp_err_t gpio_basic_set_mode(uint8_t pin, uint8_t command, uint8_t latch_mode)
{
    if (!main_is_valid_gpio(pin))
    {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    // PWM が有効な場合は停止
    gpio_pwm_stop(pin);

    // ADC が有効な場合は停止
    gpio_adc_stop(pin);

    // WS2812B が有効な場合は停止
    ws2812b_stop(pin);

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);

    switch (command)
    {
    case CMD_SET_INPUT_FLOATING:
        io_conf.mode = GPIO_MODE_INPUT;
        gpio_config(&io_conf);
        portENTER_CRITICAL(mutex);
        state->mode = BLEIO_MODE_INPUT_FLOATING;
        state->latch_mode = latch_mode;
        state->is_latched = false;
        state->stable_counter = 0;
        state->last_level = 0;
        portEXIT_CRITICAL(mutex);
        ESP_LOGI(TAG, "Set GPIO%d to INPUT_FLOATING (latch_mode=%d)", pin, latch_mode);
        break;
    case CMD_SET_INPUT_PULLUP:
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);
        portENTER_CRITICAL(mutex);
        state->mode = BLEIO_MODE_INPUT_PULLUP;
        state->latch_mode = latch_mode;
        state->is_latched = false;
        state->stable_counter = 0;
        state->last_level = 0;
        portEXIT_CRITICAL(mutex);
        ESP_LOGI(TAG, "Set GPIO%d to INPUT_PULLUP (latch_mode=%d)", pin, latch_mode);
        break;
    case CMD_SET_INPUT_PULLDOWN:
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        gpio_config(&io_conf);
        portENTER_CRITICAL(mutex);
        state->mode = BLEIO_MODE_INPUT_PULLDOWN;
        state->latch_mode = latch_mode;
        state->is_latched = false;
        state->stable_counter = 0;
        state->last_level = 0;
        portEXIT_CRITICAL(mutex);
        ESP_LOGI(TAG, "Set GPIO%d to INPUT_PULLDOWN (latch_mode=%d)", pin, latch_mode);
        break;
    default:
        ESP_LOGE(TAG, "Invalid mode command: %d", command);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t gpio_basic_write_level(uint8_t pin, uint8_t command)
{
    if (!main_is_valid_gpio(pin))
    {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    // PWM が有効な場合は停止
    gpio_pwm_stop(pin);

    // ADC が有効な場合は停止
    gpio_adc_stop(pin);

    // WS2812B が有効な場合は停止
    ws2812b_stop(pin);

    // GPIO を出力モードに設定
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_2);

    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);
    uint32_t level;
    bleio_mode_state_t new_mode;

    switch (command)
    {
    case CMD_SET_OUTPUT_LOW:
        level = 0;
        new_mode = BLEIO_MODE_OUTPUT_LOW;
        ESP_LOGI(TAG, "Set GPIO%d to OUTPUT_LOW", pin);
        break;
    case CMD_SET_OUTPUT_HIGH:
        level = 1;
        new_mode = BLEIO_MODE_OUTPUT_HIGH;
        ESP_LOGI(TAG, "Set GPIO%d to OUTPUT_HIGH", pin);
        break;
    default:
        ESP_LOGE(TAG, "Invalid write command: %d", command);
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(mutex);
    state->mode = new_mode;
    portEXIT_CRITICAL(mutex);

    return gpio_set_level(pin, level);
}

esp_err_t gpio_basic_start_blink(uint8_t pin, uint8_t command)
{
    if (!main_is_valid_gpio(pin))
    {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    // PWM が有効な場合は停止
    gpio_pwm_stop(pin);

    // ADC が有効な場合は停止
    gpio_adc_stop(pin);

    // WS2812B が有効な場合は停止
    ws2812b_stop(pin);

    // GPIO を出力モードに設定
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_2);

    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);
    bleio_mode_state_t new_mode;

    switch (command)
    {
    case CMD_SET_OUTPUT_BLINK_250MS:
        new_mode = BLEIO_MODE_BLINK_250MS;
        ESP_LOGI(TAG, "Set GPIO%d to OUTPUT_BLINK_250MS", pin);
        break;
    case CMD_SET_OUTPUT_BLINK_500MS:
        new_mode = BLEIO_MODE_BLINK_500MS;
        ESP_LOGI(TAG, "Set GPIO%d to OUTPUT_BLINK_500MS", pin);
        break;
    default:
        ESP_LOGE(TAG, "Invalid blink command: %d", command);
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(mutex);
    state->mode = new_mode;
    state->current_level = 0;
    portEXIT_CRITICAL(mutex);

    gpio_set_level(pin, 0);
    return ESP_OK;
}

esp_err_t gpio_basic_set_disconnect_behavior(uint8_t pin, uint8_t behavior)
{
    if (!main_is_valid_gpio(pin))
    {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (behavior > 2)
    {
        ESP_LOGE(TAG, "Invalid disconnect behavior: %d (0-2)", behavior);
        return ESP_ERR_INVALID_ARG;
    }

    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);

    portENTER_CRITICAL(mutex);
    state->disconnect_behavior = behavior;
    portEXIT_CRITICAL(mutex);

    const char *behavior_str = (behavior == 0) ? "MAINTAIN" : (behavior == 1) ? "LOW" : "HIGH";
    ESP_LOGI(TAG, "Set disconnect behavior for GPIO%d: %s", pin, behavior_str);

    return ESP_OK;
}

void gpio_basic_update_blink_outputs(uint8_t global_blink_counter)
{
    // 250ms 点滅: カウンタが偶数 (0, 2) で点灯、奇数 (1, 3) で消灯
    bool blink_250ms_level = (global_blink_counter % 2 == 0);

    // 500ms 点滅: カウンタが 0-1 で点灯、2-3 で消灯
    bool blink_500ms_level = (global_blink_counter < 2);

    portMUX_TYPE *mutex = main_get_gpio_states_mutex();

    for (int pin = 0; pin < 40; pin++)
    {
        if (!main_is_valid_gpio(pin))
        {
            continue;
        }

        bleio_gpio_state_t *state = main_get_gpio_state(pin);

        portENTER_CRITICAL(mutex);
        bleio_mode_state_t mode = state->mode;
        portEXIT_CRITICAL(mutex);

        uint8_t new_level = 0;
        bool should_update = false;

        if (mode == BLEIO_MODE_BLINK_250MS)
        {
            // 250ms 周期: グローバルカウンタが偶数で点灯
            new_level = blink_250ms_level ? 1 : 0;
            should_update = true;
        }
        else if (mode == BLEIO_MODE_BLINK_500MS)
        {
            // 500ms 周期: グローバルカウンタが 0-1 で点灯
            new_level = blink_500ms_level ? 1 : 0;
            should_update = true;
        }

        if (should_update)
        {
            portENTER_CRITICAL(mutex);
            state->current_level = new_level;
            portEXIT_CRITICAL(mutex);

            gpio_set_level(pin, new_level);
        }
    }
}

void gpio_basic_update_input_latches(void)
{
    portMUX_TYPE *mutex = main_get_gpio_states_mutex();

    for (int pin = 0; pin < 40; pin++)
    {
        if (!main_is_valid_gpio(pin))
        {
            continue;
        }

        bleio_gpio_state_t *state = main_get_gpio_state(pin);

        portENTER_CRITICAL(mutex);
        uint8_t latch_mode = state->latch_mode;
        bool is_latched = state->is_latched;
        bleio_mode_state_t mode = state->mode;
        uint8_t last_level = state->last_level;
        uint8_t stable_counter = state->stable_counter;
        portEXIT_CRITICAL(mutex);

        // ラッチモードが設定されていない、または既にラッチ済みの場合はスキップ
        if (latch_mode == LATCH_MODE_NONE || is_latched)
        {
            continue;
        }

        // 入力モードかチェック
        if (mode != BLEIO_MODE_INPUT_FLOATING &&
            mode != BLEIO_MODE_INPUT_PULLUP &&
            mode != BLEIO_MODE_INPUT_PULLDOWN)
        {
            continue;
        }

        uint8_t level = gpio_get_level(pin);
        uint8_t target = (latch_mode == LATCH_MODE_HIGH) ? 1 : 0;

        bool new_is_latched = is_latched;
        uint8_t new_stable_counter = stable_counter;

        if (level == target)
        {
            if (last_level == target)
            {
                new_stable_counter++;
                if (new_stable_counter >= LATCH_STABLE_COUNT)
                {
                    new_is_latched = true;
                    ESP_LOGI(TAG, "GPIO%d latched to %s", pin, target ? "HIGH" : "LOW");
                }
            }
            else
            {
                new_stable_counter = 1;
            }
        }
        else
        {
            new_stable_counter = 0;
        }

        portENTER_CRITICAL(mutex);
        state->is_latched = new_is_latched;
        state->stable_counter = new_stable_counter;
        state->last_level = level;
        portEXIT_CRITICAL(mutex);
    }
}
