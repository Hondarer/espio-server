#include "ws28xx.h"
#include "main.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "BLEIO-WS2812B";

// ガンマ補正テーブル (γ=2.6)
static const uint8_t gamma8[256] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,
    1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,
    2,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   5,   5,   5,
    5,   6,   6,   6,   6,   7,   7,   7,   7,   8,   8,   8,   9,   9,   9,  10,
   10,  10,  11,  11,  11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,
   17,  17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  24,  24,  25,
   25,  26,  27,  27,  28,  29,  29,  30,  31,  32,  32,  33,  34,  35,  35,  36,
   37,  38,  39,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  50,
   51,  52,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  66,  67,  68,
   69,  70,  72,  73,  74,  75,  77,  78,  79,  81,  82,  83,  85,  86,  87,  89,
   90,  92,  93,  95,  96,  98,  99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
  115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
  144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
  177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
  215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255
};

// 内部データ
static ws2812b_config_t ws2812b_configs[40] = {0};

// RMT エンコーダ構造体
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    rmt_symbol_word_t ws2812b_bit0;
    rmt_symbol_word_t ws2812b_bit1;
} ws2812b_encoder_t;

// 内部関数のプロトタイプ
static inline uint32_t gamma32(uint32_t rgb);
static void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, uint16_t *h, uint8_t *s, uint8_t *v);
static uint8_t flicker_rand(uint32_t *state);
static uint32_t ws2812b_color_hsv(uint16_t hue, uint8_t sat, uint8_t val);
static size_t ws2812b_encoder_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                     const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state);
static esp_err_t ws2812b_encoder_reset(rmt_encoder_t *encoder);
static esp_err_t ws2812b_encoder_del(rmt_encoder_t *encoder);
static esp_err_t ws2812b_encoder_new(rmt_encoder_handle_t *ret_encoder);

void ws2812b_init(void)
{
    // WS2812B 設定の初期化
    for (int i = 0; i < 40; i++)
    {
        ws2812b_configs[i].num_leds = 0;
        ws2812b_configs[i].brightness = 0;
        ws2812b_configs[i].led_data = NULL;
        ws2812b_configs[i].rmt_channel = NULL;
        ws2812b_configs[i].rmt_encoder = NULL;
        ws2812b_configs[i].gpio_pattern.pattern_type = WS2812B_PATTERN_ON;
        ws2812b_configs[i].gpio_pattern.pattern_param1 = 0;
        ws2812b_configs[i].gpio_pattern.pattern_param2 = 0;
        ws2812b_configs[i].gpio_pattern.hue = 0;
        ws2812b_configs[i].led_patterns = NULL;
        ws2812b_configs[i].base_colors = NULL;
        ws2812b_configs[i].flicker_data = NULL;
    }

    ESP_LOGI(TAG, "WS2812B module initialized");
}

/**
 * @brief ガンマ補正を適用 (32bit RGB 値)
 *
 * @param rgb RGB 値 (R << 16) | (G << 8) | B
 * @return uint32_t ガンマ補正後の RGB 値
 */
static inline uint32_t gamma32(uint32_t rgb)
{
    uint8_t r = (rgb >> 16) & 0xFF;
    uint8_t g = (rgb >> 8) & 0xFF;
    uint8_t b = rgb & 0xFF;

    r = gamma8[r];
    g = gamma8[g];
    b = gamma8[b];

    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

/**
 * @brief RGB 色空間から HSV 色空間への変換
 *
 * @param r 赤 (0-255)
 * @param g 緑 (0-255)
 * @param b 青 (0-255)
 * @param h 色相 (0-65535) への出力ポインタ
 * @param s 彩度 (0-255) への出力ポインタ
 * @param v 明度 (0-255) への出力ポインタ
 */
static void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b,
                       uint16_t *h, uint8_t *s, uint8_t *v)
{
    uint8_t max = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    uint8_t min = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
    uint8_t delta = max - min;

    *v = max;

    if (max == 0)
    {
        *s = 0;
        *h = 0;
        return;
    }

    *s = (uint32_t)delta * 255 / max;

    if (delta == 0)
    {
        *h = 0;
        return;
    }

    int32_t hue;
    if (max == r)
    {
        hue = ((int32_t)(g - b) * 65536) / (6 * delta);
        if (hue < 0)
            hue += 65536;
    }
    else if (max == g)
    {
        hue = (65536 / 3) + ((int32_t)(b - r) * 65536) / (6 * delta);
    }
    else
    {
        hue = (65536 * 2 / 3) + ((int32_t)(r - g) * 65536) / (6 * delta);
    }

    *h = (uint16_t)(hue & 0xFFFF);
}

/**
 * @brief 疑似乱数生成 (線形合同法)
 *
 * @param state 疑似乱数の状態 (入出力)
 * @return uint8_t 疑似乱数 (0-255)
 */
static uint8_t flicker_rand(uint32_t *state)
{
    // 線形合同法: seed = (a * seed + c) mod m
    // a=1103515245, c=12345, m=2^31
    *state = (*state * 1103515245 + 12345) & 0x7FFFFFFF;
    return (*state >> 16) & 0xFF;
}

/**
 * @brief HSV 色空間から RGB 色空間への変換
 *
 * @param hue 色相 (0-65535)
 * @param sat 彩度 (0-255)
 * @param val 明度 (0-255)
 * @return uint32_t RGB 値 (R << 16) | (G << 8) | B
 */
static uint32_t ws2812b_color_hsv(uint16_t hue, uint8_t sat, uint8_t val)
{
    // セクタント計算 (色相環を 6 分割)
    // hue * 6 を計算して、上位ビットからセクタント番号と位置を取得
    uint32_t hue_scaled = (uint32_t)hue * 6;
    uint8_t sextant = hue_scaled >> 16;      // 整数部分 (0-6)
    uint8_t h_fraction = (hue_scaled >> 8) & 0xFF;  // 小数部分の上位 8 ビット (0-255)

    if (sextant > 5)
    {
        sextant = 5;
        h_fraction = 255;  // 端数処理: 最大値にクリップ
    }

    // 彩度 0 の場合はグレースケール
    if (sat == 0)
    {
        return (val << 16) | (val << 8) | val;
    }

    // 中間値計算: bottom (最小輝度)
    uint16_t invsat = 255 - sat;
    uint16_t ww = val * invsat;
    ww += 1;
    ww += ww >> 8;
    uint8_t bottom = ww >> 8;

    uint8_t top = val;

    // スケール値計算
    ww = val * sat;
    ww += 1;
    ww += ww >> 8;
    uint8_t scale_val = ww >> 8;

    // rising (上昇する中間値)
    ww = scale_val * h_fraction;
    ww += 1;
    ww += ww >> 8;
    uint8_t rising = (ww >> 8) + bottom;

    // falling (下降する中間値)
    uint8_t inv_h_fraction = 255 - h_fraction;
    ww = scale_val * inv_h_fraction;
    ww += 1;
    ww += ww >> 8;
    uint8_t falling = (ww >> 8) + bottom;

    // セクタントに応じて RGB を割り当て
    uint8_t r, g, b;
    switch (sextant)
    {
    case 0:
        r = top;
        g = rising;
        b = bottom;
        break; // 赤 → 黄
    case 1:
        r = falling;
        g = top;
        b = bottom;
        break; // 黄 → 緑
    case 2:
        r = bottom;
        g = top;
        b = rising;
        break; // 緑 → シアン
    case 3:
        r = bottom;
        g = falling;
        b = top;
        break; // シアン → 青
    case 4:
        r = rising;
        g = bottom;
        b = top;
        break; // 青 → マゼンタ
    default:
        r = top;
        g = bottom;
        b = falling;
        break; // マゼンタ → 赤
    }

    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

// RMT エンコーダ実装
static size_t ws2812b_encoder_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                     const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    ws2812b_encoder_t *ws2812b_encoder = __containerof(encoder, ws2812b_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = ws2812b_encoder->bytes_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    // エンコード処理
    encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);

    *ret_state = session_state;
    return encoded_symbols;
}

static esp_err_t ws2812b_encoder_reset(rmt_encoder_t *encoder)
{
    ws2812b_encoder_t *ws2812b_encoder = __containerof(encoder, ws2812b_encoder_t, base);
    rmt_encoder_reset(ws2812b_encoder->bytes_encoder);
    rmt_encoder_reset(ws2812b_encoder->copy_encoder);
    return ESP_OK;
}

static esp_err_t ws2812b_encoder_del(rmt_encoder_t *encoder)
{
    ws2812b_encoder_t *ws2812b_encoder = __containerof(encoder, ws2812b_encoder_t, base);
    rmt_del_encoder(ws2812b_encoder->bytes_encoder);
    rmt_del_encoder(ws2812b_encoder->copy_encoder);
    free(ws2812b_encoder);
    return ESP_OK;
}

static esp_err_t ws2812b_encoder_new(rmt_encoder_handle_t *ret_encoder)
{
    ws2812b_encoder_t *ws2812b_encoder = calloc(1, sizeof(ws2812b_encoder_t));
    if (ws2812b_encoder == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    ws2812b_encoder->base.encode = ws2812b_encoder_encode;
    ws2812b_encoder->base.reset = ws2812b_encoder_reset;
    ws2812b_encoder->base.del = ws2812b_encoder_del;

    // 0 ビットと 1 ビットのシンボル定義
    ws2812b_encoder->ws2812b_bit0.level0 = 1;
    ws2812b_encoder->ws2812b_bit0.duration0 = WS2812B_T0H_TICKS;
    ws2812b_encoder->ws2812b_bit0.level1 = 0;
    ws2812b_encoder->ws2812b_bit0.duration1 = WS2812B_T0L_TICKS;

    ws2812b_encoder->ws2812b_bit1.level0 = 1;
    ws2812b_encoder->ws2812b_bit1.duration0 = WS2812B_T1H_TICKS;
    ws2812b_encoder->ws2812b_bit1.level1 = 0;
    ws2812b_encoder->ws2812b_bit1.duration1 = WS2812B_T1L_TICKS;

    // バイトエンコーダの作成
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = ws2812b_encoder->ws2812b_bit0,
        .bit1 = ws2812b_encoder->ws2812b_bit1,
        .flags.msb_first = 1
    };
    esp_err_t ret = rmt_new_bytes_encoder(&bytes_encoder_config, &ws2812b_encoder->bytes_encoder);
    if (ret != ESP_OK)
    {
        free(ws2812b_encoder);
        return ret;
    }

    // コピーエンコーダの作成
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &ws2812b_encoder->copy_encoder);
    if (ret != ESP_OK)
    {
        rmt_del_encoder(ws2812b_encoder->bytes_encoder);
        free(ws2812b_encoder);
        return ret;
    }

    *ret_encoder = &ws2812b_encoder->base;
    return ESP_OK;
}

void ws2812b_update_patterns(uint8_t blink_counter)
{
    // BLINK 系パターンの更新が必要か判定 (blink_counter が変化したとき)
    static uint8_t prev_blink_counter = 0xFF;

    bool blink_changed = (blink_counter != prev_blink_counter);
    if (blink_changed)
    {
        prev_blink_counter = blink_counter;
    }

    // デジタル出力の点滅状態を取得
    bool blink_250ms_level = (blink_counter % 2 == 0);
    bool blink_500ms_level = (blink_counter < 2);

    for (int pin = 0; pin < 40; pin++)
    {
        if (!main_is_valid_gpio(pin))
            continue;

        portMUX_TYPE *mutex = main_get_gpio_states_mutex();
        bleio_gpio_state_t *state = main_get_gpio_state(pin);

        portENTER_CRITICAL_ISR(mutex);
        bleio_mode_state_t mode = state->mode;
        portEXIT_CRITICAL_ISR(mutex);

        if (mode != BLEIO_MODE_WS2812B)
            continue;

        ws2812b_config_t *config = &ws2812b_configs[pin];
        if (config->num_leds == 0 || config->led_data == NULL)
            continue;

        bool need_update = false;
        bool has_rainbow = false;

        // LED ごとにパターンを適用
        for (uint16_t led_idx = 0; led_idx < config->num_leds; led_idx++)
        {
            // パターン設定を取得 (個別設定がなければ GPIO パターン)
            ws2812b_led_pattern_t *pattern = &config->gpio_pattern;
            if (config->led_patterns != NULL &&
                config->led_patterns[led_idx].pattern_type != WS2812B_PATTERN_UNSET)
            {
                pattern = &config->led_patterns[led_idx];
            }

            uint8_t r = 0, g = 0, b = 0;
            bool update_this_led = false;

            switch (pattern->pattern_type)
            {
            case WS2812B_PATTERN_ON:
                // ベースカラーに基準輝度を適用 (初回のみ更新)
                if (config->base_colors != NULL)
                {
                    r = (config->base_colors[led_idx * 3] * config->brightness) / 255;
                    g = (config->base_colors[led_idx * 3 + 1] * config->brightness) / 255;
                    b = (config->base_colors[led_idx * 3 + 2] * config->brightness) / 255;
                }
                // PATTERN_ON は変化しないので、初回設定以降は更新不要
                break;

            case WS2812B_PATTERN_BLINK_250MS:
                // BLINK 系は global_blink_counter が変化したときのみ更新
                if (blink_changed)
                {
                    if (blink_250ms_level && config->base_colors != NULL)
                    {
                        r = (config->base_colors[led_idx * 3] * config->brightness) / 255;
                        g = (config->base_colors[led_idx * 3 + 1] * config->brightness) / 255;
                        b = (config->base_colors[led_idx * 3 + 2] * config->brightness) / 255;
                    }
                    update_this_led = true;
                }
                break;

            case WS2812B_PATTERN_BLINK_500MS:
                // BLINK 系は global_blink_counter が変化したときのみ更新
                if (blink_changed)
                {
                    if (blink_500ms_level && config->base_colors != NULL)
                    {
                        r = (config->base_colors[led_idx * 3] * config->brightness) / 255;
                        g = (config->base_colors[led_idx * 3 + 1] * config->brightness) / 255;
                        b = (config->base_colors[led_idx * 3 + 2] * config->brightness) / 255;
                    }
                    update_this_led = true;
                }
                break;

            case WS2812B_PATTERN_RAINBOW:
            {
                // RAINBOW パターンは毎回更新
                has_rainbow = true;
                update_this_led = true;

                // 色相が一周する LED 個数 (1-16)
                uint8_t hue_period = pattern->pattern_param1;
                if (hue_period == 0)
                    hue_period = 12; // デフォルト

                // 基準クロックは常に GPIO 全体の hue を使用 (すべての LED が同じクロックを共有)
                uint16_t base_hue = config->gpio_pattern.hue;

                // 色相オフセットの計算 (LED 番号によるオフセット)
                // LED0 (ESP32 に近い) が最も進んだ色相、LED9 (遠い) が遅れた色相
                // これにより、色が ESP32 から遠い方に流れるように見える
                uint16_t hue_offset = ((uint32_t)led_idx * 65536) / hue_period;
                uint16_t current_hue = base_hue - hue_offset;

                // HSV → RGB 変換 (彩度 255、明度は基準輝度)
                uint32_t rgb = ws2812b_color_hsv(current_hue, 255, config->brightness);

                // ガンマ補正
                rgb = gamma32(rgb);

                r = (rgb >> 16) & 0xFF;
                g = (rgb >> 8) & 0xFF;
                b = rgb & 0xFF;
                break;
            }

            case WS2812B_PATTERN_FLICKER:
            {
                // FLICKER パターンは毎回更新
                update_this_led = true;

                // 基準色を取得
                uint8_t base_r = 0, base_g = 0, base_b = 0;
                if (config->base_colors != NULL)
                {
                    base_r = config->base_colors[led_idx * 3];
                    base_g = config->base_colors[led_idx * 3 + 1];
                    base_b = config->base_colors[led_idx * 3 + 2];
                }

                // flicker_data が未確保の場合は確保
                if (config->flicker_data == NULL)
                {
                    config->flicker_data = (led_flicker_data_t *)calloc(config->num_leds, sizeof(led_flicker_data_t));
                    if (config->flicker_data == NULL)
                    {
                        ESP_LOGE(TAG, "Failed to allocate flicker_data");
                        break;
                    }

                    // 各 LED の flicker_data を初期化
                    for (uint16_t i = 0; i < config->num_leds; i++)
                    {
                        // 疑似乱数のシードを初期化 (LED インデックスとシステム時刻から)
                        config->flicker_data[i].seed = (i * 12345 + esp_timer_get_time() / 1000) & 0x7FFFFFFF;

                        // 基準色の HSV を計算してキャッシュ
                        uint8_t br = config->base_colors ? config->base_colors[i * 3] : 0;
                        uint8_t bg = config->base_colors ? config->base_colors[i * 3 + 1] : 0;
                        uint8_t bb = config->base_colors ? config->base_colors[i * 3 + 2] : 0;
                        rgb_to_hsv(br, bg, bb,
                                   &config->flicker_data[i].base_hue,
                                   &config->flicker_data[i].base_sat,
                                   &config->flicker_data[i].base_val);

                        // 間欠カオス法用の初期化（ランダム化して開始時からリズム感を出す）
                        // x: 0.3-0.7 のランダムな値（参考\main.c では 0.5 だが、開始時の変化を出すため）
                        uint32_t seed = config->flicker_data[i].seed;
                        uint8_t rand_x = flicker_rand(&seed);
                        config->flicker_data[i].x = 0.3f + (float)rand_x / 255.0f * 0.4f;  // 0.3-0.7

                        // maxloop: 参考\main.c と同じく 3-11 のランダムな値
                        config->flicker_data[i].maxloop = (flicker_rand(&seed) % 9) + 3;

                        // count: 0-maxloop のランダムな値（すぐに x 更新が起きるようにする）
                        config->flicker_data[i].count = flicker_rand(&seed) % (config->flicker_data[i].maxloop + 1);

                        // 色相用の初期化（明度とは独立したランダム値）
                        uint8_t rand_hue = flicker_rand(&seed);
                        config->flicker_data[i].hue_offset_target = -0.5f + (float)rand_hue / 255.0f;  // -0.5 - +0.5
                        config->flicker_data[i].hue_offset = config->flicker_data[i].hue_offset_target;

                        // seed を更新
                        config->flicker_data[i].seed = seed;

                        // フィールドの初期値
                        // y の初期値は x に対応する値（ランダム化された x から計算）
                        // val_ema = (x * 0.375 + 0.625) * base_val
                        float y_init = (config->flicker_data[i].x * 0.375f + 0.625f) * (float)config->flicker_data[i].base_val;
                        config->flicker_data[i].val_ema = (uint8_t)y_init;
                        config->flicker_data[i].hue_ema = config->flicker_data[i].base_hue;
                    }
                }

                // 基準色が変更された可能性があるので、RGB → HSV 変換を実行してキャッシュを更新
                rgb_to_hsv(base_r, base_g, base_b,
                           &config->flicker_data[led_idx].base_hue,
                           &config->flicker_data[led_idx].base_sat,
                           &config->flicker_data[led_idx].base_val);

                // パラメータ取得
                uint8_t speed = pattern->pattern_param1;   // 1..255 (0 のときはデフォルト)
                if (speed == 0) speed = 128;
                uint8_t range = pattern->pattern_param2;   // 1..255 (0 のときはデフォルト)
                if (range == 0) range = 128;

                led_flicker_data_t *fd = &config->flicker_data[led_idx];

                // base_val=0 (黒) の場合、消灯を維持
                if (fd->base_val == 0)
                {
                    r = 0;
                    g = 0;
                    b = 0;
                    update_this_led = true;
                    break;
                }

                // 間欠カオス法による 1/f ゆらぎ (参考\main.c のアルゴリズム)
                if (fd->count == 0)
                {
                    // 間欠カオス法: 状態変数 x を更新（明度用）
                    if (fd->x < 0.5f)
                    {
                        fd->x = fd->x + 2.0f * fd->x * fd->x;
                    }
                    else
                    {
                        fd->x = fd->x - 2.0f * (1.0f - fd->x) * (1.0f - fd->x);
                    }

                    // 最小値・最大値への張り付きを防ぐため、乱数で離す
                    if (fd->x < 0.005f)
                    {
                        fd->x += (float)(flicker_rand(&fd->seed) % 1000) / 10000.0f;
                    }
                    else if (fd->x > 0.995f)
                    {
                        fd->x -= (float)(flicker_rand(&fd->seed) % 1000) / 10000.0f;
                    }

#if WS2812B_FLICKER_HUE_VARIATION > 0
                    // 色相用の目標値を更新（明度とは独立したランダムウォーク）
                    int16_t hue_delta = ((int16_t)flicker_rand(&fd->seed) - 128) / 2;  // -64 - +63
                    fd->hue_offset_target += (float)hue_delta / 256.0f;

                    // -0.5 - +0.5 の範囲にクリップ
                    if (fd->hue_offset_target < -0.5f) fd->hue_offset_target = -0.5f;
                    if (fd->hue_offset_target > 0.5f) fd->hue_offset_target = 0.5f;
#endif
                }

                // 光の増減がスムーズになるよう徐々に明るさを切り替える
                // PIC版との互換性: y と x は両方 0-1 の範囲で管理
                // y = y * (1 - 1/maxloop) + x * 1/maxloop

                // val_ema から y_normalized に逆変換
                // val_ema = (y_normalized * 0.375 + 0.625) * base_val
                // → y_normalized = (val_ema / base_val - 0.625) / 0.375
                float y_normalized = ((float)fd->val_ema / (float)fd->base_val - 0.625f) / 0.375f;

                float maxloop_f = (float)fd->maxloop;
                y_normalized = y_normalized * (1.0f - 1.0f / maxloop_f) + fd->x * (1.0f / maxloop_f);

                // range パラメータで変化幅を調整 (0-1 の範囲で)
                // range=128 のとき range_factor=1.0 で PIC 版と同じ動作
                float range_factor = (float)range / 128.0f;
                y_normalized = 0.5f + (y_normalized - 0.5f) * range_factor;

                // 0-1 にクリップ
                if (y_normalized < 0.0f) y_normalized = 0.0f;
                if (y_normalized > 1.0f) y_normalized = 1.0f;

                // PIC版と同じスケーリング: CCPR1L = y * 96.0 + 160.0
                // PWM 範囲 62.5%-100% を base_val に適用
                // 出力 = (y * 96/256 + 160/256) * base_val = (y * 0.375 + 0.625) * base_val
                float val_out = (y_normalized * 0.375f + 0.625f) * (float)fd->base_val;
                if (val_out > 255.0f) val_out = 255.0f;

                fd->val_ema = (uint8_t)val_out;

                // カウンタを更新
                fd->count++;
                if (fd->count > fd->maxloop)
                {
                    fd->count = 0;

                    // 移行スピードを乱数で調整 (speed パラメータで調整)
                    // 参考\main.c との互換性: 20ms × (3-11) = 10ms × (6-22)
                    // speed が低いほど maxloop が大きくなり、変化が遅くなる
                    // speed=128 (デフォルト) で参考\main.c と同じ速度 (平均 140ms)
                    uint8_t base_maxloop = 14 - (speed / 32);  // 6..14 程度の範囲
                    if (base_maxloop < 3) base_maxloop = 3;
                    fd->maxloop = (flicker_rand(&fd->seed) % 9) + base_maxloop;
                }

                // 色相のゆらぎ (WS2812B_FLICKER_HUE_VARIATION で制御)
#if WS2812B_FLICKER_HUE_VARIATION > 0
                // 色相オフセットを目標値に向かって平滑化（明度とは独立）
                float alpha_hue = 0.30f;  // 色相の平滑化係数（明度と同程度の速度）
                fd->hue_offset += alpha_hue * (fd->hue_offset_target - fd->hue_offset);

                // range パラメータと係数を適用
                int16_t hue_offset_scaled = (int16_t)(fd->hue_offset * (float)range * (float)WS2812B_FLICKER_HUE_VARIATION);
                int32_t hue_val = (int32_t)fd->base_hue + hue_offset_scaled;
                if (hue_val < 0) hue_val += 65536;
                if (hue_val > 65535) hue_val -= 65536;
                fd->hue_ema = (uint16_t)hue_val;
#else
                // 色相ゆらぎ無効時は基準値を維持
                fd->hue_ema = fd->base_hue;
#endif

                // 彩度は基準値を維持
                uint8_t s_now = fd->base_sat;

                // HSV → RGB (全体輝度を反映) → ガンマ補正
                uint8_t v_final = (uint8_t)(((uint32_t)fd->val_ema * config->brightness) / 255);
                uint32_t rgb = ws2812b_color_hsv(fd->hue_ema, s_now, v_final);
                rgb = gamma32(rgb);

                // 出力 RGB を取り出す
                r = (rgb >> 16) & 0xFF;
                g = (rgb >>  8) & 0xFF;
                b = rgb & 0xFF;

                break;
            }

            default:
                // 未設定または不明なパターン
                break;
            }

            if (update_this_led)
            {
                // カラーオーダーに応じて LED データバッファに書き込み
                switch (config->led_type)
                {
                case LED_TYPE_WS2812B_RGB:
                case LED_TYPE_WS2811:
                    // RGB 順
                    config->led_data[led_idx * 3] = r;
                    config->led_data[led_idx * 3 + 1] = g;
                    config->led_data[led_idx * 3 + 2] = b;
                    break;

                case LED_TYPE_WS2812B_GRB:
                    // GRB 順
                    config->led_data[led_idx * 3] = g;
                    config->led_data[led_idx * 3 + 1] = r;
                    config->led_data[led_idx * 3 + 2] = b;
                    break;

                default:
                    // デフォルトは RGB 順
                    config->led_data[led_idx * 3] = r;
                    config->led_data[led_idx * 3 + 1] = g;
                    config->led_data[led_idx * 3 + 2] = b;
                    break;
                }
                need_update = true;
            }
        }

        // 色相の更新 (RAINBOW パターン用)
        // すべての RAINBOW LED は同じ基準クロック (gpio_pattern.hue) を共有
        if (has_rainbow)
        {
            uint16_t speed = 128; // デフォルトスピード

            // スピードの決定: GPIO パターンが RAINBOW なら、そのスピードを使用
            if (config->gpio_pattern.pattern_type == WS2812B_PATTERN_RAINBOW)
            {
                speed = config->gpio_pattern.pattern_param2;
                if (speed == 0)
                    speed = 128;
            }
            else
            {
                // GPIO パターンが RAINBOW でない場合、最初の RAINBOW LED のスピードを使用
                if (config->led_patterns != NULL)
                {
                    for (uint16_t led_idx = 0; led_idx < config->num_leds; led_idx++)
                    {
                        if (config->led_patterns[led_idx].pattern_type == WS2812B_PATTERN_RAINBOW)
                        {
                            speed = config->led_patterns[led_idx].pattern_param2;
                            if (speed == 0)
                                speed = 128;
                            break;
                        }
                    }
                }
            }

            // 基準クロック (gpio_pattern.hue) を更新
            uint16_t hue_increment = speed * 8; // チューニング係数
            config->gpio_pattern.hue += hue_increment;
        }

        // RMT で LED に送信
        if (need_update)
        {
            rmt_transmit_config_t tx_config = {
                .loop_count = 0,
            };

            rmt_transmit(config->rmt_channel, config->rmt_encoder,
                         config->led_data, config->num_leds * 3, &tx_config);
        }
    }
}

esp_err_t ws2812b_turn_off_all_leds(uint8_t pin)
{
    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);

    portENTER_CRITICAL(mutex);
    bleio_mode_state_t mode = state->mode;
    portEXIT_CRITICAL(mutex);

    if (mode != BLEIO_MODE_WS2812B)
    {
        ESP_LOGW(TAG, "GPIO%d is not in WS2812B mode", pin);
        return ESP_ERR_INVALID_STATE;
    }

    ws2812b_config_t *config = &ws2812b_configs[pin];

    if (config->led_data == NULL || config->rmt_channel == NULL || config->rmt_encoder == NULL)
    {
        ESP_LOGE(TAG, "WS2812B configuration for GPIO%d is invalid", pin);
        return ESP_ERR_INVALID_STATE;
    }

    // LED データバッファをすべて 0 にクリア (すべて消灯)
    memset(config->led_data, 0, config->num_leds * 3);

    // 消灯データを送信
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    esp_err_t ret = rmt_transmit(config->rmt_channel, config->rmt_encoder,
                                  config->led_data, config->num_leds * 3, &tx_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send WS2812B turn-off data on GPIO%d: %s", pin, esp_err_to_name(ret));
        return ret;
    }

    // 送信完了を待つ
    ret = rmt_tx_wait_all_done(config->rmt_channel, 100);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "WS2812B turn-off data transmission wait timeout on GPIO%d", pin);
    }

    ESP_LOGI(TAG, "Turned off all WS2812B LEDs on GPIO%d (mode maintained)", pin);
    return ESP_OK;
}

void ws2812b_stop(uint8_t pin)
{
    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);

    portENTER_CRITICAL(mutex);
    bleio_mode_state_t mode = state->mode;
    portEXIT_CRITICAL(mutex);

    if (mode == BLEIO_MODE_WS2812B)
    {
        ws2812b_config_t *config = &ws2812b_configs[pin];

        // WS2812B モードを無効化する前に、すべての LED を消灯する
        if (config->led_data != NULL && config->rmt_channel != NULL && config->rmt_encoder != NULL)
        {
            // LED データバッファをすべて 0 にクリア (すべて消灯)
            memset(config->led_data, 0, config->num_leds * 3);

            // 消灯データを送信
            rmt_transmit_config_t tx_config = {
                .loop_count = 0,
            };

            esp_err_t ret = rmt_transmit(config->rmt_channel, config->rmt_encoder,
                                          config->led_data, config->num_leds * 3, &tx_config);
            if (ret == ESP_OK)
            {
                // 送信完了を待つ
                rmt_tx_wait_all_done(config->rmt_channel, 100);
                ESP_LOGI(TAG, "Turned off all WS2812B LEDs on GPIO%d", pin);
            }
            else
            {
                ESP_LOGW(TAG, "Failed to send WS2812B turn-off data on GPIO%d: %s", pin, esp_err_to_name(ret));
            }
        }

        // RMT チャネルとエンコーダを削除
        if (config->rmt_channel != NULL)
        {
            rmt_disable(config->rmt_channel);
            rmt_del_channel(config->rmt_channel);
            config->rmt_channel = NULL;
        }

        if (config->rmt_encoder != NULL)
        {
            rmt_del_encoder(config->rmt_encoder);
            config->rmt_encoder = NULL;
        }

        // RMT が GPIO の制御を解放した後、GPIO を LOW に設定して WS2812B の点灯を確実に消す
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 0);

        // LED データバッファを解放
        if (config->led_data != NULL)
        {
            free(config->led_data);
            config->led_data = NULL;
        }

        // base_colors を解放
        if (config->base_colors != NULL)
        {
            free(config->base_colors);
            config->base_colors = NULL;
        }

        // LED 個別パターンを解放
        if (config->led_patterns != NULL)
        {
            free(config->led_patterns);
            config->led_patterns = NULL;
        }

        // flicker_data を解放
        if (config->flicker_data != NULL)
        {
            free(config->flicker_data);
            config->flicker_data = NULL;
        }

        config->num_leds = 0;
        config->brightness = 0;

        // パターン設定をクリア
        config->gpio_pattern.pattern_type = WS2812B_PATTERN_ON;
        config->gpio_pattern.pattern_param1 = 0;
        config->gpio_pattern.pattern_param2 = 0;
        config->gpio_pattern.hue = 0;

        portENTER_CRITICAL(mutex);
        state->mode = BLEIO_MODE_UNSET;
        portEXIT_CRITICAL(mutex);

        ESP_LOGI(TAG, "Stopped WS2812B on GPIO%d", pin);
    }
}

esp_err_t ws2812b_enable(uint8_t pin, uint16_t num_leds, uint8_t brightness, serial_led_type_t led_type)
{
    // パラメータ検証
    if (!main_is_valid_output_pin(pin))
    {
        ESP_LOGE(TAG, "GPIO%d does not support WS2812B output", pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (num_leds == 0 || num_leds > WS2812B_MAX_LEDS)
    {
        ESP_LOGE(TAG, "Invalid LED count: %d (range: 1-%d)", num_leds, WS2812B_MAX_LEDS);
        return ESP_ERR_INVALID_ARG;
    }

    // LED 種別の検証
    if (led_type > LED_TYPE_WS2811)
    {
        ESP_LOGW(TAG, "Invalid LED type: %d, using default (WS2812B_RGB)", led_type);
        led_type = LED_TYPE_WS2812B_RGB;
    }

    // 既存のモードをクリーンアップ
    ws2812b_stop(pin);

    // LED データバッファを確保 (RGB 形式、3 バイト × LED 個数)
    uint8_t *led_data = (uint8_t *)calloc(num_leds * 3, sizeof(uint8_t));
    if (led_data == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate LED data buffer");
        return ESP_ERR_NO_MEM;
    }

    // RMT チャネルの設定
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = pin,
        .mem_block_symbols = 64,
        .resolution_hz = WS2812B_RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };

    rmt_channel_handle_t rmt_channel = NULL;
    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &rmt_channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create RMT channel: %s", esp_err_to_name(ret));
        free(led_data);
        return ret;
    }

    // エンコーダの作成
    rmt_encoder_handle_t rmt_encoder = NULL;
    ret = ws2812b_encoder_new(&rmt_encoder);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create WS2812B encoder: %s", esp_err_to_name(ret));
        rmt_del_channel(rmt_channel);
        free(led_data);
        return ret;
    }

    // RMT チャネルを有効化
    ret = rmt_enable(rmt_channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        rmt_del_encoder(rmt_encoder);
        rmt_del_channel(rmt_channel);
        free(led_data);
        return ret;
    }

    // base_colors 配列を割り当て (RGB 形式、3 バイト × num_leds)
    uint8_t *base_colors = (uint8_t *)calloc(num_leds * 3, sizeof(uint8_t));
    if (base_colors == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate base_colors");
        rmt_disable(rmt_channel);
        rmt_del_encoder(rmt_encoder);
        rmt_del_channel(rmt_channel);
        free(led_data);
        return ESP_ERR_NO_MEM;
    }

    // 設定を保存
    ws2812b_configs[pin].num_leds = num_leds;
    ws2812b_configs[pin].brightness = brightness;
    ws2812b_configs[pin].led_type = led_type;
    ws2812b_configs[pin].led_data = led_data;
    ws2812b_configs[pin].rmt_channel = rmt_channel;
    ws2812b_configs[pin].rmt_encoder = rmt_encoder;
    ws2812b_configs[pin].base_colors = base_colors;

    // GPIO パターンの初期化 (デフォルトは PATTERN_ON)
    ws2812b_configs[pin].gpio_pattern.pattern_type = WS2812B_PATTERN_ON;
    ws2812b_configs[pin].gpio_pattern.pattern_param1 = 0;
    ws2812b_configs[pin].gpio_pattern.pattern_param2 = 0;
    ws2812b_configs[pin].gpio_pattern.hue = 0;

    // LED 個別パターンは NULL で初期化 (必要に応じて動的割り当て)
    ws2812b_configs[pin].led_patterns = NULL;

    // flicker_data は NULL で初期化 (必要に応じて動的割り当て)
    ws2812b_configs[pin].flicker_data = NULL;

    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);

    portENTER_CRITICAL(mutex);
    state->mode = BLEIO_MODE_WS2812B;
    portEXIT_CRITICAL(mutex);

    // 初期状態としてすべての LED を消灯 (RGB = 0, 0, 0) にする
    // led_data は calloc で 0 初期化されているので、そのまま送信
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    ret = rmt_transmit(rmt_channel, rmt_encoder, led_data, num_leds * 3, &tx_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send WS2812B initialization data: %s", esp_err_to_name(ret));
        // エラーでも継続 (次回のコマンドで送信できる可能性がある)
    }
    else
    {
        // 送信完了を待つ
        ret = rmt_tx_wait_all_done(rmt_channel, 100);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "WS2812B initialization data transmission wait timeout");
        }
    }

    ESP_LOGI(TAG, "Set GPIO%d to WS2812B output (LED count: %d, brightness: %d/255 = %.1f%%, initial state: all off)",
             pin, num_leds, brightness, (brightness * 100.0) / 255.0);

    return ESP_OK;
}

esp_err_t ws2812b_set_color(uint8_t pin, uint16_t led_index, uint8_t r, uint8_t g, uint8_t b)
{
    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);

    portENTER_CRITICAL(mutex);
    bleio_mode_state_t mode = state->mode;
    portEXIT_CRITICAL(mutex);

    if (mode != BLEIO_MODE_WS2812B)
    {
        ESP_LOGE(TAG, "GPIO%d is not in WS2812B mode", pin);
        return ESP_ERR_INVALID_STATE;
    }

    ws2812b_config_t *config = &ws2812b_configs[pin];

    // LED 番号は 1 から始まる (1-indexed)
    if (led_index < 1 || led_index > config->num_leds)
    {
        ESP_LOGE(TAG, "Invalid LED index: %d (valid range: 1-%d)", led_index, config->num_leds);
        return ESP_ERR_INVALID_ARG;
    }

    // base_colors に保存 (パターンで使用するため)
    if (config->base_colors != NULL)
    {
        uint32_t base_offset = (led_index - 1) * 3;
        config->base_colors[base_offset + 0] = r;
        config->base_colors[base_offset + 1] = g;
        config->base_colors[base_offset + 2] = b;
    }

    // 輝度を適用
    uint32_t r_scaled = (r * config->brightness) / 255;
    uint32_t g_scaled = (g * config->brightness) / 255;
    uint32_t b_scaled = (b * config->brightness) / 255;

    // カラーオーダーに応じて保存 (配列アクセスは 0-indexed なので -1)
    uint32_t offset = (led_index - 1) * 3;
    switch (config->led_type)
    {
    case LED_TYPE_WS2812B_RGB:
    case LED_TYPE_WS2811:
        // RGB 順
        config->led_data[offset + 0] = (uint8_t)r_scaled;
        config->led_data[offset + 1] = (uint8_t)g_scaled;
        config->led_data[offset + 2] = (uint8_t)b_scaled;
        break;

    case LED_TYPE_WS2812B_GRB:
        // GRB 順
        config->led_data[offset + 0] = (uint8_t)g_scaled;
        config->led_data[offset + 1] = (uint8_t)r_scaled;
        config->led_data[offset + 2] = (uint8_t)b_scaled;
        break;

    default:
        // デフォルトは RGB 順
        config->led_data[offset + 0] = (uint8_t)r_scaled;
        config->led_data[offset + 1] = (uint8_t)g_scaled;
        config->led_data[offset + 2] = (uint8_t)b_scaled;
        break;
    }

    // データを送信
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    esp_err_t ret = rmt_transmit(config->rmt_channel, config->rmt_encoder,
                                   config->led_data, config->num_leds * 3, &tx_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send WS2812B data: %s", esp_err_to_name(ret));
        return ret;
    }

    // 送信完了を待つ
    ret = rmt_tx_wait_all_done(config->rmt_channel, 100);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "WS2812B transmission wait timeout");
    }

    ESP_LOGI(TAG, "Set LED%d on GPIO%d (R=%d, G=%d, B=%d → R=%d, G=%d, B=%d)",
             led_index, pin, r, g, b, (uint8_t)r_scaled, (uint8_t)g_scaled, (uint8_t)b_scaled);

    return ESP_OK;
}

esp_err_t ws2812b_set_pattern(uint8_t pin, uint8_t led_index,
                               uint8_t pattern_type, uint8_t param1, uint8_t param2)
{
    portMUX_TYPE *mutex = main_get_gpio_states_mutex();
    bleio_gpio_state_t *state = main_get_gpio_state(pin);

    portENTER_CRITICAL(mutex);
    bleio_mode_state_t mode = state->mode;
    portEXIT_CRITICAL(mutex);

    if (mode != BLEIO_MODE_WS2812B)
    {
        ESP_LOGE(TAG, "GPIO%d is not in WS2812B mode", pin);
        return ESP_ERR_INVALID_STATE;
    }

    ws2812b_config_t *config = &ws2812b_configs[pin];

    // パターンタイプの検証 (0-4 および 0xFF (UNSET) を許可)
    if (pattern_type > WS2812B_PATTERN_FLICKER && pattern_type != WS2812B_PATTERN_UNSET)
    {
        ESP_LOGE(TAG, "Invalid pattern type: %d", pattern_type);
        return ESP_ERR_INVALID_ARG;
    }

    // LED 番号 0 に対して UNSET は無効
    if (led_index == 0 && pattern_type == WS2812B_PATTERN_UNSET)
    {
        ESP_LOGE(TAG, "Cannot set UNSET pattern to GPIO-wide pattern (led_index=0)");
        return ESP_ERR_INVALID_ARG;
    }

    if (led_index == 0)
    {
        // GPIO 全体のパターン設定
        config->gpio_pattern.pattern_type = pattern_type;
        config->gpio_pattern.pattern_param1 = param1;
        config->gpio_pattern.pattern_param2 = param2;
        config->gpio_pattern.hue = 0; // 色相リセット

        ESP_LOGI(TAG, "Set GPIO%d pattern to %d (all LEDs, param1=%d, param2=%d)",
                 pin, pattern_type, param1, param2);
    }
    else
    {
        // 個別 LED のパターン設定
        if (led_index > config->num_leds)
        {
            ESP_LOGE(TAG, "LED index %d out of range (max: %d)", led_index, config->num_leds);
            return ESP_ERR_INVALID_ARG;
        }

        // LED パターン配列が未割り当てなら割り当て
        if (config->led_patterns == NULL)
        {
            config->led_patterns = (ws2812b_led_pattern_t *)malloc(
                config->num_leds * sizeof(ws2812b_led_pattern_t));
            if (config->led_patterns == NULL)
            {
                ESP_LOGE(TAG, "Failed to allocate led_patterns");
                return ESP_ERR_NO_MEM;
            }

            // すべて未設定 (0xFF) で初期化
            for (uint16_t i = 0; i < config->num_leds; i++)
            {
                config->led_patterns[i].pattern_type = WS2812B_PATTERN_UNSET;
                config->led_patterns[i].pattern_param1 = 0;
                config->led_patterns[i].pattern_param2 = 0;
                config->led_patterns[i].hue = 0;
            }
        }

        // LED パターン設定
        uint16_t idx = led_index - 1; // LED 番号 1 = インデックス 0
        config->led_patterns[idx].pattern_type = pattern_type;
        config->led_patterns[idx].pattern_param1 = param1;
        config->led_patterns[idx].pattern_param2 = param2;
        config->led_patterns[idx].hue = 0;

        if (pattern_type == WS2812B_PATTERN_UNSET)
        {
            ESP_LOGI(TAG, "Cleared GPIO%d LED%d individual pattern (will use GPIO pattern)",
                     pin, led_index);
        }
        else
        {
            ESP_LOGI(TAG, "Set GPIO%d LED%d pattern to %d (param1=%d, param2=%d)",
                     pin, led_index, pattern_type, param1, param2);
        }
    }

    return ESP_OK;
}

ws2812b_config_t* ws2812b_get_config(uint8_t pin)
{
    return &ws2812b_configs[pin];
}
