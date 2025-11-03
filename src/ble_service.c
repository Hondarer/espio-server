#include "ble_service.h"
#include "gpio_basic.h"
#include "gpio_pwm.h"
#include "gpio_adc.h"
#include "ws2812b.h"
#include "main.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

static const char *TAG = "BLEIO-BLE";

// サービスとキャラクタリスティックの UUID
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x4b, 0x91, 0x33, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
                     0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f);

static const ble_uuid128_t gatt_svr_chr_write_uuid =
    BLE_UUID128_INIT(0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
                     0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe);

static const ble_uuid128_t gatt_svr_chr_read_uuid =
    BLE_UUID128_INIT(0x7e, 0xe8, 0x7b, 0x5d, 0x2e, 0x7a, 0x3d, 0xbf,
                     0x3a, 0x41, 0xf7, 0xd8, 0xe3, 0xd5, 0x95, 0x1c);

static const ble_uuid128_t gatt_svr_chr_adc_read_uuid =
    BLE_UUID128_INIT(0x1d, 0x0c, 0x9b, 0x8a, 0x7f, 0x6e, 0x5d, 0x8c,
                     0x1b, 0x4a, 0x9f, 0x4e, 0x3c, 0x7b, 0x8a, 0x2d);

// 認証モードフラグ
static bool auth_mode_enabled = false;

// 内部関数のプロトタイプ
static int gatt_svr_chr_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_svr_chr_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_svr_chr_adc_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void ble_app_set_security(void);
static void ble_app_on_sync(void);

// GPIO 書き込みキャラクタリスティック コールバック
static int gatt_svr_chr_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    struct os_mbuf *om = ctxt->om;
    uint16_t len = OS_MBUF_PKTLEN(om);

    // 最小長チェック: 1 (コマンド個数) + 6 (最低1コマンド)
    if (len < 7)
    {
        ESP_LOGE(TAG, "Invalid write length: %d (minimum 7)", len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t cmd_count;
    os_mbuf_copydata(om, 0, 1, &cmd_count);

    // パケット長チェック
    uint16_t expected_len = 1 + (cmd_count * 6);
    if (len != expected_len)
    {
        ESP_LOGE(TAG, "Invalid packet length: %d (expected %d for %d commands)",
                 len, expected_len, cmd_count);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    ESP_LOGI(TAG, "Received %d commands", cmd_count);

    // 各コマンドを処理
    for (int i = 0; i < cmd_count; i++)
    {
        uint8_t cmd_data[6];
        os_mbuf_copydata(om, 1 + (i * 6), 6, cmd_data);

        uint8_t pin = cmd_data[0];
        uint8_t command = cmd_data[1];
        uint8_t param1 = cmd_data[2];
        uint8_t param2 = cmd_data[3];
        uint8_t param3 = cmd_data[4];
        uint8_t param4 = cmd_data[5];

        ESP_LOGI(TAG, "Command %d: pin=%d, command=%d, param1=%d, param2=%d, param3=%d, param4=%d",
                 i + 1, pin, command, param1, param2, param3, param4);

        esp_err_t ret;
        if (command == CMD_SET_OUTPUT_LOW || command == CMD_SET_OUTPUT_HIGH)
        {
            ret = gpio_basic_write_level(pin, command);
        }
        else if (command == CMD_SET_OUTPUT_BLINK_250MS || command == CMD_SET_OUTPUT_BLINK_500MS)
        {
            ret = gpio_basic_start_blink(pin, command);
        }
        else if (command == CMD_SET_OUTPUT_PWM)
        {
            ret = gpio_pwm_set(pin, param1, param2); // param1 = duty_cycle, param2 = freq_preset
        }
        else if (command == CMD_SET_OUTPUT_ON_DISCONNECT)
        {
            ret = gpio_basic_set_disconnect_behavior(pin, param1); // param1 = disconnect_behavior
        }
        else if (command == CMD_SET_OUTPUT_WS2812B_ENABLE)
        {
            uint16_t num_leds = param1;
            uint8_t brightness = (param2 == 0) ? 255 : param2; // 0 の場合は 100% (255)
            ret = ws2812b_enable(pin, num_leds, brightness); // param1 = num_leds, param2 = brightness
        }
        else if (command == CMD_SET_OUTPUT_WS2812B_BASECOLOR)
        {
            uint16_t led_index = param1;
            uint8_t r = param2;
            uint8_t g = param3;
            uint8_t b = param4;
            ret = ws2812b_set_color(pin, led_index, r, g, b); // param1 = led_index, param2-4 = RGB
        }
        else if (command == CMD_SET_OUTPUT_WS2812B_PATTERN)
        {
            uint8_t led_index = param1;
            uint8_t pattern_type = param2;
            uint8_t pattern_param1 = param3;
            uint8_t pattern_param2 = param4;
            ret = ws2812b_set_pattern(pin, led_index, pattern_type, pattern_param1, pattern_param2);
        }
        else if (command >= CMD_SET_INPUT_FLOATING && command <= CMD_SET_INPUT_PULLDOWN)
        {
            ret = gpio_basic_set_mode(pin, command, param1); // param1 = latch_mode
        }
        else if (command == CMD_SET_ADC_ENABLE)
        {
            ret = gpio_adc_enable(pin, param1); // param1 = attenuation
        }
        else if (command == CMD_SET_ADC_DISABLE)
        {
            ret = gpio_adc_disable(pin);
        }
        else
        {
            ESP_LOGE(TAG, "Unknown command: %d", command);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Command %d failed", i + 1);
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    return 0;
}

// GPIO 読み取りキャラクタリスティック コールバック
static int gatt_svr_chr_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        // READ 操作: すべての入力モード設定済みピンの状態を返す
        uint8_t buffer[1 + MAX_USABLE_GPIO * 2]; // 1 (カウント) + 24 * 2 (ピン番号と状態) = 49 バイト
        uint8_t count = 0;

        portMUX_TYPE *mutex = main_get_gpio_states_mutex();

        // すべての GPIO をスキャンして、入力モードのピンを収集
        for (int pin = 0; pin < 40; pin++)
        {
            if (!main_is_valid_gpio(pin))
            {
                continue;
            }

            bleio_gpio_state_t *state = main_get_gpio_state(pin);

            portENTER_CRITICAL(mutex);
            bleio_mode_state_t mode = state->mode;
            uint8_t latch_mode = state->latch_mode;
            bool is_latched = state->is_latched;
            portEXIT_CRITICAL(mutex);

            // 入力モードかチェック
            if (mode == BLEIO_MODE_INPUT_FLOATING ||
                mode == BLEIO_MODE_INPUT_PULLUP ||
                mode == BLEIO_MODE_INPUT_PULLDOWN)
            {

                uint8_t level;

                // ラッチモードの処理
                if (latch_mode == LATCH_MODE_NONE)
                {
                    // ラッチなし: 現在の GPIO レベルをそのまま返す
                    level = gpio_get_level(pin);
                }
                else
                {
                    // ラッチあり
                    if (is_latched)
                    {
                        // ラッチ済み: ターゲット値を返す
                        level = (latch_mode == LATCH_MODE_HIGH) ? 1 : 0;
                    }
                    else
                    {
                        // 未ラッチ: ターゲット値の逆を返す (過渡状態での誤検出を避ける)
                        level = (latch_mode == LATCH_MODE_HIGH) ? 0 : 1;
                    }
                }

                buffer[1 + count * 2] = pin;
                buffer[1 + count * 2 + 1] = level;
                count++;

                ESP_LOGI(TAG, "Input GPIO%d: %s (latch_mode=%d, is_latched=%d)",
                         pin, level ? "HIGH" : "LOW", latch_mode, is_latched);
            }
        }

        buffer[0] = count;
        uint16_t data_len = 1 + count * 2;

        ESP_LOGI(TAG, "Sending %d input states (%d bytes)", count, data_len);
        int rc = os_mbuf_append(ctxt->om, buffer, data_len);
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    // WRITE 操作は無効
    return BLE_ATT_ERR_UNLIKELY;
}

// ADC 読み取りキャラクタリスティック コールバック
static int gatt_svr_chr_adc_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        // READ 操作: すべての ADC モード設定済みピンの値を返す
        uint8_t buffer[1 + MAX_USABLE_GPIO * 3]; // 1 (カウント) + 24 * 3 (ピン番号と ADC 値) = 73 バイト
        uint8_t count = 0;

        portMUX_TYPE *mutex = main_get_gpio_states_mutex();

        // すべての GPIO をスキャンして、ADC モードのピンを収集
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

            // ADC モードかチェック
            if (mode == BLEIO_MODE_ADC)
            {
                uint16_t adc_value = gpio_adc_read_value(pin);

                buffer[1 + count * 3] = pin;
                buffer[1 + count * 3 + 1] = adc_value & 0xFF;        // 下位バイト
                buffer[1 + count * 3 + 2] = (adc_value >> 8) & 0xFF; // 上位バイト
                count++;

                ESP_LOGI(TAG, "ADC GPIO%d: %d (0x%03X)", pin, adc_value, adc_value);
            }
        }

        buffer[0] = count;
        uint16_t data_len = 1 + count * 3;

        ESP_LOGI(TAG, "Sending %d ADC values (%d bytes)", count, data_len);
        int rc = os_mbuf_append(ctxt->om, buffer, data_len);
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    // WRITE 操作は無効
    return BLE_ATT_ERR_UNLIKELY;
}

// GATT サービス定義
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                // GPIO 書き込みキャラクタリスティック
                .uuid = &gatt_svr_chr_write_uuid.u,
                .access_cb = gatt_svr_chr_write_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                // GPIO 読み取りキャラクタリスティック
                .uuid = &gatt_svr_chr_read_uuid.u,
                .access_cb = gatt_svr_chr_read_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                // ADC 読み取りキャラクタリスティック
                .uuid = &gatt_svr_chr_adc_read_uuid.u,
                .access_cb = gatt_svr_chr_adc_read_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                0, // 終端
            }},
    },
    {
        0, // 終端
    },
};

// BLE GAP イベントハンドラ
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        if (event->connect.status == 0)
        {
            main_set_conn_handle(event->connect.conn_handle);

            // 接続パラメータを更新して電力変動を抑制
            // より長い Connection Interval を使用することで電力ピークを分散
            struct ble_gap_upd_params params = {
                .itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MAX,  // 30ms
                .itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX,  // 30ms
                .latency = 0,
                .supervision_timeout = 500,  // 5000ms
                .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
                .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN,
            };
            ble_gap_update_params(main_get_conn_handle(), &params);
            ESP_LOGI(TAG, "Updated connection parameters for stable power consumption");
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnect; reason=%d", event->disconnect.reason);
        main_set_conn_handle(0);

        portMUX_TYPE *mutex = main_get_gpio_states_mutex();

        // 切断時の振る舞いに従って GPIO を設定
        for (int pin = 0; pin < 40; pin++)
        {
            if (!main_is_valid_gpio(pin))
            {
                continue;
            }

            bleio_gpio_state_t *state = main_get_gpio_state(pin);

            portENTER_CRITICAL(mutex);
            bleio_mode_state_t mode = state->mode;
            uint8_t disconnect_behavior = state->disconnect_behavior;
            portEXIT_CRITICAL(mutex);

            // 出力モードかつ切断時の振る舞いが設定されている場合
            if (disconnect_behavior != 0)
            {
                if (mode == BLEIO_MODE_WS2812B)
                {
                    // WS2812B モードの場合
                    if (disconnect_behavior == 1)
                    {
                        // すべての LED を消灯 (モードは維持)
                        ws2812b_turn_off_all_leds(pin);
                        ESP_LOGI(TAG, "Turned off all WS2812B LEDs on GPIO%d due to disconnect", pin);
                    }
                    // disconnect_behavior == 2 の場合は何もしない (状態を維持)
                }
                else if (mode == BLEIO_MODE_OUTPUT_LOW || mode == BLEIO_MODE_OUTPUT_HIGH ||
                         mode == BLEIO_MODE_BLINK_250MS || mode == BLEIO_MODE_BLINK_500MS ||
                         mode == BLEIO_MODE_PWM)
                {
                    // その他の出力モードの場合
                    if (disconnect_behavior == 1)
                    {
                        // LOW に設定
                        gpio_basic_write_level(pin, CMD_SET_OUTPUT_LOW);
                        ESP_LOGI(TAG, "Set GPIO%d to LOW due to disconnect", pin);
                    }
                    else if (disconnect_behavior == 2)
                    {
                        // HIGH に設定
                        gpio_basic_write_level(pin, CMD_SET_OUTPUT_HIGH);
                        ESP_LOGI(TAG, "Set GPIO%d to HIGH due to disconnect", pin);
                    }
                }
            }
        }

        // 再度アドバタイズを開始
        ble_service_start_advertising();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE advertise complete");
        break;
    }
    return 0;
}

// BLE セキュリティ設定
static void ble_app_set_security(void)
{
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;  // ボンディング有効
    ble_hs_cfg.sm_mitm = 0;     // MITM 保護なし (NoInputNoOutput のため)
    ble_hs_cfg.sm_sc = 1;       // Secure Connections 有効
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;
}

// BLE アドバタイズ開始
void ble_service_start_advertising(void)
{
    // Advertising data: Flags + 128-bit Service UUID(s)
    struct ble_hs_adv_fields adv;
    memset(&adv, 0, sizeof(adv));

    // 一般発見 + BR/EDR 非対応
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // 128-bit サービス UUID を広告に載せる
    adv.uuids128 = &gatt_svr_svc_uuid;
    adv.num_uuids128 = 1;
    adv.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: rc=%d", rc);
        return;
    }

    // Scan Response: デバイス名を応答する
    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (uint8_t *)ble_svc_gap_device_name();
    rsp.name_len = strlen((char *)rsp.name);
    rsp.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: rc=%d", rc);
        return;
    }

    // アドバタイズ・パラメータ
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;         // 接続可
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;         // 一般発見可能
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN; // 30ms
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX; // 60ms

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "Advertising with 128-bit Service UUID");
    }
}

// BLE 同期コールバック
static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "BLE host synchronized");

    // MAC アドレスを取得
    uint8_t own_addr_type;
    uint8_t addr[6];
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to infer address type: rc=%d", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(own_addr_type, addr, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to copy address: rc=%d", rc);
        return;
    }

    // MAC アドレスの下位3バイトを使用してデバイス名を生成
    // addr[0] が最下位バイト、addr[5] が最上位バイト
    // 下位3バイトを表示するには addr[2], addr[1], addr[0] の順
    char device_name[32];
    if (auth_mode_enabled)
    {
        snprintf(device_name, sizeof(device_name), "BLEIO %02x%02x%02x [SEC]",
                 addr[2], addr[1], addr[0]);
    }
    else
    {
        snprintf(device_name, sizeof(device_name), "BLEIO %02x%02x%02x",
                 addr[2], addr[1], addr[0]);
    }

    // デバイス名を設定
    ble_svc_gap_device_name_set(device_name);
    ESP_LOGI(TAG, "Device name set to: %s", device_name);

    ble_hs_util_ensure_addr(0);
    ble_service_start_advertising();
}

// BLE ホストタスク
void ble_service_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// BLE サービス初期化
void ble_service_init(bool auth_enabled)
{
    auth_mode_enabled = auth_enabled;

    ESP_LOGI(TAG, "Initializing BLE service (auth_mode=%s)", auth_enabled ? "enabled" : "disabled");

    // NVS 初期化
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // NimBLE スタック初期化
    ESP_ERROR_CHECK(nimble_port_init());

    // BLE ホスト設定
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = NULL;

    // セキュリティ設定 (認証モードが有効な場合)
    if (auth_enabled)
    {
        ble_app_set_security();
    }

    // GATT サービス初期化
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    ESP_LOGI(TAG, "BLE service initialized");
}
