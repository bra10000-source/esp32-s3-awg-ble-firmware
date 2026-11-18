#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "AWG_BLE";

// Exact 12-byte packed binary framing structure matching the Flutter AWG-4 app
#pragma pack(push, 1)
struct WaveformPacket {
    uint8_t channel;
    uint8_t waveType;
    uint32_t frequency;
    uint16_t amplitude;
    int16_t offset;
    uint16_t phase;
};
#pragma pack(pop)

// Nordic UART Service (NUS) UUIDs
// Primary Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E (Reversed for little-endian UUID128)
static const ble_uuid128_t nus_svc_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

// RX Characteristic (Write): 6E400002-...
static const ble_uuid128_t nus_chr_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

// TX Characteristic (Notify): 6E400003-...
static const ble_uuid128_t nus_chr_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint16_t tx_val_handle;
static uint16_t conn_handle_global = BLE_HS_CONN_HANDLE_NONE;

// Callback for incoming write packets from the mobile application
static int nus_rx_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len == sizeof(WaveformPacket)) {
            WaveformPacket packet{};
            ble_hs_mbuf_to_flat(ctxt->om, &packet, sizeof(packet), NULL);

            // Log parsed parameters
            ESP_LOGI(TAG, "=== AWG [CH %d] Update Received ===", packet.channel);
            ESP_LOGI(TAG, "Waveform:  %d", packet.waveType);
            ESP_LOGI(TAG, "Frequency: %lu Hz", (unsigned long)packet.frequency);
            ESP_LOGI(TAG, "Amplitude: %u mV", packet.amplitude);
            ESP_LOGI(TAG, "Offset:    %d mV", packet.offset);
            ESP_LOGI(TAG, "Phase:     %u deg", packet.phase);

            // Return 0x06 (ACK) notification to mobile app
            uint8_t ack = 0x06;
            struct os_mbuf *om = ble_hs_mbuf_from_flat(&ack, sizeof(ack));
            if (om && conn_handle_global != BLE_HS_CONN_HANDLE_NONE) {
                ble_gatts_notify_custom(conn_handle_global, tx_val_handle, om);
                ESP_LOGI(TAG, "ACK sent -> CH %d (status 0x06)", packet.channel);
            }
        } else {
            ESP_LOGW(TAG, "Invalid packet size received: %u bytes", len);
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int nus_tx_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

// Explicit C++26-compliant GATT Service Table Definition
static struct ble_gatt_chr_def nus_chrs[3];
static struct ble_gatt_svc_def gatt_svcs[2];

static void init_gatt_defs(void) {
    memset(nus_chrs, 0, sizeof(nus_chrs));
    memset(gatt_svcs, 0, sizeof(gatt_svcs));

    // RX Characteristic (Write)
    nus_chrs[0].uuid = &nus_chr_rx_uuid.u;
    nus_chrs[0].access_cb = nus_rx_access;
    nus_chrs[0].flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP;

    // TX Characteristic (Notify)
    nus_chrs[1].uuid = &nus_chr_tx_uuid.u;
    nus_chrs[1].access_cb = nus_tx_access;
    nus_chrs[1].flags = BLE_GATT_CHR_F_NOTIFY;
    nus_chrs[1].val_handle = &tx_val_handle;

    // Primary NUS Service
    gatt_svcs[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    gatt_svcs[0].uuid = &nus_svc_uuid.u;
    gatt_svcs[0].characteristics = nus_chrs;
}

static void start_advertising(void);

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                conn_handle_global = event->connect.conn_handle;
                ESP_LOGI(TAG, "Connected to mobile app (conn_handle: %u)", conn_handle_global);
            } else {
                start_advertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            conn_handle_global = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGI(TAG, "Disconnected; restarting BLE advertising");
            start_advertising();
            break;
    }
    return 0;
}

static void start_advertising(void) {
    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // 1. Primary Advertising Packet: Flags + 128-bit NUS Service UUID (fits inside 31-byte limit)
    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &nus_svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields: %d", rc);
        return;
    }

    // 2. Scan Response Packet: Complete Device Name "AWG-4"
    struct ble_hs_adv_fields rsp_fields{};
    const char *device_name = "AWG-4";
    rsp_fields.name = (uint8_t *)device_name;
    rsp_fields.name_len = strlen(device_name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan response fields: %d", rc);
        return;
    }

    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, gap_event_handler, NULL);
    ESP_LOGI(TAG, "BLE advertising started as 'AWG-4' with NUS UUID");
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void on_sync(void) {
    start_advertising();
}

extern "C" void app_main(void) {
    // 1. Initialize NVS (required by BLE subsystem)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize NimBLE host stack
    ESP_ERROR_CHECK(nimble_port_init());

    // 3. Initialize GAP and GATT services
    init_gatt_defs();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_hs_cfg.sync_cb = on_sync;

    // 4. Run NimBLE host in FreeRTOS task
    nimble_port_freertos_init(ble_host_task);
}