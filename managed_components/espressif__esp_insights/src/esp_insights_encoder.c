/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <esp_diagnostics.h>
#include <esp_diagnostics_metrics.h>
#include <esp_diagnostics_variables.h>

#include "esp_insights_cbor_encoder.h"

#if CONFIG_ESP_INSIGHTS_META_VERSION_10
#define INSIGHTS_VERSION_MAJOR           "1"
#else
#define INSIGHTS_VERSION_MAJOR           "2"
#endif
#define INSIGHTS_VERSION_MINOR           "0"
#define INSIGHTS_VERSION                 INSIGHTS_VERSION_MAJOR \
                                            "." INSIGHTS_VERSION_MINOR

#if CONFIG_ESP_INSIGHTS_META_VERSION_10
#define INSIGHTS_META_VERSION_MAJOR      "1"
#else
#define INSIGHTS_META_VERSION_MAJOR      "2"
#endif
#define INSIGHTS_META_VERSION_MINOR      "0"

#define INSIGHTS_META_VERSION            INSIGHTS_META_VERSION_MAJOR \
                                            "." INSIGHTS_META_VERSION_MINOR

#define INSIGHTS_DATA_TYPE          0x02
#define INSIGHTS_META_DATA_TYPE     0x03
#define INSIGHTS_CONF_DATA_TYPE     0x12
#define TLV_OFFSET                  3

static void esp_insights_encode_meta_data(void)
{
#if CONFIG_DIAG_ENABLE_METRICS
    uint32_t metrics_len = 0;
    const esp_diag_metrics_meta_t *metrics = esp_diag_metrics_meta_get_all(&metrics_len);
    if (!metrics) {
        return;
    }
    esp_insights_cbor_encode_meta_metrics((const esp_diag_metrics_meta_t *)metrics, metrics_len);
#endif /* CONFIG_DIAG_ENABLE_METRICS */

#if CONFIG_DIAG_ENABLE_VARIABLES
    uint32_t variables_len = 0;
    const esp_diag_variable_meta_t *variables = esp_diag_variable_meta_get_all(&variables_len);
    if (!variables) {
        return;
    }
    esp_insights_cbor_encode_meta_variables((const esp_diag_variable_meta_t *)variables, variables_len);
#endif /* CONFIG_DIAG_ENABLE_VARIABLES */
}

size_t esp_insights_encode_meta(uint8_t *out_data, size_t out_data_size, char *sha256)
{
    if (!out_data || !out_data_size) {
        return 0;
    }
    char sha[DIAG_HEX_SHA_SIZE + 1];
    bytes_to_hex((uint8_t *) sha256,(uint8_t *) sha, DIAG_SHA_SIZE);
    esp_insights_cbor_encode_meta_begin(out_data + TLV_OFFSET,
                                        out_data_size - TLV_OFFSET,
                                        INSIGHTS_META_VERSION, sha);
    esp_insights_cbor_encode_meta_data_begin();
    esp_insights_encode_meta_data();
    esp_insights_cbor_encode_meta_data_end();
    uint16_t len = esp_insights_cbor_encode_meta_end(out_data + TLV_OFFSET);

    out_data[0] = INSIGHTS_META_DATA_TYPE;      /* Data type inidcation diagnostics meta - 1 byte */
    memcpy(&out_data[1], &len, sizeof(len));    /* Data length - 2 bytes */
    len += TLV_OFFSET;
    return len;
}

esp_err_t esp_insights_encode_data_begin(void *out_data, size_t out_data_size)
{
    if (!out_data || !out_data_size) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_insights_cbor_encode_diag_begin(out_data + TLV_OFFSET, out_data_size - TLV_OFFSET, INSIGHTS_VERSION);
    esp_insights_cbor_encode_diag_data_begin();
    return ESP_OK;
}

size_t esp_insights_encode_conf_meta(uint8_t *out_data, size_t out_data_size, char *sha256)
{
    if (!out_data || !out_data_size) {
        return 0;
    }
    char sha[DIAG_HEX_SHA_SIZE + 1];
    bytes_to_hex((uint8_t *) sha256,(uint8_t *) sha, DIAG_SHA_SIZE);
    esp_insights_cbor_encode_meta_begin(out_data + TLV_OFFSET,
                                        out_data_size - TLV_OFFSET,
                                        INSIGHTS_META_VERSION, sha);
    esp_insights_cbor_encode_conf_meta_data_begin();
    /* TODO: Implement and collect diagnostics specific conf meta */
    // esp_insights_encode_conf_meta_data();
    esp_insights_cbor_encode_conf_meta_data_end();

    uint16_t len = esp_insights_cbor_encode_meta_end(out_data + TLV_OFFSET);

    out_data[0] = INSIGHTS_META_DATA_TYPE;      /* Data type inidcation diagnostics meta - 1 byte */
    memcpy(&out_data[1], &len, sizeof(len));    /* Data length - 2 bytes */
    len += TLV_OFFSET;
    return len;
}

void esp_insights_encode_boottime_data(void)
{
    /* encode device info */
    esp_diag_device_info_t device_info;
    memset(&device_info, 0, sizeof(device_info));
    esp_diag_device_info_get(&device_info);
    esp_insights_cbor_encode_diag_boot_info(&device_info);

    /* encode core dump summary */
#if CONFIG_ESP_INSIGHTS_COREDUMP_ENABLE
    const char *TAG = "Insights";
    esp_err_t err = esp_core_dump_image_check();
    if (err == ESP_OK) {
        esp_core_dump_summary_t *summary = malloc(sizeof(esp_core_dump_summary_t));
        if (summary) {
            memset(summary, 0, sizeof(esp_core_dump_summary_t));
            if (esp_core_dump_get_summary(summary) == ESP_OK) {
                esp_insights_cbor_encode_diag_crash(summary);
            }
            free(summary);
        }
    } else if (err == ESP_ERR_INVALID_CRC) {
        ESP_LOGE(TAG, "Core dump stored in flash is corrupted");
    }
#endif /* CONFIG_ESP_INSIGHTS_COREDUMP_ENABLE */
}

void esp_insights_encode_conf_data()
{
    /* collect the configs */
    esp_insights_cbor_encode_diag_conf_data_begin();
    esp_insights_cbor_encode_diag_conf_data();
    esp_insights_cbor_encode_diag_conf_data_end();
}


size_t esp_insights_encode_conf_end(uint8_t *out_data)
{
    if (!out_data) {
        return 0;
    }
    esp_insights_cbor_encode_diag_data_end();
    uint16_t len = esp_insights_cbor_encode_diag_end(out_data + TLV_OFFSET);

    out_data[0] = INSIGHTS_CONF_DATA_TYPE;      /* Data type indicating diagnostics - 1 byte */
    memcpy(&out_data[1], &len, sizeof(len));    /* Data length - 2 bytes */
    len += TLV_OFFSET;
    return len;
}

size_t esp_insights_encode_critical_data(const void *data, size_t data_size)
{
    size_t consumed = 0;
    if (data) {
        consumed = esp_insights_cbor_encode_diag_logs(data, data_size);
        if (consumed) {
            uint8_t meta_idx = ((uint8_t *) data)[0];
            const rtc_store_meta_header_t *hdr = rtc_store_get_meta_record_by_index(meta_idx);
            if (hdr) {
                esp_insights_cbor_encode_meta_c_hdr(hdr);
            }
        }
    }
    return consumed;
}

size_t esp_insights_encode_non_critical_data(const void *data, size_t data_size)
{
    size_t consumed_max = 0;
    if (data) {
#if CONFIG_DIAG_ENABLE_METRICS
        consumed_max = esp_insights_cbor_encode_diag_metrics(data, data_size);
#endif /* CONFIG_DIAG_ENABLE_METRICS */
#if CONFIG_DIAG_ENABLE_VARIABLES
        size_t consumed = esp_insights_cbor_encode_diag_variables(data, data_size);
        if (consumed > consumed_max) {
            consumed_max = consumed;
        }
#endif /* CONFIG_DIAG_ENABLE_VARIABLES */
#if CONFIG_DIAG_ENABLE_METRICS || CONFIG_DIAG_ENABLE_VARIABLES
        if (consumed_max) {
            uint8_t meta_idx = ((uint8_t *) data)[0];
            const rtc_store_meta_header_t *hdr = rtc_store_get_meta_record_by_index(meta_idx);
            if (hdr) {
                esp_insights_cbor_encode_meta_nc_hdr(hdr);
            }
        }
#endif
    }
    return consumed_max;
}

size_t esp_insights_encode_data_end(uint8_t *out_data)
{
    if (!out_data) {
        return 0;
    }
    esp_insights_cbor_encode_diag_data_end();
    uint16_t len = esp_insights_cbor_encode_diag_end(out_data + TLV_OFFSET);

    out_data[0] = INSIGHTS_DATA_TYPE;               /* Data type indicating diagnostics - 1 byte */
    memcpy(&out_data[1], &len, sizeof(len));    /* Data length - 2 bytes */
    len += TLV_OFFSET;
    return len;
}
