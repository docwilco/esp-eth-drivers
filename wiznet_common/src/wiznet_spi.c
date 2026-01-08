/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "wiznet_spi.h"

static const char *TAG = "wiznet.spi";

#define WIZNET_SPI_LOCK_TIMEOUT_MS (50)

/**
 * @brief Internal SPI context structure
 */
typedef struct {
    spi_device_handle_t hdl;
    SemaphoreHandle_t lock;
} eth_spi_info_t;

static inline bool wiznet_spi_lock(eth_spi_info_t *spi)
{
    return xSemaphoreTake(spi->lock, pdMS_TO_TICKS(WIZNET_SPI_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static inline bool wiznet_spi_unlock(eth_spi_info_t *spi)
{
    return xSemaphoreGive(spi->lock) == pdTRUE;
}

void *wiznet_spi_init(const void *spi_config)
{
    void *ret = NULL;
    const wiznet_spi_config_t *config = (const wiznet_spi_config_t *)spi_config;
    eth_spi_info_t *spi = calloc(1, sizeof(eth_spi_info_t));
    ESP_GOTO_ON_FALSE(spi, NULL, err, TAG, "no memory for SPI context data");

    /* SPI device init */
    spi_device_interface_config_t spi_devcfg;
    spi_devcfg = *(config->spi_devcfg);
    if (config->spi_devcfg->command_bits == 0 && config->spi_devcfg->address_bits == 0) {
        /* configure default SPI frame format for WIZnet chips */
        spi_devcfg.command_bits = 16; // Actually it's the address phase in WIZnet SPI frame
        spi_devcfg.address_bits = 8;  // Actually it's the control phase WIZnet SPI frame
    } else {
        ESP_GOTO_ON_FALSE(config->spi_devcfg->command_bits == 16 && config->spi_devcfg->address_bits == 8,
                          NULL, err, TAG, "incorrect SPI frame format (command_bits/address_bits)");
    }
    ESP_GOTO_ON_FALSE(spi_bus_add_device(config->spi_host_id, &spi_devcfg, &spi->hdl) == ESP_OK, NULL,
                      err, TAG, "adding device to SPI host #%i failed", config->spi_host_id + 1);
    /* create mutex */
    spi->lock = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(spi->lock, NULL, err, TAG, "create lock failed");

    ret = spi;
    return ret;
err:
    if (spi) {
        if (spi->hdl) {
            spi_bus_remove_device(spi->hdl);
        }
        if (spi->lock) {
            vSemaphoreDelete(spi->lock);
        }
        free(spi);
    }
    return ret;
}

esp_err_t wiznet_spi_deinit(void *spi_ctx)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_bus_remove_device(spi->hdl);
    vSemaphoreDelete(spi->lock);

    free(spi);
    return ret;
}

esp_err_t wiznet_spi_write(void *spi_ctx, uint32_t cmd, uint32_t addr, const void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_transaction_t trans = {
        .cmd = cmd,
        .addr = addr,
        .length = 8 * len,
        .tx_buffer = value
    };
    if (wiznet_spi_lock(spi)) {
        if (spi_device_polling_transmit(spi->hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        wiznet_spi_unlock(spi);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    return ret;
}

esp_err_t wiznet_spi_read(void *spi_ctx, uint32_t cmd, uint32_t addr, void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_transaction_t trans = {
        .flags = len <= 4 ? SPI_TRANS_USE_RXDATA : 0, // use direct reads for registers to prevent overwrites by 4-byte boundary writes
        .cmd = cmd,
        .addr = addr,
        .length = 8 * len,
        .rx_buffer = value
    };
    if (wiznet_spi_lock(spi)) {
        if (spi_device_polling_transmit(spi->hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        wiznet_spi_unlock(spi);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    if ((trans.flags & SPI_TRANS_USE_RXDATA) && len <= 4) {
        memcpy(value, trans.rx_data, len);  // copy register values to output
    }
    return ret;
}

uint32_t wiznet_spi_probe_max_transfer_sz(void *spi_ctx, uint32_t min_size, uint32_t max_size)
{
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;
    if (!spi || !spi->hdl) {
        ESP_LOGE(TAG, "probe: invalid spi context");
        return 0;
    }

    /* Use at least 1 byte for probing to see what the bus actually supports */
    if (min_size < 1) {
        min_size = 1;
    }

    ESP_LOGI(TAG, "probe: min_size=%" PRIu32 ", max_size=%" PRIu32, min_size, max_size);

    /* Binary search for maximum transfer size
     * The SPI driver returns ESP_ERR_INVALID_SIZE if the transfer exceeds max_transfer_sz
     */
    uint32_t low = min_size;
    uint32_t high = max_size;
    uint32_t result = 0;

    /* Allocate a small test buffer - we only need the SPI driver to check the size,
     * we don't actually need to transfer this much data */
    uint8_t dummy_buf[4] = {0};

    /* First verify minimum size works */
    spi_transaction_t test_trans = {
        .length = 8 * min_size,
        .tx_buffer = dummy_buf,
        .rx_buffer = NULL,
    };
    ESP_LOGI(TAG, "probe: testing min_size=%" PRIu32 " bytes", min_size);
    esp_err_t err = spi_device_polling_transmit(spi->hdl, &test_trans);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI min_size test failed (err=0x%x), max_transfer_sz too small (need at least %" PRIu32 " bytes)", err, min_size);
        return 0;
    }
    ESP_LOGI(TAG, "probe: min_size test passed");

    ESP_LOGI(TAG, "probe: binary search low=%" PRIu32 ", high=%" PRIu32, low, high);
    while (low <= high) {
        uint32_t mid = low + (high - low) / 2;

        /* Try a zero-length transaction with the size set - the driver validates
         * the size before doing any actual transfer */
        spi_transaction_t trans = {
            .length = 8 * mid,
            .tx_buffer = dummy_buf,
            .rx_buffer = NULL,
        };

        /* Use polling_transmit to check if size is valid */
        esp_err_t err = spi_device_polling_transmit(spi->hdl, &trans);

        if (err == ESP_OK) {
            /* This size works, try larger */
            ESP_LOGD(TAG, "probe: mid=%" PRIu32 " OK, trying larger", mid);
            result = mid;
            low = mid + 1;
        } else if (err == ESP_ERR_INVALID_SIZE || err == ESP_ERR_INVALID_ARG) {
            /* Too large, try smaller
             * ESP_ERR_INVALID_SIZE: transfer exceeds max_transfer_sz
             * ESP_ERR_INVALID_ARG: SPI driver validates tx_buffer size against length */
            ESP_LOGD(TAG, "probe: mid=%" PRIu32 " too large (err=0x%x), trying smaller", mid, err);
            high = mid - 1;
        } else {
            /* Other error, use last known good size */
            ESP_LOGW(TAG, "probe: mid=%" PRIu32 " unexpected error 0x%x, stopping", mid, err);
            break;
        }
    }

    ESP_LOGI(TAG, "Probed SPI max transfer size: %" PRIu32 " bytes", result);
    return result;
}
