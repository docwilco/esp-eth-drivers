/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_eth_mac_w6100.h"
#include "esp_idf_version.h"
#include "w6100.h"

static const char *TAG = "w6100.mac";

#define W6100_SPI_LOCK_TIMEOUT_MS (50)
#define W6100_100M_TX_TMO_US (200)
#define W6100_10M_TX_TMO_US (1500)
#define W6100_ETH_MAC_RX_BUF_SIZE_AUTO (0)

typedef struct {
    uint32_t offset;
    uint32_t copy_len;
    uint32_t rx_len;
    uint32_t remain;
}__attribute__((packed)) emac_w6100_auto_buf_info_t;

typedef struct {
    spi_device_handle_t hdl;
    SemaphoreHandle_t lock;
} eth_spi_info_t;

typedef struct {
    void *ctx;
    void *(*init)(const void *spi_config);
    esp_err_t (*deinit)(void *spi_ctx);
    esp_err_t (*read)(void *spi_ctx, uint32_t cmd, uint32_t addr, void *data, uint32_t data_len);
    esp_err_t (*write)(void *spi_ctx, uint32_t cmd, uint32_t addr, const void *data, uint32_t data_len);
} eth_spi_custom_driver_t;

typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    eth_spi_custom_driver_t spi;
    TaskHandle_t rx_task_hdl;
    uint32_t sw_reset_timeout_ms;
    int int_gpio_num;
    esp_timer_handle_t poll_timer;
    uint32_t poll_period_ms;
    uint8_t addr[ETH_ADDR_LEN];
    bool packets_remain;
    uint8_t *rx_buffer;
    uint32_t tx_tmo;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    uint8_t mcast_v4_cnt;
    uint8_t mcast_v6_cnt;
#endif
} emac_w6100_t;

static void *w6100_spi_init(const void *spi_config)
{
    void *ret = NULL;
    eth_w6100_config_t *w6100_config = (eth_w6100_config_t *)spi_config;
    eth_spi_info_t *spi = calloc(1, sizeof(eth_spi_info_t));
    ESP_GOTO_ON_FALSE(spi, NULL, err, TAG, "no memory for SPI context data");

    /* SPI device init */
    spi_device_interface_config_t spi_devcfg;
    spi_devcfg = *(w6100_config->spi_devcfg);
    if (w6100_config->spi_devcfg->command_bits == 0 && w6100_config->spi_devcfg->address_bits == 0) {
        /* configure default SPI frame format */
        spi_devcfg.command_bits = 16; // Actually it's the address phase in W6100 SPI frame
        spi_devcfg.address_bits = 8;  // Actually it's the control phase in W6100 SPI frame
    } else {
        ESP_GOTO_ON_FALSE(w6100_config->spi_devcfg->command_bits == 16 && w6100_config->spi_devcfg->address_bits == 8,
                            NULL, err, TAG, "incorrect SPI frame format (command_bits/address_bits)");
    }
    ESP_GOTO_ON_FALSE(spi_bus_add_device(w6100_config->spi_host_id, &spi_devcfg, &spi->hdl) == ESP_OK, NULL,
                                            err, TAG, "adding device to SPI host #%i failed", w6100_config->spi_host_id + 1);
    /* create mutex */
    spi->lock = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(spi->lock, NULL, err, TAG, "create lock failed");

    ret = spi;
    return ret;
err:
    if (spi) {
        if (spi->lock) {
            vSemaphoreDelete(spi->lock);
        }
        free(spi);
    }
    return ret;
}

static esp_err_t w6100_spi_deinit(void *spi_ctx)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_bus_remove_device(spi->hdl);
    vSemaphoreDelete(spi->lock);

    free(spi);
    return ret;
}

static inline bool w6100_spi_lock(eth_spi_info_t *spi)
{
    return xSemaphoreTake(spi->lock, pdMS_TO_TICKS(W6100_SPI_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static inline bool w6100_spi_unlock(eth_spi_info_t *spi)
{
    return xSemaphoreGive(spi->lock) == pdTRUE;
}

static esp_err_t w6100_spi_write(void *spi_ctx, uint32_t cmd, uint32_t addr, const void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_transaction_t trans = {
        .cmd = cmd,
        .addr = addr,
        .length = 8 * len,
        .tx_buffer = value
    };
    if (w6100_spi_lock(spi)) {
        if (spi_device_polling_transmit(spi->hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        w6100_spi_unlock(spi);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    return ret;
}

static esp_err_t w6100_spi_read(void *spi_ctx, uint32_t cmd, uint32_t addr, void *value, uint32_t len)
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
    if (w6100_spi_lock(spi)) {
        if (spi_device_polling_transmit(spi->hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        w6100_spi_unlock(spi);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    if ((trans.flags & SPI_TRANS_USE_RXDATA) && len <= 4) {
        memcpy(value, trans.rx_data, len);  // copy register values to output
    }
    return ret;
}

static esp_err_t w6100_read(emac_w6100_t *emac, uint32_t address, void *data, uint32_t len)
{
    uint32_t cmd = (address >> W6100_ADDR_OFFSET); // Address phase in W6100 SPI frame
    uint32_t addr = ((address & 0xFFFF) | (W6100_ACCESS_MODE_READ << W6100_RWB_OFFSET)
                    | W6100_SPI_OP_MODE_VDM); // Control phase in W6100 SPI frame

    return emac->spi.read(emac->spi.ctx, cmd, addr, data, len);
}

static esp_err_t w6100_write(emac_w6100_t *emac, uint32_t address, const void *data, uint32_t len)
{
    uint32_t cmd = (address >> W6100_ADDR_OFFSET); // Address phase in W6100 SPI frame
    uint32_t addr = ((address & 0xFFFF) | (W6100_ACCESS_MODE_WRITE << W6100_RWB_OFFSET)
                    | W6100_SPI_OP_MODE_VDM); // Control phase in W6100 SPI frame

    return emac->spi.write(emac->spi.ctx, cmd, addr, data, len);
}

static esp_err_t w6100_send_command(emac_w6100_t *emac, uint8_t command, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_CR(0), &command, sizeof(command)), err, TAG, "write SCR failed");
    // after W6100 accepts the command, the command register will be cleared automatically
    uint32_t to = 0;
    for (to = 0; to < timeout_ms / 10; to++) {
        ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_CR(0), &command, sizeof(command)), err, TAG, "read SCR failed");
        if (!command) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_GOTO_ON_FALSE(to < timeout_ms / 10, ESP_ERR_TIMEOUT, err, TAG, "send command timeout");

err:
    return ret;
}

static esp_err_t w6100_get_tx_free_size(emac_w6100_t *emac, uint16_t *size)
{
    esp_err_t ret = ESP_OK;
    uint16_t free0, free1 = 0;
    // read TX_FSR register more than once, until we get the same value
    // this is a trick because we might be interrupted between reading the high/low part of the TX_FSR register (16 bits in length)
    do {
        ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_TX_FSR(0), &free0, sizeof(free0)), err, TAG, "read TX FSR failed");
        ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_TX_FSR(0), &free1, sizeof(free1)), err, TAG, "read TX FSR failed");
    } while (free0 != free1);

    *size = __builtin_bswap16(free0);

err:
    return ret;
}

static esp_err_t w6100_get_rx_received_size(emac_w6100_t *emac, uint16_t *size)
{
    esp_err_t ret = ESP_OK;
    uint16_t received0, received1 = 0;
    do {
        ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_RX_RSR(0), &received0, sizeof(received0)), err, TAG, "read RX RSR failed");
        ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_RX_RSR(0), &received1, sizeof(received1)), err, TAG, "read RX RSR failed");
    } while (received0 != received1);
    *size = __builtin_bswap16(received0);

err:
    return ret;
}

static esp_err_t w6100_write_buffer(emac_w6100_t *emac, const void *buffer, uint32_t len, uint16_t offset)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_MEM_SOCK_TX(0, offset), buffer, len), err, TAG, "write TX buffer failed");
err:
    return ret;
}

static esp_err_t w6100_read_buffer(emac_w6100_t *emac, void *buffer, uint32_t len, uint16_t offset)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_MEM_SOCK_RX(0, offset), buffer, len), err, TAG, "read RX buffer failed");
err:
    return ret;
}

static esp_err_t w6100_set_mac_addr(emac_w6100_t *emac)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SHAR, emac->addr, 6), err, TAG, "write MAC address register failed");

err:
    return ret;
}

static esp_err_t w6100_reset(emac_w6100_t *emac)
{
    esp_err_t ret = ESP_OK;
    /* software reset - write 0 to RST bit to trigger reset */
    uint8_t sycr0 = 0x00; // Clear RST bit (bit 7) to reset
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SYCR0, &sycr0, sizeof(sycr0)), err, TAG, "write SYCR0 failed");
    
    /* Wait for reset to complete - need to wait for chip to stabilize */
    vTaskDelay(pdMS_TO_TICKS(100));  // W6100 needs ~60.3ms after reset
    
    return ESP_OK;
err:
    return ret;
}

static esp_err_t w6100_verify_id(emac_w6100_t *emac)
{
    esp_err_t ret = ESP_OK;
    uint16_t chip_id = 0;
    uint16_t version = 0;

    // Read chip ID
    ESP_LOGD(TAG, "Waiting W6100 to start & verify chip ID...");
    uint32_t to = 0;
    for (to = 0; to < emac->sw_reset_timeout_ms / 10; to++) {
        ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_CIDR, &chip_id, sizeof(chip_id)), err, TAG, "read CIDR failed");
        chip_id = __builtin_bswap16(chip_id);
        if (chip_id == W6100_CHIP_ID) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (chip_id != W6100_CHIP_ID) {
        ESP_LOGE(TAG, "W6100 chip ID mismatched, expected 0x%04x, got 0x%04" PRIx16, W6100_CHIP_ID, chip_id);
        return ESP_ERR_INVALID_VERSION;
    }
    
    // Also verify version
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_VER, &version, sizeof(version)), err, TAG, "read VER failed");
    version = __builtin_bswap16(version);
    ESP_LOGI(TAG, "W6100 chip ID: 0x%04" PRIx16 ", version: 0x%04" PRIx16, chip_id, version);
    
    return ESP_OK;
err:
    return ret;
}

static esp_err_t w6100_setup_default(emac_w6100_t *emac)
{
    esp_err_t ret = ESP_OK;
    uint8_t reg_value = 16;

    // Unlock network configuration
    uint8_t unlock = W6100_NETLCKR_UNLOCK;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_NETLCKR, &unlock, sizeof(unlock)), err, TAG, "unlock network config failed");

    // Only SOCK0 can be used as MAC RAW mode, so we give the whole buffer (16KB TX and 16KB RX) to SOCK0
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_RX_BSR(0), &reg_value, sizeof(reg_value)), err, TAG, "set rx buffer size failed");
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_TX_BSR(0), &reg_value, sizeof(reg_value)), err, TAG, "set tx buffer size failed");
    reg_value = 0;
    for (int i = 1; i < 8; i++) {
        ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_RX_BSR(i), &reg_value, sizeof(reg_value)), err, TAG, "set rx buffer size failed");
        ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_TX_BSR(i), &reg_value, sizeof(reg_value)), err, TAG, "set tx buffer size failed");
    }

    /* Configure network mode - block ping responses for security */
    reg_value = 0;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_NETMR, &reg_value, sizeof(reg_value)), err, TAG, "write NETMR failed");
    
    /* Disable interrupt for all sockets by default */
    reg_value = 0;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SIMR, &reg_value, sizeof(reg_value)), err, TAG, "write SIMR failed");
    
    /* Enable MAC RAW mode for SOCK0, enable MAC filter */
    reg_value = W6100_SMR_MACRAW | W6100_SMR_MF;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    /* Block IPv4 and IPv6 multicast by default until add_mac_filter is called.
     * Per datasheet: MMB=1/MMB6=1 blocks multicast, MMB=0/MMB6=0 allows it. */
    reg_value |= W6100_SMR_MMB | W6100_SMR_MMB6;
#endif
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_MR(0), &reg_value, sizeof(reg_value)), err, TAG, "write SMR failed");
    
    /* Enable receive event for SOCK0 */
    reg_value = W6100_SIR_RECV;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_IMR(0), &reg_value, sizeof(reg_value)), err, TAG, "write SOCK0 IMR failed");
    
    /* Set the interrupt re-assert level to maximum (~1.5ms) to lower the chances of missing it */
    uint16_t int_level = __builtin_bswap16(0xFFFF);
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_INTPTMR, &int_level, sizeof(int_level)), err, TAG, "write INTPTMR failed");

    /* Enable global interrupt */
    reg_value = W6100_SYCR1_IEN;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SYCR1, &reg_value, sizeof(reg_value)), err, TAG, "write SYCR1 failed");

err:
    return ret;
}

static esp_err_t emac_w6100_start(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    uint8_t reg_value = 0;
    /* open SOCK0 */
    ESP_GOTO_ON_ERROR(w6100_send_command(emac, W6100_SCR_OPEN, 100), err, TAG, "issue OPEN command failed");

    /* enable interrupt for SOCK0 */
    reg_value = W6100_SIMR_SOCK0;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SIMR, &reg_value, sizeof(reg_value)), err, TAG, "write SIMR failed");

err:
    return ret;
}

static esp_err_t emac_w6100_stop(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    uint8_t reg_value = 0;
    /* disable interrupt */
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SIMR, &reg_value, sizeof(reg_value)), err, TAG, "write SIMR failed");
    /* close SOCK0 */
    ESP_GOTO_ON_ERROR(w6100_send_command(emac, W6100_SCR_CLOSE, 100), err, TAG, "issue CLOSE command failed");

err:
    return ret;
}

static esp_err_t emac_w6100_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(eth, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac's mediator to null");
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    emac->eth = eth;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_w6100_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    // PHY registers are mapped directly in W6100's register space
    // The phy_reg parameter contains the full W6100 register address
    uint8_t val = (uint8_t)reg_value;
    ESP_GOTO_ON_ERROR(w6100_write(emac, phy_reg, &val, sizeof(val)), err, TAG, "write PHY register failed");

err:
    return ret;
}

static esp_err_t emac_w6100_read_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(reg_value, ESP_ERR_INVALID_ARG, err, TAG, "can't set reg_value to null");
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    // PHY registers are mapped directly in W6100's register space
    // The phy_reg parameter contains the full W6100 register address
    uint8_t val = 0;
    ESP_GOTO_ON_ERROR(w6100_read(emac, phy_reg, &val, sizeof(val)), err, TAG, "read PHY register failed");
    *reg_value = val;

err:
    return ret;
}

static esp_err_t emac_w6100_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    memcpy(emac->addr, addr, 6);
    ESP_GOTO_ON_ERROR(w6100_set_mac_addr(emac), err, TAG, "set mac address failed");

err:
    return ret;
}

static esp_err_t emac_w6100_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    memcpy(addr, emac->addr, 6);

err:
    return ret;
}

static esp_err_t emac_w6100_set_link(esp_eth_mac_t *mac, eth_link_t link)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    switch (link) {
    case ETH_LINK_UP:
        ESP_LOGD(TAG, "link is up");
        ESP_GOTO_ON_ERROR(mac->start(mac), err, TAG, "w6100 start failed");
        if (emac->poll_timer) {
            ESP_GOTO_ON_ERROR(esp_timer_start_periodic(emac->poll_timer, emac->poll_period_ms * 1000),
                                err, TAG, "start poll timer failed");
        }
        break;
    case ETH_LINK_DOWN:
        ESP_LOGD(TAG, "link is down");
        ESP_GOTO_ON_ERROR(mac->stop(mac), err, TAG, "w6100 stop failed");
        if (emac->poll_timer) {
            ESP_GOTO_ON_ERROR(esp_timer_stop(emac->poll_timer),
                                err, TAG, "stop poll timer failed");
        }
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unknown link status");
        break;
    }

err:
    return ret;
}

static esp_err_t emac_w6100_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    switch (speed) {
    case ETH_SPEED_10M:
        emac->tx_tmo = W6100_10M_TX_TMO_US;
        ESP_LOGD(TAG, "working in 10Mbps");
        break;
    case ETH_SPEED_100M:
        emac->tx_tmo = W6100_100M_TX_TMO_US;
        ESP_LOGD(TAG, "working in 100Mbps");
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unknown speed");
        break;
    }
err:
    return ret;
}

static esp_err_t emac_w6100_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex)
{
    esp_err_t ret = ESP_OK;
    switch (duplex) {
    case ETH_DUPLEX_HALF:
        ESP_LOGD(TAG, "working in half duplex");
        break;
    case ETH_DUPLEX_FULL:
        ESP_LOGD(TAG, "working in full duplex");
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unknown duplex");
        break;
    }

err:
    return ret;
}

static esp_err_t emac_w6100_set_promiscuous(esp_eth_mac_t *mac, bool enable)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    uint8_t smr = 0;
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_MR(0), &smr, sizeof(smr)), err, TAG, "read SMR failed");
    if (enable) {
        smr &= ~W6100_SMR_MF;
    } else {
        smr |= W6100_SMR_MF;
    }
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_MR(0), &smr, sizeof(smr)), err, TAG, "write SMR failed");

err:
    return ret;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
/**
 * @brief Set multicast blocking state for IPv4 and IPv6
 * 
 * Per datasheet: MMB=1/MMB6=1 blocks multicast, MMB=0/MMB6=0 allows it.
 */
static esp_err_t emac_w6100_set_mcast_block(emac_w6100_t *emac, bool block_v4, bool block_v6)
{
    esp_err_t ret = ESP_OK;
    uint8_t smr;
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_MR(0), &smr, sizeof(smr)), err, TAG, "read SMR failed");
    ESP_LOGD(TAG, "set_mcast_block: block_v4=%d, block_v6=%d, SMR before=0x%02x", block_v4, block_v6, smr);
    /* Datasheet logic: set bit to block, clear bit to allow */
    if (block_v4) {
        smr |= W6100_SMR_MMB;    // Set to block
    } else {
        smr &= ~W6100_SMR_MMB;   // Clear to allow
    }
    if (block_v6) {
        smr |= W6100_SMR_MMB6;   // Set to block
    } else {
        smr &= ~W6100_SMR_MMB6;  // Clear to allow
    }
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_MR(0), &smr, sizeof(smr)), err, TAG, "write SMR failed");
    ESP_LOGD(TAG, "set_mcast_block: SMR after=0x%02x (MMB=%d, MMB6=%d)", smr, (smr & W6100_SMR_MMB) ? 1 : 0, (smr & W6100_SMR_MMB6) ? 1 : 0);
err:
    return ret;
}

static esp_err_t emac_w6100_add_mac_filter(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    ESP_LOGD(TAG, "add_mac_filter: %02x:%02x:%02x:%02x:%02x:%02x (v4_cnt=%d, v6_cnt=%d)",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
             emac->mcast_v4_cnt, emac->mcast_v6_cnt);
    // W6100 doesn't have specific MAC filter, so we just un-block multicast.
    if (addr[0] == 0x01 && addr[1] == 0x00 && addr[2] == 0x5e) {
        // IPv4 multicast
        if (emac->mcast_v4_cnt == 0) {
            ESP_GOTO_ON_ERROR(emac_w6100_set_mcast_block(emac, false, emac->mcast_v6_cnt == 0),
                              err, TAG, "set multicast block failed");
        }
        emac->mcast_v4_cnt++;
    } else if (addr[0] == 0x33 && addr[1] == 0x33) {
        // IPv6 multicast
        if (emac->mcast_v6_cnt == 0) {
            ESP_GOTO_ON_ERROR(emac_w6100_set_mcast_block(emac, emac->mcast_v4_cnt == 0, false),
                              err, TAG, "set multicast block failed");
        }
        emac->mcast_v6_cnt++;
    } else {
        ESP_LOGE(TAG, "W6100 filters in IP multicast frames only!");
        ret = ESP_ERR_NOT_SUPPORTED;
    }
err:
    return ret;
}

static esp_err_t emac_w6100_rm_mac_filter(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    ESP_LOGI(TAG, "rm_mac_filter: %02x:%02x:%02x:%02x:%02x:%02x (v4_cnt=%d, v6_cnt=%d)",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
             emac->mcast_v4_cnt, emac->mcast_v6_cnt);
    if (addr[0] == 0x01 && addr[1] == 0x00 && addr[2] == 0x5e) {
        // IPv4 multicast
        if (emac->mcast_v4_cnt > 0) {
            emac->mcast_v4_cnt--;
            if (emac->mcast_v4_cnt == 0) {
                ESP_GOTO_ON_ERROR(emac_w6100_set_mcast_block(emac, true, emac->mcast_v6_cnt == 0),
                                  err, TAG, "set multicast block failed");
            }
        }
    } else if (addr[0] == 0x33 && addr[1] == 0x33) {
        // IPv6 multicast
        if (emac->mcast_v6_cnt > 0) {
            emac->mcast_v6_cnt--;
            if (emac->mcast_v6_cnt == 0) {
                ESP_GOTO_ON_ERROR(emac_w6100_set_mcast_block(emac, emac->mcast_v4_cnt == 0, true),
                                  err, TAG, "set multicast block failed");
            }
        }
    } else {
        ESP_LOGE(TAG, "W6100 filters in IP multicast frames only!");
        ret = ESP_ERR_NOT_SUPPORTED;
    }
err:
    return ret;
}

static esp_err_t emac_w6100_set_all_multicast(esp_eth_mac_t *mac, bool enable)
{
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    ESP_RETURN_ON_ERROR(emac_w6100_set_mcast_block(emac, !enable, !enable), TAG, "set multicast block failed");
    emac->mcast_v4_cnt = 0;
    emac->mcast_v6_cnt = 0;
    if (enable) {
        ESP_LOGW(TAG, "W6100 filters in IP multicast frames only!");
    }
    return ESP_OK;
}
#endif // ESP_IDF_VERSION >= 6.0.0

static esp_err_t emac_w6100_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable)
{
    /* W6100 doesn't support flow control function */
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t emac_w6100_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t ability)
{
    /* W6100 doesn't support PAUSE function */
    return ESP_ERR_NOT_SUPPORTED;
}

static inline bool is_w6100_sane_for_rxtx(emac_w6100_t *emac)
{
    uint8_t physr;
    /* PHY is ok for rx and tx operations if LNK bit is set */
    if (w6100_read(emac, W6100_REG_PHYSR, &physr, 1) == ESP_OK && (physr & W6100_PHYSR_LNK)) {
        return true;
    }
    return false;
}

static esp_err_t emac_w6100_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    uint16_t offset = 0;

    ESP_GOTO_ON_FALSE(length <= ETH_MAX_PACKET_SIZE, ESP_ERR_INVALID_ARG, err,
                        TAG, "frame size is too big (actual %" PRIu32 ", maximum %u)", length, ETH_MAX_PACKET_SIZE);
    // check if there're free memory to store this packet
    uint16_t free_size = 0;
    ESP_GOTO_ON_ERROR(w6100_get_tx_free_size(emac, &free_size), err, TAG, "get free size failed");
    ESP_GOTO_ON_FALSE(length <= free_size, ESP_ERR_NO_MEM, err, TAG, "free size (%" PRIu16 ") < send length (%" PRIu32 ")", free_size, length);
    // get current write pointer
    ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_TX_WR(0), &offset, sizeof(offset)), err, TAG, "read TX WR failed");
    offset = __builtin_bswap16(offset);
    // copy data to tx memory
    ESP_GOTO_ON_ERROR(w6100_write_buffer(emac, buf, length, offset), err, TAG, "write frame failed");
    // update write pointer
    offset += length;
    offset = __builtin_bswap16(offset);
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_TX_WR(0), &offset, sizeof(offset)), err, TAG, "write TX WR failed");
    // issue SEND command
    ESP_GOTO_ON_ERROR(w6100_send_command(emac, W6100_SCR_SEND, 100), err, TAG, "issue SEND command failed");

    // polling the TX done event
    uint8_t status = 0;
    uint64_t start = esp_timer_get_time();
    uint64_t now = 0;
    do {
        now = esp_timer_get_time();
        if (!is_w6100_sane_for_rxtx(emac) || (now - start) > emac->tx_tmo) {
            return ESP_FAIL;
        }
        ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_IR(0), &status, sizeof(status)), err, TAG, "read SOCK0 IR failed");
    } while (!(status & W6100_SIR_SENDOK));
    // clear the event bit
    status = W6100_SIR_SENDOK;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_IRCLR(0), &status, sizeof(status)), err, TAG, "write SOCK0 IRCLR failed");

err:
    return ret;
}

static esp_err_t emac_w6100_alloc_recv_buf(emac_w6100_t *emac, uint8_t **buf, uint32_t *length)
{
    esp_err_t ret = ESP_OK;
    uint16_t offset = 0;
    uint16_t rx_len = 0;
    uint32_t copy_len = 0;
    uint16_t remain_bytes = 0;
    *buf = NULL;

    w6100_get_rx_received_size(emac, &remain_bytes);
    if (remain_bytes) {
        // get current read pointer
        ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_RX_RD(0), &offset, sizeof(offset)), err, TAG, "read RX RD failed");
        offset = __builtin_bswap16(offset);
        // read head
        ESP_GOTO_ON_ERROR(w6100_read_buffer(emac, &rx_len, sizeof(rx_len), offset), err, TAG, "read frame header failed");
        rx_len = __builtin_bswap16(rx_len) - 2; // data size includes 2 bytes of header
        // frames larger than expected will be truncated
        copy_len = rx_len > *length ? *length : rx_len;
        // runt frames are not forwarded by W6100, but check the length anyway since it could be corrupted at SPI bus
        ESP_GOTO_ON_FALSE(copy_len >= ETH_MIN_PACKET_SIZE - ETH_CRC_LEN, ESP_ERR_INVALID_SIZE, err, TAG, "invalid frame length %" PRIu32, copy_len);
        *buf = malloc(copy_len);
        if (*buf != NULL) {
            emac_w6100_auto_buf_info_t *buff_info = (emac_w6100_auto_buf_info_t *)*buf;
            buff_info->offset = offset;
            buff_info->copy_len = copy_len;
            buff_info->rx_len = rx_len;
            buff_info->remain = remain_bytes;
        } else {
            ret = ESP_ERR_NO_MEM;
            goto err;
        }
    }
err:
    *length = rx_len;
    return ret;
}

static esp_err_t emac_w6100_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    uint16_t offset = 0;
    uint16_t rx_len = 0;
    uint16_t copy_len = 0;
    uint16_t remain_bytes = 0;
    emac->packets_remain = false;

    if (*length != W6100_ETH_MAC_RX_BUF_SIZE_AUTO) {
        w6100_get_rx_received_size(emac, &remain_bytes);
        if (remain_bytes) {
            // get current read pointer
            ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_RX_RD(0), &offset, sizeof(offset)), err, TAG, "read RX RD failed");
            offset = __builtin_bswap16(offset);
            // read head first
            ESP_GOTO_ON_ERROR(w6100_read_buffer(emac, &rx_len, sizeof(rx_len), offset), err, TAG, "read frame header failed");
            rx_len = __builtin_bswap16(rx_len) - 2; // data size includes 2 bytes of header
            // frames larger than expected will be truncated
            copy_len = rx_len > *length ? *length : rx_len;
        } else {
            // silently return when no frame is waiting
            goto err;
        }
    } else {
        emac_w6100_auto_buf_info_t *buff_info = (emac_w6100_auto_buf_info_t *)buf;
        offset = buff_info->offset;
        copy_len = buff_info->copy_len;
        rx_len = buff_info->rx_len;
        remain_bytes = buff_info->remain;
    }
    // 2 bytes of header
    offset += 2;
    // read the payload
    ESP_GOTO_ON_ERROR(w6100_read_buffer(emac, emac->rx_buffer, copy_len, offset), err, TAG, "read payload failed, len=%" PRIu16 ", offset=%" PRIu16, rx_len, offset);
    memcpy(buf, emac->rx_buffer, copy_len);
    offset += rx_len;
    // update read pointer
    offset = __builtin_bswap16(offset);
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_RX_RD(0), &offset, sizeof(offset)), err, TAG, "write RX RD failed");
    /* issue RECV command */
    ESP_GOTO_ON_ERROR(w6100_send_command(emac, W6100_SCR_RECV, 100), err, TAG, "issue RECV command failed");
    // check if there're more data need to process
    remain_bytes -= rx_len + 2;
    emac->packets_remain = remain_bytes > 0;

    *length = copy_len;
    return ret;
err:
    *length = 0;
    return ret;
}

static esp_err_t emac_w6100_flush_recv_frame(emac_w6100_t *emac)
{
    esp_err_t ret = ESP_OK;
    uint16_t offset = 0;
    uint16_t rx_len = 0;
    uint16_t remain_bytes = 0;
    emac->packets_remain = false;

    w6100_get_rx_received_size(emac, &remain_bytes);
    if (remain_bytes) {
        // get current read pointer
        ESP_GOTO_ON_ERROR(w6100_read(emac, W6100_REG_SOCK_RX_RD(0), &offset, sizeof(offset)), err, TAG, "read RX RD failed");
        offset = __builtin_bswap16(offset);
        // read head first
        ESP_GOTO_ON_ERROR(w6100_read_buffer(emac, &rx_len, sizeof(rx_len), offset), err, TAG, "read frame header failed");
        // update read pointer
        rx_len = __builtin_bswap16(rx_len);
        offset += rx_len;
        offset = __builtin_bswap16(offset);
        ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SOCK_RX_RD(0), &offset, sizeof(offset)), err, TAG, "write RX RD failed");
        /* issue RECV command */
        ESP_GOTO_ON_ERROR(w6100_send_command(emac, W6100_SCR_RECV, 100), err, TAG, "issue RECV command failed");
        // check if there're more data need to process
        remain_bytes -= rx_len;
        emac->packets_remain = remain_bytes > 0;
    }
err:
    return ret;
}

IRAM_ATTR static void w6100_isr_handler(void *arg)
{
    emac_w6100_t *emac = (emac_w6100_t *)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    /* notify w6100 task */
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

static void w6100_poll_timer(void *arg)
{
    emac_w6100_t *emac = (emac_w6100_t *)arg;
    xTaskNotifyGive(emac->rx_task_hdl);
}

static void emac_w6100_task(void *arg)
{
    emac_w6100_t *emac = (emac_w6100_t *)arg;
    uint8_t status = 0;
    uint8_t *buffer = NULL;
    uint32_t frame_len = 0;
    uint32_t buf_len = 0;
    esp_err_t ret;
    while (1) {
        /* check if the task receives any notification */
        if (emac->int_gpio_num >= 0) {                                   // if in interrupt mode
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0 &&   // if no notification ...
                gpio_get_level(emac->int_gpio_num) != 0) {               // ...and no interrupt asserted
                continue;                                                // -> just continue to check again
            }
        } else {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        /* read interrupt status */
        w6100_read(emac, W6100_REG_SOCK_IR(0), &status, sizeof(status));
        /* packet received */
        if (status & W6100_SIR_RECV) {
            /* clear interrupt status */
            uint8_t clr = W6100_SIR_RECV;
            w6100_write(emac, W6100_REG_SOCK_IRCLR(0), &clr, sizeof(clr));
            do {
                /* define max expected frame len */
                frame_len = ETH_MAX_PACKET_SIZE;
                if ((ret = emac_w6100_alloc_recv_buf(emac, &buffer, &frame_len)) == ESP_OK) {
                    if (buffer != NULL) {
                        /* we have memory to receive the frame of maximal size previously defined */
                        buf_len = W6100_ETH_MAC_RX_BUF_SIZE_AUTO;
                        if (emac->parent.receive(&emac->parent, buffer, &buf_len) == ESP_OK) {
                            if (buf_len == 0) {
                                free(buffer);
                            } else if (frame_len > buf_len) {
                                ESP_LOGE(TAG, "received frame was truncated");
                                free(buffer);
                            } else {
                                ESP_LOGD(TAG, "receive len=%" PRIu32, buf_len);
                                /* pass the buffer to stack (e.g. TCP/IP layer) */
                                emac->eth->stack_input(emac->eth, buffer, buf_len);
                            }
                        } else {
                            ESP_LOGE(TAG, "frame read from module failed");
                            free(buffer);
                        }
                    } else if (frame_len) {
                        ESP_LOGE(TAG, "invalid combination of frame_len(%" PRIu32 ") and buffer pointer(%p)", frame_len, buffer);
                    }
                } else if (ret == ESP_ERR_NO_MEM) {
                    ESP_LOGE(TAG, "no mem for receive buffer");
                    emac_w6100_flush_recv_frame(emac);
                } else {
                    ESP_LOGE(TAG, "unexpected error 0x%x", ret);
                }
            } while (emac->packets_remain);
        }
    }
    vTaskDelete(NULL);
}

static esp_err_t emac_w6100_init(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    if (emac->int_gpio_num >= 0) {
        esp_rom_gpio_pad_select_gpio(emac->int_gpio_num);
        gpio_set_direction(emac->int_gpio_num, GPIO_MODE_INPUT);
        gpio_set_pull_mode(emac->int_gpio_num, GPIO_PULLUP_ONLY);
        gpio_set_intr_type(emac->int_gpio_num, GPIO_INTR_NEGEDGE); // active low
        gpio_intr_enable(emac->int_gpio_num);
        gpio_isr_handler_add(emac->int_gpio_num, w6100_isr_handler, emac);
    }
    ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL), err, TAG, "lowlevel init failed");
    /* reset w6100 */
    ESP_GOTO_ON_ERROR(w6100_reset(emac), err, TAG, "reset w6100 failed");
    /* verify chip id */
    ESP_GOTO_ON_ERROR(w6100_verify_id(emac), err, TAG, "verify chip ID failed");
    /* default setup of internal registers */
    ESP_GOTO_ON_ERROR(w6100_setup_default(emac), err, TAG, "w6100 default setup failed");
    /* set MAC address to SHAR register */
    ESP_GOTO_ON_ERROR(w6100_set_mac_addr(emac), err, TAG, "set mac address failed");
    return ESP_OK;
err:
    if (emac->int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->int_gpio_num);
        gpio_reset_pin(emac->int_gpio_num);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ret;
}

static esp_err_t emac_w6100_deinit(esp_eth_mac_t *mac)
{
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    mac->stop(mac);
    if (emac->int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->int_gpio_num);
        gpio_reset_pin(emac->int_gpio_num);
    }
    if (emac->poll_timer && esp_timer_is_active(emac->poll_timer)) {
        esp_timer_stop(emac->poll_timer);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ESP_OK;
}

static esp_err_t emac_w6100_del(esp_eth_mac_t *mac)
{
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, parent);
    if (emac->poll_timer) {
        esp_timer_delete(emac->poll_timer);
    }
    vTaskDelete(emac->rx_task_hdl);
    emac->spi.deinit(emac->spi.ctx);
    heap_caps_free(emac->rx_buffer);
    free(emac);
    return ESP_OK;
}

esp_eth_mac_t *esp_eth_mac_new_w6100(const eth_w6100_config_t *w6100_config, const eth_mac_config_t *mac_config)
{
    esp_eth_mac_t *ret = NULL;
    emac_w6100_t *emac = NULL;
    ESP_GOTO_ON_FALSE(w6100_config && mac_config, NULL, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE((w6100_config->int_gpio_num >= 0) != (w6100_config->poll_period_ms > 0), NULL, err, TAG, "invalid configuration argument combination");
    emac = calloc(1, sizeof(emac_w6100_t));
    ESP_GOTO_ON_FALSE(emac, NULL, err, TAG, "no mem for MAC instance");
    /* bind methods and attributes */
    emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
    emac->tx_tmo = W6100_100M_TX_TMO_US;  // default to 100Mbps timeout
    emac->int_gpio_num = w6100_config->int_gpio_num;
    emac->poll_period_ms = w6100_config->poll_period_ms;
    emac->parent.set_mediator = emac_w6100_set_mediator;
    emac->parent.init = emac_w6100_init;
    emac->parent.deinit = emac_w6100_deinit;
    emac->parent.start = emac_w6100_start;
    emac->parent.stop = emac_w6100_stop;
    emac->parent.del = emac_w6100_del;
    emac->parent.write_phy_reg = emac_w6100_write_phy_reg;
    emac->parent.read_phy_reg = emac_w6100_read_phy_reg;
    emac->parent.set_addr = emac_w6100_set_addr;
    emac->parent.get_addr = emac_w6100_get_addr;
    emac->parent.set_speed = emac_w6100_set_speed;
    emac->parent.set_duplex = emac_w6100_set_duplex;
    emac->parent.set_link = emac_w6100_set_link;
    emac->parent.set_promiscuous = emac_w6100_set_promiscuous;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    emac->parent.add_mac_filter = emac_w6100_add_mac_filter;
    emac->parent.rm_mac_filter = emac_w6100_rm_mac_filter;
    emac->parent.set_all_multicast = emac_w6100_set_all_multicast;
#endif
    emac->parent.set_peer_pause_ability = emac_w6100_set_peer_pause_ability;
    emac->parent.enable_flow_ctrl = emac_w6100_enable_flow_ctrl;
    emac->parent.transmit = emac_w6100_transmit;
    emac->parent.receive = emac_w6100_receive;

    if (w6100_config->custom_spi_driver.init != NULL && w6100_config->custom_spi_driver.deinit != NULL
            && w6100_config->custom_spi_driver.read != NULL && w6100_config->custom_spi_driver.write != NULL) {
        ESP_LOGD(TAG, "Using user's custom SPI Driver");
        emac->spi.init = w6100_config->custom_spi_driver.init;
        emac->spi.deinit = w6100_config->custom_spi_driver.deinit;
        emac->spi.read = w6100_config->custom_spi_driver.read;
        emac->spi.write = w6100_config->custom_spi_driver.write;
        /* Custom SPI driver device init */
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(w6100_config->custom_spi_driver.config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    } else {
        ESP_LOGD(TAG, "Using default SPI Driver");
        emac->spi.init = w6100_spi_init;
        emac->spi.deinit = w6100_spi_deinit;
        emac->spi.read = w6100_spi_read;
        emac->spi.write = w6100_spi_write;
        /* SPI device init */
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(w6100_config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    }

    /* create w6100 task */
    BaseType_t core_num = tskNO_AFFINITY;
    if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
        core_num = esp_cpu_get_core_id();
    }
    BaseType_t xReturned = xTaskCreatePinnedToCore(emac_w6100_task, "w6100_tsk", mac_config->rx_task_stack_size, emac,
                           mac_config->rx_task_prio, &emac->rx_task_hdl, core_num);
    ESP_GOTO_ON_FALSE(xReturned == pdPASS, NULL, err, TAG, "create w6100 task failed");

    emac->rx_buffer = heap_caps_malloc(ETH_MAX_PACKET_SIZE, MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(emac->rx_buffer, NULL, err, TAG, "RX buffer allocation failed");

    if (emac->int_gpio_num < 0) {
        const esp_timer_create_args_t poll_timer_args = {
            .callback = w6100_poll_timer,
            .name = "emac_spi_poll_timer",
            .arg = emac,
            .skip_unhandled_events = true
        };
        ESP_GOTO_ON_FALSE(esp_timer_create(&poll_timer_args, &emac->poll_timer) == ESP_OK, NULL, err, TAG, "create poll timer failed");
    }

    return &(emac->parent);

err:
    if (emac) {
        if (emac->poll_timer) {
            esp_timer_delete(emac->poll_timer);
        }
        if (emac->rx_task_hdl) {
            vTaskDelete(emac->rx_task_hdl);
        }
        if (emac->spi.ctx) {
            emac->spi.deinit(emac->spi.ctx);
        }
        heap_caps_free(emac->rx_buffer);
        free(emac);
    }
    return ret;
}
