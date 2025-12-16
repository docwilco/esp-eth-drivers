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
#include "wiznet_spi.h"
#include "wiznet_mac_common.h"
#include "w6100.h"

static const char *TAG = "w6100.mac";

#define W6100_100M_TX_TMO_US (200)
#define W6100_10M_TX_TMO_US (1500)

typedef struct {
    emac_wiznet_t base;  // Must be first member for safe casting
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    uint8_t mcast_v4_cnt;
    uint8_t mcast_v6_cnt;
#endif
} emac_w6100_t;

/*******************************************************************************
 * Chip-specific ops for W6100
 ******************************************************************************/

static bool is_w6100_sane_for_rxtx(emac_wiznet_t *emac);

static const wiznet_chip_ops_t w6100_ops = {
    /* Socket 0 registers (pre-computed addresses) */
    .reg_sock_cr = W6100_REG_SOCK_CR(0),
    .reg_sock_ir = W6100_REG_SOCK_IR(0),
    .reg_sock_tx_fsr = W6100_REG_SOCK_TX_FSR(0),
    .reg_sock_tx_wr = W6100_REG_SOCK_TX_WR(0),
    .reg_sock_rx_rsr = W6100_REG_SOCK_RX_RSR(0),
    .reg_sock_rx_rd = W6100_REG_SOCK_RX_RD(0),
    .reg_simr = W6100_REG_SIMR,

    /* Memory base addresses (offset added at runtime) */
    .mem_sock_tx_base = W6100_MEM_SOCK_TX(0, 0),
    .mem_sock_rx_base = W6100_MEM_SOCK_RX(0, 0),

    /* W6100 uses separate IRCLR register to clear interrupts */
    .reg_sock_irclr = W6100_REG_SOCK_IRCLR(0),

    /* Command values */
    .cmd_send = W6100_SCR_SEND,
    .cmd_recv = W6100_SCR_RECV,

    /* Interrupt bits */
    .sir_send = W6100_SIR_SENDOK,
    .sir_recv = W6100_SIR_RECV,
    .simr_sock0 = W6100_SIMR_SOCK0,

    /* Callbacks */
    .is_sane_for_rxtx = is_w6100_sane_for_rxtx,
};

static bool is_w6100_sane_for_rxtx(emac_wiznet_t *emac)
{
    uint8_t physr;
    /* PHY is ok for rx and tx operations if LNK bit is set */
    if (emac->spi.read(emac->spi.ctx, (W6100_REG_PHYSR >> 16), (W6100_REG_PHYSR & 0xFFFF), &physr, 1) == ESP_OK 
        && (physr & W6100_PHYSR_LNK)) {
        return true;
    }
    return false;
}

static esp_err_t w6100_read(emac_w6100_t *emac, uint32_t address, void *data, uint32_t len)
{
    uint32_t cmd = (address >> W6100_ADDR_OFFSET); // Address phase in W6100 SPI frame
    uint32_t addr = ((address & 0xFFFF) | (W6100_ACCESS_MODE_READ << W6100_RWB_OFFSET)
                    | W6100_SPI_OP_MODE_VDM); // Control phase in W6100 SPI frame

    return emac->base.spi.read(emac->base.spi.ctx, cmd, addr, data, len);
}

static esp_err_t w6100_write(emac_w6100_t *emac, uint32_t address, const void *data, uint32_t len)
{
    uint32_t cmd = (address >> W6100_ADDR_OFFSET); // Address phase in W6100 SPI frame
    uint32_t addr = ((address & 0xFFFF) | (W6100_ACCESS_MODE_WRITE << W6100_RWB_OFFSET)
                    | W6100_SPI_OP_MODE_VDM); // Control phase in W6100 SPI frame

    return emac->base.spi.write(emac->base.spi.ctx, cmd, addr, data, len);
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

static esp_err_t w6100_set_mac_addr(emac_w6100_t *emac)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SHAR, emac->base.addr, 6), err, TAG, "write MAC address register failed");

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
    for (to = 0; to < emac->base.sw_reset_timeout_ms / 10; to++) {
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
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, base.parent);
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
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, base.parent);
    uint8_t reg_value = 0;
    /* disable interrupt */
    ESP_GOTO_ON_ERROR(w6100_write(emac, W6100_REG_SIMR, &reg_value, sizeof(reg_value)), err, TAG, "write SIMR failed");
    /* close SOCK0 */
    ESP_GOTO_ON_ERROR(w6100_send_command(emac, W6100_SCR_CLOSE, 100), err, TAG, "issue CLOSE command failed");

err:
    return ret;
}

static esp_err_t emac_w6100_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, base.parent);
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
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, base.parent);
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
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, base.parent);
    memcpy(emac->base.addr, addr, 6);
    ESP_GOTO_ON_ERROR(w6100_set_mac_addr(emac), err, TAG, "set mac address failed");

err:
    return ret;
}

static esp_err_t emac_w6100_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, base.parent);
    switch (speed) {
    case ETH_SPEED_10M:
        emac->base.tx_tmo = W6100_10M_TX_TMO_US;
        ESP_LOGD(TAG, "working in 10Mbps");
        break;
    case ETH_SPEED_100M:
        emac->base.tx_tmo = W6100_100M_TX_TMO_US;
        ESP_LOGD(TAG, "working in 100Mbps");
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unknown speed");
        break;
    }
err:
    return ret;
}

static esp_err_t emac_w6100_set_promiscuous(esp_eth_mac_t *mac, bool enable)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, base.parent);
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
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, base.parent);
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
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, base.parent);
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
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, base.parent);
    ESP_RETURN_ON_ERROR(emac_w6100_set_mcast_block(emac, !enable, !enable), TAG, "set multicast block failed");
    emac->mcast_v4_cnt = 0;
    emac->mcast_v6_cnt = 0;
    if (enable) {
        ESP_LOGW(TAG, "W6100 filters in IP multicast frames only!");
    }
    return ESP_OK;
}
#endif // ESP_IDF_VERSION >= 6.0.0

static esp_err_t emac_w6100_init(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_w6100_t *emac = __containerof(mac, emac_w6100_t, base.parent);
    esp_eth_mediator_t *eth = emac->base.eth;
    if (emac->base.int_gpio_num >= 0) {
        esp_rom_gpio_pad_select_gpio(emac->base.int_gpio_num);
        gpio_set_direction(emac->base.int_gpio_num, GPIO_MODE_INPUT);
        gpio_set_pull_mode(emac->base.int_gpio_num, GPIO_PULLUP_ONLY);
        gpio_set_intr_type(emac->base.int_gpio_num, GPIO_INTR_NEGEDGE); // active low
        gpio_intr_enable(emac->base.int_gpio_num);
        gpio_isr_handler_add(emac->base.int_gpio_num, wiznet_isr_handler, emac);
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
    if (emac->base.int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->base.int_gpio_num);
        gpio_reset_pin(emac->base.int_gpio_num);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ret;
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
    emac->base.tag = TAG;
    emac->base.ops = &w6100_ops;
    emac->base.sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
    emac->base.tx_tmo = W6100_100M_TX_TMO_US;  // default to 100Mbps timeout
    emac->base.int_gpio_num = w6100_config->int_gpio_num;
    emac->base.poll_period_ms = w6100_config->poll_period_ms;
    emac->base.parent.set_mediator = emac_wiznet_set_mediator;
    emac->base.parent.init = emac_w6100_init;
    emac->base.parent.deinit = emac_wiznet_deinit;
    emac->base.parent.start = emac_w6100_start;
    emac->base.parent.stop = emac_w6100_stop;
    emac->base.parent.del = emac_wiznet_del;
    emac->base.parent.write_phy_reg = emac_w6100_write_phy_reg;
    emac->base.parent.read_phy_reg = emac_w6100_read_phy_reg;
    emac->base.parent.set_addr = emac_w6100_set_addr;
    emac->base.parent.get_addr = emac_wiznet_get_addr;
    emac->base.parent.set_speed = emac_w6100_set_speed;
    emac->base.parent.set_duplex = emac_wiznet_set_duplex;
    emac->base.parent.set_link = emac_wiznet_set_link;
    emac->base.parent.set_promiscuous = emac_w6100_set_promiscuous;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    emac->base.parent.add_mac_filter = emac_w6100_add_mac_filter;
    emac->base.parent.rm_mac_filter = emac_w6100_rm_mac_filter;
    emac->base.parent.set_all_multicast = emac_w6100_set_all_multicast;
#endif
    emac->base.parent.set_peer_pause_ability = emac_wiznet_set_peer_pause_ability;
    emac->base.parent.enable_flow_ctrl = emac_wiznet_enable_flow_ctrl;
    emac->base.parent.transmit = emac_wiznet_transmit;
    emac->base.parent.receive = emac_wiznet_receive;

    if (w6100_config->custom_spi_driver.init != NULL && w6100_config->custom_spi_driver.deinit != NULL
            && w6100_config->custom_spi_driver.read != NULL && w6100_config->custom_spi_driver.write != NULL) {
        ESP_LOGD(TAG, "Using user's custom SPI Driver");
        emac->base.spi.init = w6100_config->custom_spi_driver.init;
        emac->base.spi.deinit = w6100_config->custom_spi_driver.deinit;
        emac->base.spi.read = w6100_config->custom_spi_driver.read;
        emac->base.spi.write = w6100_config->custom_spi_driver.write;
        /* Custom SPI driver device init */
        ESP_GOTO_ON_FALSE((emac->base.spi.ctx = emac->base.spi.init(w6100_config->custom_spi_driver.config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    } else {
        ESP_LOGD(TAG, "Using default SPI Driver");
        emac->base.spi.init = wiznet_spi_init;
        emac->base.spi.deinit = wiznet_spi_deinit;
        emac->base.spi.read = wiznet_spi_read;
        emac->base.spi.write = wiznet_spi_write;
        /* SPI device init */
        ESP_GOTO_ON_FALSE((emac->base.spi.ctx = emac->base.spi.init(w6100_config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    }

    /* create w6100 task */
    BaseType_t core_num = tskNO_AFFINITY;
    if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
        core_num = esp_cpu_get_core_id();
    }
    BaseType_t xReturned = xTaskCreatePinnedToCore(emac_wiznet_task, "w6100_tsk", mac_config->rx_task_stack_size, emac,
                           mac_config->rx_task_prio, &emac->base.rx_task_hdl, core_num);
    ESP_GOTO_ON_FALSE(xReturned == pdPASS, NULL, err, TAG, "create w6100 task failed");

    emac->base.rx_buffer = heap_caps_malloc(ETH_MAX_PACKET_SIZE, MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(emac->base.rx_buffer, NULL, err, TAG, "RX buffer allocation failed");

    if (emac->base.int_gpio_num < 0) {
        const esp_timer_create_args_t poll_timer_args = {
            .callback = wiznet_poll_timer,
            .name = "emac_spi_poll_timer",
            .arg = emac,
            .skip_unhandled_events = true
        };
        ESP_GOTO_ON_FALSE(esp_timer_create(&poll_timer_args, &emac->base.poll_timer) == ESP_OK, NULL, err, TAG, "create poll timer failed");
    }

    return &(emac->base.parent);

err:
    if (emac) {
        if (emac->base.poll_timer) {
            esp_timer_delete(emac->base.poll_timer);
        }
        if (emac->base.rx_task_hdl) {
            vTaskDelete(emac->base.rx_task_hdl);
        }
        if (emac->base.spi.ctx) {
            emac->base.spi.deinit(emac->base.spi.ctx);
        }
        heap_caps_free(emac->base.rx_buffer);
        free(emac);
    }
    return ret;
}
