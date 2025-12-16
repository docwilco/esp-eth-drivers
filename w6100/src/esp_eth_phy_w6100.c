/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_eth_phy.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "w6100.h"

#define W6100_WAIT_FOR_RESET_MS (10) // wait for W6100 internal PHY after reset

static const char *TAG = "w6100.phy";

/***************Vendor Specific Register***************/
/**
 * @brief PHYSR (PHY Status Register) bit definitions
 *
 * W6100 PHYSR bit layout:
 * - Bit 7: CAB (Cable Off, 1=unplugged)
 * - Bits 5:3: MODE (Operation mode)
 * - Bit 2: DPX (Duplex, 1=half, 0=full) - NOTE: Inverted from W5500!
 * - Bit 1: SPD (Speed, 1=10Mbps, 0=100Mbps) - NOTE: Inverted from W5500!
 * - Bit 0: LNK (Link, 1=up, 0=down)
 */
typedef union {
    struct {
        uint8_t link: 1;   /*!< Link status (1=up, 0=down) */
        uint8_t speed: 1;  /*!< Speed status (1=10M, 0=100M) - INVERTED from W5500 */
        uint8_t duplex: 1; /*!< Duplex status (1=half, 0=full) - INVERTED from W5500 */
        uint8_t opmode: 3; /*!< Operation mode */
        uint8_t reserved: 1;
        uint8_t cab: 1;    /*!< Cable off (1=unplugged) */
    };
    uint8_t val;
} physr_reg_t;

/**
 * @brief PHYCR0 (PHY Control Register 0) operation modes
 */
typedef enum {
    W6100_OP_MODE_AUTO = 0x00,          /*!< Auto negotiation */
    W6100_OP_MODE_100BT_FULL = 0x04,    /*!< 100BASE-TX Full Duplex */
    W6100_OP_MODE_100BT_HALF = 0x05,    /*!< 100BASE-TX Half Duplex */
    W6100_OP_MODE_10BT_FULL = 0x06,     /*!< 10BASE-T Full Duplex */
    W6100_OP_MODE_10BT_HALF = 0x07,     /*!< 10BASE-T Half Duplex */
} phy_w6100_op_mode_e;

/**
 * @brief PHYCR1 (PHY Control Register 1) bit definitions
 */
typedef union {
    struct {
        uint8_t reset: 1;    /*!< PHY Reset (write 1 to reset) */
        uint8_t reserved1: 2;
        uint8_t te: 1;       /*!< 10BASE-Te mode */
        uint8_t reserved2: 1;
        uint8_t pwdn: 1;     /*!< Power Down */
        uint8_t reserved3: 2;
    };
    uint8_t val;
} phycr1_reg_t;

typedef struct {
    esp_eth_phy_t parent;
    esp_eth_mediator_t *eth;
    int addr;
    uint32_t reset_timeout_ms;
    uint32_t autonego_timeout_ms;
    eth_link_t link_status;
    int reset_gpio_num;
} phy_w6100_t;

static esp_err_t w6100_update_link_duplex_speed(phy_w6100_t *w6100)
{
    esp_err_t ret = ESP_OK;
    esp_eth_mediator_t *eth = w6100->eth;
    eth_speed_t speed = ETH_SPEED_10M;
    eth_duplex_t duplex = ETH_DUPLEX_HALF;
    physr_reg_t physr;

    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYSR, (uint32_t *) &(physr.val)), err, TAG, "read PHYSR failed");
    eth_link_t link = physr.link ? ETH_LINK_UP : ETH_LINK_DOWN;
    /* check if link status changed */
    if (w6100->link_status != link) {
        /* when link up, read negotiation result */
        if (link == ETH_LINK_UP) {
            /* W6100 PHYSR bit semantics are inverted from W5500:
             * - SPD: 1=10Mbps, 0=100Mbps (W5500: 1=100Mbps)
             * - DPX: 1=half, 0=full (W5500: 1=full)
             */
            if (physr.speed) {
                speed = ETH_SPEED_10M;
            } else {
                speed = ETH_SPEED_100M;
            }
            if (physr.duplex) {
                duplex = ETH_DUPLEX_HALF;
            } else {
                duplex = ETH_DUPLEX_FULL;
            }
            ESP_LOGI(TAG, "Link Up: %s %s", 
                     speed == ETH_SPEED_100M ? "100Mbps" : "10Mbps",
                     duplex == ETH_DUPLEX_FULL ? "Full-Duplex" : "Half-Duplex");
            ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_SPEED, (void *)speed), err, TAG, "change speed failed");
            ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_DUPLEX, (void *)duplex), err, TAG, "change duplex failed");
        }
        ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LINK, (void *)link), err, TAG, "change link failed");
        w6100->link_status = link;
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t w6100_set_mediator(esp_eth_phy_t *phy, esp_eth_mediator_t *eth)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(eth, ESP_ERR_INVALID_ARG, err, TAG, "mediator can't be null");
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    w6100->eth = eth;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t w6100_get_link(esp_eth_phy_t *phy)
{
    esp_err_t ret = ESP_OK;
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    /* Update information about link, speed, duplex */
    ESP_GOTO_ON_ERROR(w6100_update_link_duplex_speed(w6100), err, TAG, "update link duplex speed failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t w6100_set_link(esp_eth_phy_t *phy, eth_link_t link)
{
    esp_err_t ret = ESP_OK;
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    esp_eth_mediator_t *eth = w6100->eth;

    if (w6100->link_status != link) {
        w6100->link_status = link;
        // link status changed, immediately report to upper layers
        ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LINK, (void *)w6100->link_status), err, TAG, "change link failed");
    }
err:
    return ret;
}

static esp_err_t w6100_reset(esp_eth_phy_t *phy)
{
    esp_err_t ret = ESP_OK;
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    w6100->link_status = ETH_LINK_DOWN;
    esp_eth_mediator_t *eth = w6100->eth;

    // Unlock PHY configuration
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYLCKR, W6100_PHYLCKR_UNLOCK), err, TAG, "unlock PHY failed");

    // Reset PHY by setting reset bit in PHYCR1
    phycr1_reg_t phycr1;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYCR1, (uint32_t *) &(phycr1.val)), err, TAG, "read PHYCR1 failed");
    phycr1.reset = 1;
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR1, phycr1.val), err, TAG, "write PHYCR1 failed");

    vTaskDelay(pdMS_TO_TICKS(W6100_WAIT_FOR_RESET_MS));

    phycr1.reset = 0;
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR1, phycr1.val), err, TAG, "write PHYCR1 failed");

    return ESP_OK;
err:
    return ret;
}

static esp_err_t w6100_reset_hw(esp_eth_phy_t *phy)
{
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    // set reset_gpio_num to a negative value can skip hardware reset phy chip
    if (w6100->reset_gpio_num >= 0) {
        esp_rom_gpio_pad_select_gpio(w6100->reset_gpio_num);
        gpio_set_direction(w6100->reset_gpio_num, GPIO_MODE_OUTPUT);
        gpio_set_level(w6100->reset_gpio_num, 0);
        esp_rom_delay_us(100); // insert min input assert time
        gpio_set_level(w6100->reset_gpio_num, 1);
    }
    return ESP_OK;
}

static esp_err_t w6100_autonego_ctrl(esp_eth_phy_t *phy, eth_phy_autoneg_cmd_t cmd, bool *autonego_en_stat)
{
    esp_err_t ret = ESP_OK;
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    esp_eth_mediator_t *eth = w6100->eth;

    uint32_t phycr0;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYCR0, &phycr0), err, TAG, "read PHYCR0 failed");

    // Unlock PHY configuration for mode changes
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYLCKR, W6100_PHYLCKR_UNLOCK), err, TAG, "unlock PHY failed");

    switch (cmd) {
    case ESP_ETH_PHY_AUTONEGO_RESTART:
        ESP_GOTO_ON_FALSE(phycr0 == W6100_OP_MODE_AUTO,
                          ESP_ERR_INVALID_STATE, err, TAG, "auto negotiation is disabled");
        /* in case any link status has changed, let's assume we're in link down status */
        w6100->link_status = ETH_LINK_DOWN;

        // Reset PHY to restart auto-negotiation
        {
            phycr1_reg_t phycr1;
            ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYCR1, (uint32_t *) &(phycr1.val)), err, TAG, "read PHYCR1 failed");
            phycr1.reset = 1;
            ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR1, phycr1.val), err, TAG, "write PHYCR1 failed");
            vTaskDelay(pdMS_TO_TICKS(W6100_WAIT_FOR_RESET_MS));
            phycr1.reset = 0;
            ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR1, phycr1.val), err, TAG, "write PHYCR1 failed");
        }
        *autonego_en_stat = true;
        break;
    case ESP_ETH_PHY_AUTONEGO_DIS:
        /* W6100 autonegotiation cannot be separately disabled, only specific speed/duplex mode needs to be configured.
           Hence set the last used configuration based on current PHYSR */
        {
            physr_reg_t physr;
            ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYSR, (uint32_t *) &(physr.val)), err, TAG, "read PHYSR failed");

            // W6100: duplex=1 means half, speed=1 means 10M
            if (physr.duplex == 0) { // Full duplex
                if (physr.speed == 0) { // 100 Mbps
                    phycr0 = W6100_OP_MODE_100BT_FULL;
                } else {
                    phycr0 = W6100_OP_MODE_10BT_FULL;
                }
            } else { // Half duplex
                if (physr.speed == 0) { // 100 Mbps
                    phycr0 = W6100_OP_MODE_100BT_HALF;
                } else {
                    phycr0 = W6100_OP_MODE_10BT_HALF;
                }
            }
            ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR0, phycr0), err, TAG, "write PHYCR0 failed");
        }
        *autonego_en_stat = false;
        break;
    case ESP_ETH_PHY_AUTONEGO_EN:
        phycr0 = W6100_OP_MODE_AUTO;
        ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR0, phycr0), err, TAG, "write PHYCR0 failed");
        *autonego_en_stat = true;
        break;
    case ESP_ETH_PHY_AUTONEGO_G_STAT:
        *autonego_en_stat = (phycr0 == W6100_OP_MODE_AUTO);
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
err:
    return ret;
}

static esp_err_t w6100_pwrctl(esp_eth_phy_t *phy, bool enable)
{
    esp_err_t ret = ESP_OK;
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    esp_eth_mediator_t *eth = w6100->eth;

    // Unlock PHY configuration
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYLCKR, W6100_PHYLCKR_UNLOCK), err, TAG, "unlock PHY failed");

    phycr1_reg_t phycr1;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYCR1, (uint32_t *) &(phycr1.val)), err, TAG, "read PHYCR1 failed");
    phycr1.pwdn = enable ? 0 : 1;  // 0 = power on, 1 = power down
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR1, phycr1.val), err, TAG, "write PHYCR1 failed");

    if (enable) {
        // Wait for PHY to power up
        vTaskDelay(pdMS_TO_TICKS(W6100_WAIT_FOR_RESET_MS));
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t w6100_set_addr(esp_eth_phy_t *phy, uint32_t addr)
{
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    w6100->addr = addr;
    return ESP_OK;
}

static esp_err_t w6100_get_addr(esp_eth_phy_t *phy, uint32_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "addr can't be null");
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    *addr = w6100->addr;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t w6100_del(esp_eth_phy_t *phy)
{
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    free(w6100);
    return ESP_OK;
}

static esp_err_t w6100_advertise_pause_ability(esp_eth_phy_t *phy, uint32_t ability)
{
    // pause ability advertisement is not supported for W6100 internal PHY
    return ESP_OK;
}

static esp_err_t w6100_loopback(esp_eth_phy_t *phy, bool enable)
{
    // Loopback is not supported for W6100 internal PHY
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t w6100_set_speed(esp_eth_phy_t *phy, eth_speed_t speed)
{
    esp_err_t ret = ESP_OK;
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    esp_eth_mediator_t *eth = w6100->eth;

    /* Since the link is going to be reconfigured, consider it down to be status updated once the driver re-started */
    w6100->link_status = ETH_LINK_DOWN;

    // Unlock PHY configuration
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYLCKR, W6100_PHYLCKR_UNLOCK), err, TAG, "unlock PHY failed");

    physr_reg_t physr;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYSR, (uint32_t *) &(physr.val)), err, TAG, "read PHYSR failed");

    uint32_t phycr0;
    // W6100: duplex=1 means half, duplex=0 means full
    if (physr.duplex == 0) { // Full duplex
        if (speed == ETH_SPEED_100M) {
            phycr0 = W6100_OP_MODE_100BT_FULL;
        } else {
            phycr0 = W6100_OP_MODE_10BT_FULL;
        }
    } else { // Half duplex
        if (speed == ETH_SPEED_100M) {
            phycr0 = W6100_OP_MODE_100BT_HALF;
        } else {
            phycr0 = W6100_OP_MODE_10BT_HALF;
        }
    }
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR0, phycr0), err, TAG, "write PHYCR0 failed");

    // Reset PHY for configuration to take effect
    {
        phycr1_reg_t phycr1;
        ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYCR1, (uint32_t *) &(phycr1.val)), err, TAG, "read PHYCR1 failed");
        phycr1.reset = 1;
        ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR1, phycr1.val), err, TAG, "write PHYCR1 failed");
        vTaskDelay(pdMS_TO_TICKS(W6100_WAIT_FOR_RESET_MS));
        phycr1.reset = 0;
        ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR1, phycr1.val), err, TAG, "write PHYCR1 failed");
    }

err:
    return ret;
}

static esp_err_t w6100_set_duplex(esp_eth_phy_t *phy, eth_duplex_t duplex)
{
    esp_err_t ret = ESP_OK;
    phy_w6100_t *w6100 = __containerof(phy, phy_w6100_t, parent);
    esp_eth_mediator_t *eth = w6100->eth;

    /* Since the link is going to be reconfigured, consider it down to be status updated once the driver re-started */
    w6100->link_status = ETH_LINK_DOWN;

    // Unlock PHY configuration
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYLCKR, W6100_PHYLCKR_UNLOCK), err, TAG, "unlock PHY failed");

    physr_reg_t physr;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYSR, (uint32_t *) &(physr.val)), err, TAG, "read PHYSR failed");

    uint32_t phycr0;
    // W6100: speed=1 means 10M, speed=0 means 100M
    if (physr.speed == 0) { // 100Mbps
        if (duplex == ETH_DUPLEX_FULL) {
            phycr0 = W6100_OP_MODE_100BT_FULL;
        } else {
            phycr0 = W6100_OP_MODE_100BT_HALF;
        }
    } else { // 10Mbps
        if (duplex == ETH_DUPLEX_FULL) {
            phycr0 = W6100_OP_MODE_10BT_FULL;
        } else {
            phycr0 = W6100_OP_MODE_10BT_HALF;
        }
    }
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR0, phycr0), err, TAG, "write PHYCR0 failed");

    // Reset PHY for configuration to take effect
    {
        phycr1_reg_t phycr1;
        ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, w6100->addr, W6100_REG_PHYCR1, (uint32_t *) &(phycr1.val)), err, TAG, "read PHYCR1 failed");
        phycr1.reset = 1;
        ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR1, phycr1.val), err, TAG, "write PHYCR1 failed");
        vTaskDelay(pdMS_TO_TICKS(W6100_WAIT_FOR_RESET_MS));
        phycr1.reset = 0;
        ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, w6100->addr, W6100_REG_PHYCR1, phycr1.val), err, TAG, "write PHYCR1 failed");
    }
err:
    return ret;
}

static esp_err_t w6100_init(esp_eth_phy_t *phy)
{
    esp_err_t ret = ESP_OK;
    /* Power on Ethernet PHY */
    ESP_GOTO_ON_ERROR(w6100_pwrctl(phy, true), err, TAG, "power control failed");
    /* Reset Ethernet PHY */
    ESP_GOTO_ON_ERROR(w6100_reset(phy), err, TAG, "reset failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t w6100_deinit(esp_eth_phy_t *phy)
{
    esp_err_t ret = ESP_OK;
    /* Power off Ethernet PHY */
    ESP_GOTO_ON_ERROR(w6100_pwrctl(phy, false), err, TAG, "power control failed");
    return ESP_OK;
err:
    return ret;
}

esp_eth_phy_t *esp_eth_phy_new_w6100(const eth_phy_config_t *config)
{
    esp_eth_phy_t *ret = NULL;
    ESP_GOTO_ON_FALSE(config, NULL, err, TAG, "invalid arguments");
    phy_w6100_t *w6100 = calloc(1, sizeof(phy_w6100_t));
    ESP_GOTO_ON_FALSE(w6100, NULL, err, TAG, "no mem for PHY instance");
    w6100->addr = config->phy_addr;
    w6100->reset_timeout_ms = config->reset_timeout_ms;
    w6100->reset_gpio_num = config->reset_gpio_num;
    w6100->link_status = ETH_LINK_DOWN;
    w6100->autonego_timeout_ms = config->autonego_timeout_ms;
    w6100->parent.reset = w6100_reset;
    w6100->parent.reset_hw = w6100_reset_hw;
    w6100->parent.init = w6100_init;
    w6100->parent.deinit = w6100_deinit;
    w6100->parent.set_mediator = w6100_set_mediator;
    w6100->parent.autonego_ctrl = w6100_autonego_ctrl;
    w6100->parent.get_link = w6100_get_link;
    w6100->parent.set_link = w6100_set_link;
    w6100->parent.pwrctl = w6100_pwrctl;
    w6100->parent.get_addr = w6100_get_addr;
    w6100->parent.set_addr = w6100_set_addr;
    w6100->parent.advertise_pause_ability = w6100_advertise_pause_ability;
    w6100->parent.loopback = w6100_loopback;
    w6100->parent.set_speed = w6100_set_speed;
    w6100->parent.set_duplex = w6100_set_duplex;
    w6100->parent.del = w6100_del;
    return &(w6100->parent);
err:
    return ret;
}
