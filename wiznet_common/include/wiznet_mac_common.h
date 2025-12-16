/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_eth_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wiznet_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Common base structure for WIZnet EMAC implementations
 *
 * This structure contains fields common to all WIZnet Ethernet MAC drivers
 * (W5500, W6100, etc.). Chip-specific structures should embed this as their
 * first member to allow safe casting between base and derived types.
 *
 * Usage:
 * @code
 * typedef struct {
 *     emac_wiznet_t base;        // Must be first member
 *     uint8_t chip_specific;     // Chip-specific fields follow
 * } emac_w6100_t;
 * @endcode
 */
typedef struct {
    esp_eth_mac_t parent;           /*!< ESP-IDF MAC vtable (must be first for __containerof) */
    esp_eth_mediator_t *eth;        /*!< Mediator for callbacks to ESP-ETH layer */
    eth_spi_custom_driver_t spi;    /*!< SPI driver interface */
    TaskHandle_t rx_task_hdl;       /*!< RX task handle */
    const char *tag;                /*!< Logging tag (e.g., "w6100.mac") */
    uint32_t sw_reset_timeout_ms;   /*!< Software reset timeout */
    int int_gpio_num;               /*!< Interrupt GPIO number, or -1 for polling mode */
    esp_timer_handle_t poll_timer;  /*!< Poll timer handle (polling mode only) */
    uint32_t poll_period_ms;        /*!< Poll period in milliseconds */
    uint8_t addr[6];                /*!< MAC address (ETH_ADDR_LEN) */
    bool packets_remain;            /*!< Flag indicating more packets in RX buffer */
    uint8_t *rx_buffer;             /*!< RX buffer for incoming frames */
    uint32_t tx_tmo;                /*!< TX timeout in microseconds (speed-dependent) */
} emac_wiznet_t;

/**
 * @brief Set mediator for Ethernet MAC
 *
 * @param mac Ethernet MAC instance
 * @param eth Ethernet mediator
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if eth is NULL
 */
esp_err_t emac_wiznet_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth);

/**
 * @brief Get MAC address
 *
 * @param mac Ethernet MAC instance
 * @param addr Buffer to store MAC address (6 bytes)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if addr is NULL
 */
esp_err_t emac_wiznet_get_addr(esp_eth_mac_t *mac, uint8_t *addr);

/**
 * @brief Set duplex mode (informational only, WIZnet chips auto-negotiate)
 *
 * @param mac Ethernet MAC instance
 * @param duplex Duplex mode
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for unknown duplex
 */
esp_err_t emac_wiznet_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex);

/**
 * @brief Enable flow control (not supported by WIZnet chips)
 *
 * @param mac Ethernet MAC instance
 * @param enable Enable flag (ignored)
 * @return ESP_ERR_NOT_SUPPORTED always
 */
esp_err_t emac_wiznet_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable);

/**
 * @brief Set peer pause ability (not supported by WIZnet chips)
 *
 * @param mac Ethernet MAC instance
 * @param ability Pause ability (ignored)
 * @return ESP_ERR_NOT_SUPPORTED always
 */
esp_err_t emac_wiznet_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t ability);

/**
 * @brief Set link state and start/stop MAC accordingly
 *
 * @param mac Ethernet MAC instance
 * @param link Link state (ETH_LINK_UP or ETH_LINK_DOWN)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for unknown link state
 */
esp_err_t emac_wiznet_set_link(esp_eth_mac_t *mac, eth_link_t link);

/**
 * @brief Deinitialize MAC
 *
 * Stops the MAC, removes ISR handler, stops poll timer, and notifies mediator.
 *
 * @param mac Ethernet MAC instance
 * @return ESP_OK always
 */
esp_err_t emac_wiznet_deinit(esp_eth_mac_t *mac);

/**
 * @brief Delete MAC instance and free resources
 *
 * Deletes poll timer, RX task, deinitializes SPI, frees RX buffer and struct.
 *
 * @param mac Ethernet MAC instance
 * @return ESP_OK always
 */
esp_err_t emac_wiznet_del(esp_eth_mac_t *mac);

/**
 * @brief ISR handler for WIZnet interrupt pin
 *
 * Notifies the RX task when interrupt fires. Must be installed with
 * gpio_isr_handler_add() with the emac_wiznet_t pointer as argument.
 *
 * @param arg Pointer to emac_wiznet_t instance
 */
void wiznet_isr_handler(void *arg);

/**
 * @brief Poll timer callback
 *
 * Notifies the RX task periodically in polling mode. Must be registered
 * with esp_timer_create() with the emac_wiznet_t pointer as argument.
 *
 * @param arg Pointer to emac_wiznet_t instance
 */
void wiznet_poll_timer(void *arg);

#ifdef __cplusplus
}
#endif
