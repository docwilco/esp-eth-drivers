/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_eth_mac.h"
#include "esp_eth_mac_spi.h"
#include "esp_eth_phy.h"
#include "esp_eth_driver.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * MAC Configuration
 ******************************************************************************/

/**
 * @brief W6100 specific configuration
 */
typedef struct {
    int int_gpio_num;                                   /*!< Interrupt GPIO number, set -1 to not use interrupt and to poll rx status periodically */
    uint32_t poll_period_ms;                            /*!< Period in ms to poll rx status when interrupt mode is not used */
    spi_host_device_t spi_host_id;                      /*!< SPI peripheral (this field is invalid when custom SPI driver is defined) */
    spi_device_interface_config_t *spi_devcfg;          /*!< SPI device configuration (this field is invalid when custom SPI driver is defined) */
    eth_spi_custom_driver_config_t custom_spi_driver;   /*!< Custom SPI driver definitions */
} eth_w6100_config_t;

/**
 * @brief Default W6100 specific configuration
 *
 */
#define ETH_W6100_DEFAULT_CONFIG(spi_host, spi_devcfg_p) \
    {                                           \
        .int_gpio_num = 4,                      \
        .poll_period_ms = 0,                    \
        .spi_host_id = spi_host,                \
        .spi_devcfg = spi_devcfg_p,             \
        .custom_spi_driver = ETH_DEFAULT_SPI,   \
    }

/**
 * @brief Create W6100 Ethernet MAC instance
 *
 * @param[in] w6100_config W6100 specific configuration
 * @param[in] mac_config Ethernet MAC configuration
 *
 * @return
 *      - instance: pointer to MAC instance on success
 *      - NULL: on failure
 *
 * @note The W6100 chip must be connected via SPI interface
 * @note Interrupt mode is recommended for better performance
 */
esp_eth_mac_t *esp_eth_mac_new_w6100(const eth_w6100_config_t *w6100_config, const eth_mac_config_t *mac_config);

/*******************************************************************************
 * PHY Configuration
 ******************************************************************************/

/**
 * @brief Create a PHY instance for W6100
 *
 * @param[in] config PHY configuration
 *
 * @return
 *      - instance: pointer to PHY instance on success
 *      - NULL: on failure
 *
 * @note The W6100 integrates a built-in PHY, so this driver interfaces
 *       with the chip's internal PHY registers via the MAC driver's
 *       phy_reg_read/write mediator functions.
 */
esp_eth_phy_t *esp_eth_phy_new_w6100(const eth_phy_config_t *config);

/*******************************************************************************
 * Convenience Initialization
 ******************************************************************************/

/**
 * @brief W6100 initialization configuration
 */
typedef struct {
    spi_host_device_t spi_host;  /*!< SPI host (SPI2_HOST or SPI3_HOST) */
    int mosi_gpio;               /*!< GPIO for MOSI */
    int miso_gpio;               /*!< GPIO for MISO */
    int sclk_gpio;               /*!< GPIO for SCLK */
    int cs_gpio;                 /*!< GPIO for CS */
    int int_gpio;                /*!< GPIO for INT (-1 for polling mode) */
    int rst_gpio;                /*!< GPIO for RST (-1 to skip hardware reset) */
    int spi_clock_mhz;           /*!< SPI clock frequency in MHz (max 80) */
    const uint8_t *mac_addr;     /*!< MAC address (NULL to use chip default or eFuse) */
} w6100_init_config_t;

/**
 * @brief Default W6100 initialization configuration
 */
#define W6100_INIT_CONFIG_DEFAULT() { \
    .spi_host = SPI2_HOST, \
    .mosi_gpio = -1, \
    .miso_gpio = -1, \
    .sclk_gpio = -1, \
    .cs_gpio = -1, \
    .int_gpio = -1, \
    .rst_gpio = -1, \
    .spi_clock_mhz = 33, \
    .mac_addr = NULL, \
}

/**
 * @brief Initialize W6100 Ethernet
 *
 * This function initializes the SPI bus, creates MAC/PHY drivers,
 * and installs the Ethernet driver.
 *
 * @param[in] config Initialization configuration
 * @param[out] eth_handle_out Initialized Ethernet driver handle
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG when passed invalid pointers or config
 *          - ESP_ERR_NO_MEM when memory allocation fails
 *          - ESP_FAIL on any other failure
 */
esp_err_t w6100_init(const w6100_init_config_t *config, esp_eth_handle_t *eth_handle_out);

/**
 * @brief Deinitialize W6100 Ethernet
 *
 * @param[in] eth_handle Ethernet driver handle from w6100_init()
 * @param[in] spi_host SPI host used during init (needed to free bus)
 * @return
 *          - ESP_OK on success
 *          - ESP_FAIL on failure
 */
esp_err_t w6100_deinit(esp_eth_handle_t eth_handle, spi_host_device_t spi_host);

#ifdef __cplusplus
}
#endif
