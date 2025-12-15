/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "w6100.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "w6100_init";

esp_err_t w6100_init(const w6100_init_config_t *config, esp_eth_handle_t *eth_handle_out)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(config && eth_handle_out, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(config->mosi_gpio >= 0, ESP_ERR_INVALID_ARG, TAG, "mosi_gpio required");
    ESP_RETURN_ON_FALSE(config->miso_gpio >= 0, ESP_ERR_INVALID_ARG, TAG, "miso_gpio required");
    ESP_RETURN_ON_FALSE(config->sclk_gpio >= 0, ESP_ERR_INVALID_ARG, TAG, "sclk_gpio required");
    ESP_RETURN_ON_FALSE(config->cs_gpio >= 0, ESP_ERR_INVALID_ARG, TAG, "cs_gpio required");

    // Install GPIO ISR service if using interrupt mode
    if (config->int_gpio >= 0) {
        ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "GPIO ISR service install failed");
            return ret;
        }
    }

    // Initialize SPI bus
    ESP_LOGI(TAG, "Initializing SPI bus (host=%d)", config->spi_host);
    spi_bus_config_t buscfg = {
        .mosi_io_num = config->mosi_gpio,
        .miso_io_num = config->miso_gpio,
        .sclk_io_num = config->sclk_gpio,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16384,  // 16KB DMA buffer
    };
    ESP_GOTO_ON_ERROR(spi_bus_initialize(config->spi_host, &buscfg, SPI_DMA_CH_AUTO),
                      err, TAG, "SPI bus init failed");

    // Create W6100 MAC
    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = config->spi_clock_mhz * 1000 * 1000,
        .spics_io_num = config->cs_gpio,
        .queue_size = 20,
    };
    eth_w6100_config_t w6100_cfg = {
        .int_gpio_num = config->int_gpio,
        .poll_period_ms = (config->int_gpio < 0) ? 10 : 0,
        .spi_host_id = config->spi_host,
        .spi_devcfg = &spi_devcfg,
        .custom_spi_driver = ETH_DEFAULT_SPI,
    };
    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_w6100(&w6100_cfg, &mac_cfg);
    ESP_GOTO_ON_FALSE(mac, ESP_FAIL, err_spi, TAG, "Failed to create W6100 MAC");

    // Create W6100 PHY
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.reset_gpio_num = config->rst_gpio;
    esp_eth_phy_t *phy = esp_eth_phy_new_w6100(&phy_cfg);
    ESP_GOTO_ON_FALSE(phy, ESP_FAIL, err_mac, TAG, "Failed to create W6100 PHY");

    ESP_LOGI(TAG, "W6100 MAC/PHY created (INT=%d, CS=%d, CLK=%dMHz)",
             config->int_gpio, config->cs_gpio, config->spi_clock_mhz);

    // Install Ethernet driver
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_GOTO_ON_ERROR(esp_eth_driver_install(&eth_cfg, eth_handle_out),
                      err_phy, TAG, "Failed to install Ethernet driver");

    // Set MAC address if provided
    if (config->mac_addr) {
        ESP_GOTO_ON_ERROR(esp_eth_ioctl(*eth_handle_out, ETH_CMD_S_MAC_ADDR, (void *)config->mac_addr),
                          err_driver, TAG, "Failed to set MAC address");
    }

    ESP_LOGI(TAG, "W6100 initialized");
    return ESP_OK;

err_driver:
    esp_eth_driver_uninstall(*eth_handle_out);
err_phy:
    phy->del(phy);
err_mac:
    mac->del(mac);
err_spi:
    spi_bus_free(config->spi_host);
err:
    return ret;
}

esp_err_t w6100_deinit(esp_eth_handle_t eth_handle, spi_host_device_t spi_host)
{
    ESP_RETURN_ON_FALSE(eth_handle, ESP_ERR_INVALID_ARG, TAG, "eth_handle is null");
    ESP_RETURN_ON_ERROR(esp_eth_driver_uninstall(eth_handle), TAG, "Failed to uninstall driver");
    ESP_RETURN_ON_ERROR(spi_bus_free(spi_host), TAG, "Failed to free SPI bus");
    return ESP_OK;
}
