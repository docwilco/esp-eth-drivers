/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "wiznet_mac_common.h"

esp_err_t emac_wiznet_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth)
{
    emac_wiznet_t *emac = __containerof(mac, emac_wiznet_t, parent);
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(eth, ESP_ERR_INVALID_ARG, err, emac->tag, "can't set mac's mediator to null");
    emac->eth = eth;
    return ESP_OK;
err:
    return ret;
}

esp_err_t emac_wiznet_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    emac_wiznet_t *emac = __containerof(mac, emac_wiznet_t, parent);
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, emac->tag, "invalid argument");
    memcpy(addr, emac->addr, 6);

err:
    return ret;
}

esp_err_t emac_wiznet_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex)
{
    emac_wiznet_t *emac = __containerof(mac, emac_wiznet_t, parent);
    esp_err_t ret = ESP_OK;
    switch (duplex) {
    case ETH_DUPLEX_HALF:
        ESP_LOGD(emac->tag, "working in half duplex");
        break;
    case ETH_DUPLEX_FULL:
        ESP_LOGD(emac->tag, "working in full duplex");
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, emac->tag, "unknown duplex");
        break;
    }

err:
    return ret;
}

esp_err_t emac_wiznet_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable)
{
    /* WIZnet chips don't support flow control */
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t emac_wiznet_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t ability)
{
    /* WIZnet chips don't support PAUSE function */
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t emac_wiznet_set_link(esp_eth_mac_t *mac, eth_link_t link)
{
    emac_wiznet_t *emac = __containerof(mac, emac_wiznet_t, parent);
    esp_err_t ret = ESP_OK;
    switch (link) {
    case ETH_LINK_UP:
        ESP_LOGD(emac->tag, "link is up");
        ESP_GOTO_ON_ERROR(mac->start(mac), err, emac->tag, "start failed");
        if (emac->poll_timer) {
            ESP_GOTO_ON_ERROR(esp_timer_start_periodic(emac->poll_timer, emac->poll_period_ms * 1000),
                              err, emac->tag, "start poll timer failed");
        }
        break;
    case ETH_LINK_DOWN:
        ESP_LOGD(emac->tag, "link is down");
        ESP_GOTO_ON_ERROR(mac->stop(mac), err, emac->tag, "stop failed");
        if (emac->poll_timer) {
            ESP_GOTO_ON_ERROR(esp_timer_stop(emac->poll_timer),
                              err, emac->tag, "stop poll timer failed");
        }
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, emac->tag, "unknown link status");
        break;
    }

err:
    return ret;
}

esp_err_t emac_wiznet_deinit(esp_eth_mac_t *mac)
{
    emac_wiznet_t *emac = __containerof(mac, emac_wiznet_t, parent);
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

esp_err_t emac_wiznet_del(esp_eth_mac_t *mac)
{
    emac_wiznet_t *emac = __containerof(mac, emac_wiznet_t, parent);
    if (emac->poll_timer) {
        esp_timer_delete(emac->poll_timer);
    }
    vTaskDelete(emac->rx_task_hdl);
    emac->spi.deinit(emac->spi.ctx);
    heap_caps_free(emac->rx_buffer);
    free(emac);
    return ESP_OK;
}

IRAM_ATTR void wiznet_isr_handler(void *arg)
{
    emac_wiznet_t *emac = (emac_wiznet_t *)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    /* notify RX task */
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

void wiznet_poll_timer(void *arg)
{
    emac_wiznet_t *emac = (emac_wiznet_t *)arg;
    xTaskNotifyGive(emac->rx_task_hdl);
}
