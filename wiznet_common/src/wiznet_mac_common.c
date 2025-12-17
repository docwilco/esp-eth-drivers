/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
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

/*******************************************************************************
 * SPI Read/Write Helpers (using ops for register addresses)
 ******************************************************************************/

esp_err_t wiznet_read(emac_wiznet_t *emac, uint32_t address, void *data, uint32_t len)
{
    /* Address encoding is identical for W5500/W6100:
     * - Upper bits: address phase (16-bit offset)
     * - Lower bits: control phase (BSB + RWB + OM)
     * The ops structure stores pre-encoded register addresses.
     */
    uint32_t cmd = (address >> WIZNET_ADDR_OFFSET);
    uint32_t addr = (address & 0xFFFF);  // Already includes BSB, just add read bit
    return emac->spi.read(emac->spi.ctx, cmd, addr, data, len);
}

esp_err_t wiznet_write(emac_wiznet_t *emac, uint32_t address, const void *data, uint32_t len)
{
    uint32_t cmd = (address >> WIZNET_ADDR_OFFSET);
    uint32_t addr = (address & 0xFFFF) | (WIZNET_ACCESS_MODE_WRITE << WIZNET_RWB_OFFSET);
    return emac->spi.write(emac->spi.ctx, cmd, addr, data, len);
}

esp_err_t wiznet_send_command(emac_wiznet_t *emac, uint8_t command, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    const wiznet_chip_ops_t *ops = emac->ops;

    ESP_GOTO_ON_ERROR(wiznet_write(emac, ops->reg_sock_cr, &command, sizeof(command)), err, emac->tag, "write SCR failed");
    // after chip accepts the command, the command register will be cleared automatically
    uint32_t to = 0;
    for (to = 0; to < timeout_ms / 10; to++) {
        ESP_GOTO_ON_ERROR(wiznet_read(emac, ops->reg_sock_cr, &command, sizeof(command)), err, emac->tag, "read SCR failed");
        if (!command) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_GOTO_ON_FALSE(to < timeout_ms / 10, ESP_ERR_TIMEOUT, err, emac->tag, "send command timeout");

err:
    return ret;
}

static esp_err_t wiznet_get_tx_free_size(emac_wiznet_t *emac, uint16_t *size)
{
    esp_err_t ret = ESP_OK;
    const wiznet_chip_ops_t *ops = emac->ops;
    uint16_t free0, free1 = 0;
    // read TX_FSR register more than once, until we get the same value
    do {
        ESP_GOTO_ON_ERROR(wiznet_read(emac, ops->reg_sock_tx_fsr, &free0, sizeof(free0)), err, emac->tag, "read TX FSR failed");
        ESP_GOTO_ON_ERROR(wiznet_read(emac, ops->reg_sock_tx_fsr, &free1, sizeof(free1)), err, emac->tag, "read TX FSR failed");
    } while (free0 != free1);

    *size = __builtin_bswap16(free0);

err:
    return ret;
}

static esp_err_t wiznet_get_rx_received_size(emac_wiznet_t *emac, uint16_t *size)
{
    esp_err_t ret = ESP_OK;
    const wiznet_chip_ops_t *ops = emac->ops;
    uint16_t received0, received1 = 0;
    do {
        ESP_GOTO_ON_ERROR(wiznet_read(emac, ops->reg_sock_rx_rsr, &received0, sizeof(received0)), err, emac->tag, "read RX RSR failed");
        ESP_GOTO_ON_ERROR(wiznet_read(emac, ops->reg_sock_rx_rsr, &received1, sizeof(received1)), err, emac->tag, "read RX RSR failed");
    } while (received0 != received1);
    *size = __builtin_bswap16(received0);

err:
    return ret;
}

static esp_err_t wiznet_write_buffer(emac_wiznet_t *emac, const void *buffer, uint32_t len, uint16_t offset)
{
    esp_err_t ret = ESP_OK;
    uint32_t addr = emac->ops->mem_sock_tx_base | (offset << 16);
    ESP_GOTO_ON_ERROR(wiznet_write(emac, addr, buffer, len), err, emac->tag, "write TX buffer failed");
err:
    return ret;
}

static esp_err_t wiznet_read_buffer(emac_wiznet_t *emac, void *buffer, uint32_t len, uint16_t offset)
{
    esp_err_t ret = ESP_OK;
    uint32_t addr = emac->ops->mem_sock_rx_base | (offset << 16);
    ESP_GOTO_ON_ERROR(wiznet_read(emac, addr, buffer, len), err, emac->tag, "read RX buffer failed");
err:
    return ret;
}

/*******************************************************************************
 * Common Transmit/Receive Implementation
 ******************************************************************************/

esp_err_t emac_wiznet_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    emac_wiznet_t *emac = __containerof(mac, emac_wiznet_t, parent);
    const wiznet_chip_ops_t *ops = emac->ops;
    uint16_t offset = 0;

    ESP_GOTO_ON_FALSE(length <= ETH_MAX_PACKET_SIZE, ESP_ERR_INVALID_ARG, err,
                      emac->tag, "frame size is too big (actual %" PRIu32 ", maximum %u)", length, ETH_MAX_PACKET_SIZE);
    // check if there's free memory to store this packet
    uint16_t free_size = 0;
    ESP_GOTO_ON_ERROR(wiznet_get_tx_free_size(emac, &free_size), err, emac->tag, "get free size failed");
    ESP_GOTO_ON_FALSE(length <= free_size, ESP_ERR_NO_MEM, err, emac->tag, "free size (%" PRIu16 ") < send length (%" PRIu32 ")", free_size, length);
    // get current write pointer
    ESP_GOTO_ON_ERROR(wiznet_read(emac, ops->reg_sock_tx_wr, &offset, sizeof(offset)), err, emac->tag, "read TX WR failed");
    offset = __builtin_bswap16(offset);
    // copy data to tx memory
    ESP_GOTO_ON_ERROR(wiznet_write_buffer(emac, buf, length, offset), err, emac->tag, "write frame failed");
    // update write pointer
    offset += length;
    offset = __builtin_bswap16(offset);
    ESP_GOTO_ON_ERROR(wiznet_write(emac, ops->reg_sock_tx_wr, &offset, sizeof(offset)), err, emac->tag, "write TX WR failed");
    // issue SEND command
    ESP_GOTO_ON_ERROR(wiznet_send_command(emac, ops->cmd_send, 100), err, emac->tag, "issue SEND command failed");

    // polling the TX done event
    uint8_t status = 0;
    uint64_t start = esp_timer_get_time();
    uint64_t now = 0;
    do {
        now = esp_timer_get_time();
        if (!ops->is_sane_for_rxtx(emac) || (now - start) > emac->tx_tmo) {
            return ESP_FAIL;
        }
        ESP_GOTO_ON_ERROR(wiznet_read(emac, ops->reg_sock_ir, &status, sizeof(status)), err, emac->tag, "read SOCK0 IR failed");
    } while (!(status & ops->sir_send));
    // clear the event bit
    status = ops->sir_send;
    ESP_GOTO_ON_ERROR(wiznet_write(emac, ops->reg_sock_irclr, &status, sizeof(status)), err, emac->tag, "write SOCK0 IRCLR failed");

err:
    return ret;
}

typedef struct {
    uint32_t offset;
    uint32_t copy_len;
    uint32_t rx_len;
    uint32_t remain;
} __attribute__((packed)) emac_wiznet_auto_buf_info_t;

#define WIZNET_ETH_MAC_RX_BUF_SIZE_AUTO (0)

static esp_err_t emac_wiznet_alloc_recv_buf(emac_wiznet_t *emac, uint8_t **buf, uint32_t *length)
{
    esp_err_t ret = ESP_OK;
    const wiznet_chip_ops_t *ops = emac->ops;
    uint16_t offset = 0;
    uint16_t rx_len = 0;
    uint32_t copy_len = 0;
    uint16_t remain_bytes = 0;
    *buf = NULL;

    wiznet_get_rx_received_size(emac, &remain_bytes);
    if (remain_bytes) {
        // get current read pointer
        ESP_GOTO_ON_ERROR(wiznet_read(emac, ops->reg_sock_rx_rd, &offset, sizeof(offset)), err, emac->tag, "read RX RD failed");
        offset = __builtin_bswap16(offset);
        // read head
        ESP_GOTO_ON_ERROR(wiznet_read_buffer(emac, &rx_len, sizeof(rx_len), offset), err, emac->tag, "read frame header failed");
        rx_len = __builtin_bswap16(rx_len) - 2; // data size includes 2 bytes of header
        // frames larger than expected will be truncated
        copy_len = rx_len > *length ? *length : rx_len;
        // runt frames are not forwarded, but check the length anyway since it could be corrupted at SPI bus
        ESP_GOTO_ON_FALSE(copy_len >= ETH_MIN_PACKET_SIZE - ETH_CRC_LEN, ESP_ERR_INVALID_SIZE, err, emac->tag, "invalid frame length %" PRIu32, copy_len);
        *buf = malloc(copy_len);
        if (*buf != NULL) {
            emac_wiznet_auto_buf_info_t *buff_info = (emac_wiznet_auto_buf_info_t *)*buf;
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

esp_err_t emac_wiznet_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
    esp_err_t ret = ESP_OK;
    emac_wiznet_t *emac = __containerof(mac, emac_wiznet_t, parent);
    const wiznet_chip_ops_t *ops = emac->ops;
    uint16_t offset = 0;
    uint16_t rx_len = 0;
    uint16_t copy_len = 0;
    uint16_t remain_bytes = 0;
    emac->packets_remain = false;

    if (*length != WIZNET_ETH_MAC_RX_BUF_SIZE_AUTO) {
        wiznet_get_rx_received_size(emac, &remain_bytes);
        if (remain_bytes) {
            // get current read pointer
            ESP_GOTO_ON_ERROR(wiznet_read(emac, ops->reg_sock_rx_rd, &offset, sizeof(offset)), err, emac->tag, "read RX RD failed");
            offset = __builtin_bswap16(offset);
            // read head first
            ESP_GOTO_ON_ERROR(wiznet_read_buffer(emac, &rx_len, sizeof(rx_len), offset), err, emac->tag, "read frame header failed");
            rx_len = __builtin_bswap16(rx_len) - 2; // data size includes 2 bytes of header
            // frames larger than expected will be truncated
            copy_len = rx_len > *length ? *length : rx_len;
        } else {
            // silently return when no frame is waiting
            goto err;
        }
    } else {
        emac_wiznet_auto_buf_info_t *buff_info = (emac_wiznet_auto_buf_info_t *)buf;
        offset = buff_info->offset;
        copy_len = buff_info->copy_len;
        rx_len = buff_info->rx_len;
        remain_bytes = buff_info->remain;
    }
    // 2 bytes of header
    offset += 2;
    // read the payload
    ESP_GOTO_ON_ERROR(wiznet_read_buffer(emac, emac->rx_buffer, copy_len, offset), err, emac->tag, "read payload failed, len=%" PRIu16 ", offset=%" PRIu16, rx_len, offset);
    memcpy(buf, emac->rx_buffer, copy_len);
    offset += rx_len;
    // update read pointer
    offset = __builtin_bswap16(offset);
    ESP_GOTO_ON_ERROR(wiznet_write(emac, ops->reg_sock_rx_rd, &offset, sizeof(offset)), err, emac->tag, "write RX RD failed");
    /* issue RECV command */
    ESP_GOTO_ON_ERROR(wiznet_send_command(emac, ops->cmd_recv, 100), err, emac->tag, "issue RECV command failed");
    // check if there're more data need to process
    remain_bytes -= rx_len + 2;
    emac->packets_remain = remain_bytes > 0;

    *length = copy_len;
    return ret;
err:
    *length = 0;
    return ret;
}

static esp_err_t emac_wiznet_flush_recv_frame(emac_wiznet_t *emac)
{
    esp_err_t ret = ESP_OK;
    const wiznet_chip_ops_t *ops = emac->ops;
    uint16_t offset = 0;
    uint16_t rx_len = 0;
    uint16_t remain_bytes = 0;
    emac->packets_remain = false;

    wiznet_get_rx_received_size(emac, &remain_bytes);
    if (remain_bytes) {
        // get current read pointer
        ESP_GOTO_ON_ERROR(wiznet_read(emac, ops->reg_sock_rx_rd, &offset, sizeof(offset)), err, emac->tag, "read RX RD failed");
        offset = __builtin_bswap16(offset);
        // read head first
        ESP_GOTO_ON_ERROR(wiznet_read_buffer(emac, &rx_len, sizeof(rx_len), offset), err, emac->tag, "read frame header failed");
        // update read pointer
        rx_len = __builtin_bswap16(rx_len);
        offset += rx_len;
        offset = __builtin_bswap16(offset);
        ESP_GOTO_ON_ERROR(wiznet_write(emac, ops->reg_sock_rx_rd, &offset, sizeof(offset)), err, emac->tag, "write RX RD failed");
        /* issue RECV command */
        ESP_GOTO_ON_ERROR(wiznet_send_command(emac, ops->cmd_recv, 100), err, emac->tag, "issue RECV command failed");
        // check if there're more data need to process
        remain_bytes -= rx_len;
        emac->packets_remain = remain_bytes > 0;
    }
err:
    return ret;
}

void emac_wiznet_task(void *arg)
{
    emac_wiznet_t *emac = (emac_wiznet_t *)arg;
    const wiznet_chip_ops_t *ops = emac->ops;
    uint8_t status = 0;
    uint8_t *buffer = NULL;
    uint32_t frame_len = 0;
    uint32_t buf_len = 0;
    esp_err_t ret;
    while (1) {
        /* check if the task receives any notification */
        if (emac->int_gpio_num >= 0) {                                    // if in interrupt mode
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0 &&     // if no notification ...
                    gpio_get_level(emac->int_gpio_num) != 0) {            // ...and no interrupt asserted
                continue;                                                 // -> just continue to check again
            }
        } else {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        /* read interrupt status */
        wiznet_read(emac, ops->reg_sock_ir, &status, sizeof(status));
        /* packet received */
        if (status & ops->sir_recv) {
            /* clear interrupt status */
            uint8_t clr = ops->sir_recv;
            wiznet_write(emac, ops->reg_sock_irclr, &clr, sizeof(clr));
            do {
                /* define max expected frame len */
                frame_len = ETH_MAX_PACKET_SIZE;
                if ((ret = emac_wiznet_alloc_recv_buf(emac, &buffer, &frame_len)) == ESP_OK) {
                    if (buffer != NULL) {
                        /* we have memory to receive the frame of maximal size previously defined */
                        buf_len = WIZNET_ETH_MAC_RX_BUF_SIZE_AUTO;
                        if (emac->parent.receive(&emac->parent, buffer, &buf_len) == ESP_OK) {
                            if (buf_len == 0) {
                                free(buffer);
                            } else if (frame_len > buf_len) {
                                ESP_LOGE(emac->tag, "received frame was truncated");
                                free(buffer);
                            } else {
                                ESP_LOGD(emac->tag, "receive len=%" PRIu32, buf_len);
                                /* pass the buffer to stack (e.g. TCP/IP layer) */
                                emac->eth->stack_input(emac->eth, buffer, buf_len);
                            }
                        } else {
                            ESP_LOGE(emac->tag, "frame read from module failed");
                            free(buffer);
                        }
                    } else if (frame_len) {
                        ESP_LOGE(emac->tag, "invalid combination of frame_len(%" PRIu32 ") and buffer pointer(%p)", frame_len, buffer);
                    }
                } else if (ret == ESP_ERR_NO_MEM) {
                    ESP_LOGE(emac->tag, "no mem for receive buffer");
                    emac_wiznet_flush_recv_frame(emac);
                } else {
                    ESP_LOGE(emac->tag, "unexpected error 0x%x", ret);
                }
            } while (emac->packets_remain);
        }
    }
    vTaskDelete(NULL);
}
