/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include <inttypes.h>
#include "esp_eth_mac_spi.h"
#include "esp_eth_mac_w5500.h"
#include "driver/gpio.h"
#include "esp_private/gpio.h"
#include "soc/io_mux_reg.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "wiznet_spi.h"
#include "wiznet_mac_common.h"
#include "w5500.h"
#include "sdkconfig.h"

static const char *TAG = "w5500.mac";

#define W5500_TX_MEM_SIZE (0x4000)
#define W5500_RX_MEM_SIZE (0x4000)
#define W5500_100M_TX_TMO_US (200)
#define W5500_10M_TX_TMO_US (1500)

typedef struct {
    emac_wiznet_t base;  // Must be first member for safe casting
    uint8_t mcast_cnt;
} emac_w5500_t;

/*******************************************************************************
 * Chip-specific ops for W5500
 ******************************************************************************/

static bool is_w5500_sane_for_rxtx(emac_wiznet_t *emac);

static const wiznet_chip_ops_t w5500_ops = {
    /* Register translation table for common registers */
    .regs = {
        [WIZNET_REG_MAC_ADDR]        = W5500_REG_MAC,
        [WIZNET_REG_SOCK_MR]         = W5500_REG_SOCK_MR(0),
        [WIZNET_REG_SOCK_IMR]        = W5500_REG_SOCK_IMR(0),
        [WIZNET_REG_SOCK_RXBUF_SIZE] = W5500_REG_SOCK_RXBUF_SIZE(0),
        [WIZNET_REG_SOCK_TXBUF_SIZE] = W5500_REG_SOCK_TXBUF_SIZE(0),
        [WIZNET_REG_INT_LEVEL]       = W5500_REG_INTLEVEL,
    },

    /* Socket 0 registers (pre-computed addresses) */
    .reg_sock_cr = W5500_REG_SOCK_CR(0),
    .reg_sock_ir = W5500_REG_SOCK_IR(0),
    .reg_sock_tx_fsr = W5500_REG_SOCK_TX_FSR(0),
    .reg_sock_tx_wr = W5500_REG_SOCK_TX_WR(0),
    .reg_sock_rx_rsr = W5500_REG_SOCK_RX_RSR(0),
    .reg_sock_rx_rd = W5500_REG_SOCK_RX_RD(0),
    .reg_simr = W5500_REG_SIMR,

    /* Memory base addresses (offset added at runtime) */
    .mem_sock_tx_base = W5500_MEM_SOCK_TX(0, 0),
    .mem_sock_rx_base = W5500_MEM_SOCK_RX(0, 0),

    /* W5500 writes to IR register to clear interrupts (same as read) */
    .reg_sock_irclr = W5500_REG_SOCK_IR(0),

    /* Command values */
    .cmd_send = W5500_SCR_SEND,
    .cmd_recv = W5500_SCR_RECV,
    .cmd_open = W5500_SCR_OPEN,
    .cmd_close = W5500_SCR_CLOSE,

    /* Interrupt bits */
    .sir_send = W5500_SIR_SEND,
    .sir_recv = W5500_SIR_RECV,
    .simr_sock0 = W5500_SIMR_SOCK0,

    /* Bit masks */
    .smr_mac_filter = W5500_SMR_MAC_FILTER,

    /* Callbacks */
    .is_sane_for_rxtx = is_w5500_sane_for_rxtx,
};

static bool is_w5500_sane_for_rxtx(emac_wiznet_t *emac)
{
    uint8_t phycfg;
    /* phy is ok for rx and tx operations if bits RST and LNK are set (no link down, no reset) */
    if (wiznet_read(emac, W5500_REG_PHYCFGR, &phycfg, 1) == ESP_OK 
        && (phycfg & 0x8001)) {
        return true;
    }
    return false;
}

/* Helper macros to cast emac_w5500_t* to emac_wiznet_t* for wiznet_* functions */
#define W5500_READ(emac, addr, data, len)   wiznet_read(&(emac)->base, (addr), (data), (len))
#define W5500_WRITE(emac, addr, data, len)  wiznet_write(&(emac)->base, (addr), (data), (len))
#define W5500_SEND_CMD(emac, cmd, tmo)      wiznet_send_command(&(emac)->base, (cmd), (tmo))

static esp_err_t w5500_set_mac_addr(emac_w5500_t *emac)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_MAC, emac->base.addr, 6), err, TAG, "write MAC address register failed");
err:
    return ret;
}

static esp_err_t w5500_reset(emac_w5500_t *emac)
{
    esp_err_t ret = ESP_OK;
    /* software reset */
    uint8_t mr = W5500_MR_RST; // Set RST bit (auto clear)
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_MR, &mr, sizeof(mr)), err, TAG, "write MR failed");
    uint32_t to = 0;
    for (to = 0; to < emac->base.sw_reset_timeout_ms / 10; to++) {
        ESP_GOTO_ON_ERROR(W5500_READ(emac, W5500_REG_MR, &mr, sizeof(mr)), err, TAG, "read MR failed");
        if (!(mr & W5500_MR_RST)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_GOTO_ON_FALSE(to < emac->base.sw_reset_timeout_ms / 10, ESP_ERR_TIMEOUT, err, TAG, "reset timeout");

err:
    return ret;
}

static esp_err_t w5500_verify_id(emac_w5500_t *emac)
{
    esp_err_t ret = ESP_OK;
    uint8_t version = 0;

    // W5500 doesn't have chip ID, we check the version number instead
    // The version number may be polled multiple times since it was observed that
    // some W5500 units may return version 0 when it is read right after the reset
    ESP_LOGD(TAG, "Waiting W5500 to start & verify version...");
    uint32_t to = 0;
    for (to = 0; to < emac->base.sw_reset_timeout_ms / 10; to++) {
        ESP_GOTO_ON_ERROR(W5500_READ(emac, W5500_REG_VERSIONR, &version, sizeof(version)), err, TAG, "read VERSIONR failed");
        if (version == W5500_CHIP_VERSION) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGE(TAG, "W5500 version mismatched, expected 0x%02x, got 0x%02" PRIx8, W5500_CHIP_VERSION, version);
    return ESP_ERR_INVALID_VERSION;
err:
    return ret;
}

static esp_err_t w5500_setup_default(emac_w5500_t *emac)
{
    esp_err_t ret = ESP_OK;
    uint8_t reg_value = 16;

    // Only SOCK0 can be used as MAC RAW mode, so we give the whole buffer (16KB TX and 16KB RX) to SOCK0, which doesn't have any effect for TX though.
    // A larger TX buffer doesn't buy us pipelining - each SEND is one frame and must complete before the next.
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_SOCK_RXBUF_SIZE(0), &reg_value, sizeof(reg_value)), err, TAG, "set rx buffer size failed");
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_SOCK_TXBUF_SIZE(0), &reg_value, sizeof(reg_value)), err, TAG, "set tx buffer size failed");
    reg_value = 0;
    for (int i = 1; i < 8; i++) {
        ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_SOCK_RXBUF_SIZE(i), &reg_value, sizeof(reg_value)), err, TAG, "set rx buffer size failed");
        ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_SOCK_TXBUF_SIZE(i), &reg_value, sizeof(reg_value)), err, TAG, "set tx buffer size failed");
    }

    /* Enable ping block, disable PPPoE, WOL */
    reg_value = W5500_MR_PB;
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_MR, &reg_value, sizeof(reg_value)), err, TAG, "write MR failed");
    /* Disable interrupt for all sockets by default */
    reg_value = 0;
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_SIMR, &reg_value, sizeof(reg_value)), err, TAG, "write SIMR failed");
    /* Enable MAC RAW mode for SOCK0, enable MAC filter, no blocking broadcast and block multicast */
    reg_value = W5500_SMR_MAC_RAW | W5500_SMR_MAC_FILTER | W5500_SMR_MAC_BLOCK_MCAST;
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_SOCK_MR(0), &reg_value, sizeof(reg_value)), err, TAG, "write SMR failed");
    /* Enable receive event for SOCK0 */
    reg_value = W5500_SIR_RECV;
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_SOCK_IMR(0), &reg_value, sizeof(reg_value)), err, TAG, "write SOCK0 IMR failed");
    /* Set the interrupt re-assert level to maximum (~1.5ms) to lower the chances of missing it */
    uint16_t int_level = __builtin_bswap16(0xFFFF);
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_INTLEVEL, &int_level, sizeof(int_level)), err, TAG, "write INTLEVEL failed");

err:
    return ret;
}

static esp_err_t emac_w5500_start(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);
    uint8_t reg_value = 0;
    /* open SOCK0 */
    ESP_GOTO_ON_ERROR(W5500_SEND_CMD(emac, W5500_SCR_OPEN, 100), err, TAG, "issue OPEN command failed");
    /* enable interrupt for SOCK0 */
    reg_value = W5500_SIMR_SOCK0;
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_SIMR, &reg_value, sizeof(reg_value)), err, TAG, "write SIMR failed");

err:
    return ret;
}

static esp_err_t emac_w5500_stop(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);
    uint8_t reg_value = 0;
    /* disable interrupt */
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_SIMR, &reg_value, sizeof(reg_value)), err, TAG, "write SIMR failed");
    /* close SOCK0 */
    ESP_GOTO_ON_ERROR(W5500_SEND_CMD(emac, W5500_SCR_CLOSE, 100), err, TAG, "issue CLOSE command failed");

err:
    return ret;
}

static esp_err_t emac_w5500_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value)
{
    esp_err_t ret = ESP_OK;
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);
    // PHY register and MAC registers are mixed together in W5500
    // The only PHY register is PHYCFGR
    ESP_GOTO_ON_FALSE(phy_reg == W5500_REG_PHYCFGR, ESP_FAIL, err, TAG, "wrong PHY register");
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_PHYCFGR, &reg_value, sizeof(uint8_t)), err, TAG, "write PHY register failed");

err:
    return ret;
}

static esp_err_t emac_w5500_read_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(reg_value, ESP_ERR_INVALID_ARG, err, TAG, "can't set reg_value to null");
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);
    // PHY register and MAC registers are mixed together in W5500
    // The only PHY register is PHYCFGR
    ESP_GOTO_ON_FALSE(phy_reg == W5500_REG_PHYCFGR, ESP_FAIL, err, TAG, "wrong PHY register");
    ESP_GOTO_ON_ERROR(W5500_READ(emac, W5500_REG_PHYCFGR, reg_value, sizeof(uint8_t)), err, TAG, "read PHY register failed");

err:
    return ret;
}

static esp_err_t emac_w5500_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);
    memcpy(emac->base.addr, addr, 6);
    ESP_GOTO_ON_ERROR(w5500_set_mac_addr(emac), err, TAG, "set mac address failed");

err:
    return ret;
}

static esp_err_t emac_w5500_set_block_ip4_mcast(esp_eth_mac_t *mac, bool block)
{
    esp_err_t ret = ESP_OK;
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);
    uint8_t smr;
    ESP_GOTO_ON_ERROR(W5500_READ(emac, W5500_REG_SOCK_MR(0), &smr, sizeof(smr)), err, TAG, "read SMR failed");
    if (block) {
        smr |= W5500_SMR_MAC_BLOCK_MCAST;
    } else {
        smr &= ~W5500_SMR_MAC_BLOCK_MCAST;
    }
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_SOCK_MR(0), &smr, sizeof(smr)), err, TAG, "write SMR failed");
err:
    return ret;
}

static esp_err_t emac_w5500_add_mac_filter(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);
    // W5500 doesn't have specific MAC filter, so we just un-block multicast. W5500 filters out all multicast packets
    // except for IP multicast. However, behavior is not consistent. IPv4 multicast can be blocked, but IPv6 is always
    // accepted (this is not documented behavior, but it's observed on the real hardware).
    if (addr[0] == 0x01 && addr[1] == 0x00 && addr[2] == 0x5e) {
        ESP_GOTO_ON_ERROR(emac_w5500_set_block_ip4_mcast(mac, false), err, TAG, "set block multicast failed");
        emac->mcast_cnt++;
    } else if (addr[0] == 0x33 && addr[1] == 0x33) {
        ESP_LOGW(TAG, "IPv6 multicast is always filtered in by W5500.");
    } else {
        ESP_LOGE(TAG, "W5500 filters in IP multicast frames only!");
        ret = ESP_ERR_NOT_SUPPORTED;
    }
err:
    return ret;
}

static esp_err_t emac_w5500_del_mac_filter(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);

    ESP_GOTO_ON_FALSE(!(addr[0] == 0x33 && addr[1] == 0x33), ESP_FAIL, err, TAG, "IPv6 multicast is always filtered in by W5500.");

    if (addr[0] == 0x01 && addr[1] == 0x00 && addr[2] == 0x5e && emac->mcast_cnt > 0) {
        emac->mcast_cnt--;
    }
    if (emac->mcast_cnt == 0) {
        // W5500 doesn't have specific MAC filter, so we just block multicast
        ESP_GOTO_ON_ERROR(emac_w5500_set_block_ip4_mcast(mac, true), err, TAG, "set block multicast failed");
    }
err:
    return ret;
}

static esp_err_t emac_w5500_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)
{
    esp_err_t ret = ESP_OK;
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);
    switch (speed) {
    case ETH_SPEED_10M:
        emac->base.tx_tmo = W5500_10M_TX_TMO_US;
        ESP_LOGD(TAG, "working in 10Mbps");
        break;
    case ETH_SPEED_100M:
        emac->base.tx_tmo = W5500_100M_TX_TMO_US;
        ESP_LOGD(TAG, "working in 100Mbps");
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unknown speed");
        break;
    }

err:
    return ret;
}

static esp_err_t emac_w5500_set_promiscuous(esp_eth_mac_t *mac, bool enable)
{
    esp_err_t ret = ESP_OK;
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);
    uint8_t smr = 0;
    ESP_GOTO_ON_ERROR(W5500_READ(emac, W5500_REG_SOCK_MR(0), &smr, sizeof(smr)), err, TAG, "read SMR failed");
    if (enable) {
        smr &= ~W5500_SMR_MAC_FILTER;
    } else {
        smr |= W5500_SMR_MAC_FILTER;
    }
    ESP_GOTO_ON_ERROR(W5500_WRITE(emac, W5500_REG_SOCK_MR(0), &smr, sizeof(smr)), err, TAG, "write SMR failed");

err:
    return ret;
}

static esp_err_t emac_w5500_set_all_multicast(esp_eth_mac_t *mac, bool enable)
{
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);
    ESP_RETURN_ON_ERROR(emac_w5500_set_block_ip4_mcast(mac, !enable), TAG, "set block multicast failed");
    emac->mcast_cnt = 0;
    if (enable) {
        ESP_LOGW(TAG, "W5500 filters in IP multicast frames only!");
    } else {
        ESP_LOGW(TAG, "W5500 always filters in IPv6 multicast frames!");
    }
    return ESP_OK;
}

static esp_err_t emac_w5500_init(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_w5500_t *emac = __containerof(mac, emac_w5500_t, base.parent);
    esp_eth_mediator_t *eth = emac->base.eth;
    if (emac->base.int_gpio_num >= 0) {
        gpio_func_sel(emac->base.int_gpio_num, PIN_FUNC_GPIO);
        gpio_input_enable(emac->base.int_gpio_num);
        gpio_pullup_en(emac->base.int_gpio_num);
        gpio_set_intr_type(emac->base.int_gpio_num, GPIO_INTR_NEGEDGE); // active low
        gpio_intr_enable(emac->base.int_gpio_num);
        gpio_isr_handler_add(emac->base.int_gpio_num, wiznet_isr_handler, emac);
    }
    ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL), err, TAG, "lowlevel init failed");
    /* reset w5500 */
    ESP_GOTO_ON_ERROR(w5500_reset(emac), err, TAG, "reset w5500 failed");
    /* verify chip id */
    ESP_GOTO_ON_ERROR(w5500_verify_id(emac), err, TAG, "verify chip ID failed");
    /* default setup of internal registers */
    ESP_GOTO_ON_ERROR(w5500_setup_default(emac), err, TAG, "w5500 default setup failed");
    return ESP_OK;
err:
    if (emac->base.int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->base.int_gpio_num);
        gpio_reset_pin(emac->base.int_gpio_num);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ret;
}

esp_eth_mac_t *esp_eth_mac_new_w5500(const eth_w5500_config_t *w5500_config, const eth_mac_config_t *mac_config)
{
    esp_eth_mac_t *ret = NULL;
    emac_w5500_t *emac = NULL;
    ESP_GOTO_ON_FALSE(w5500_config && mac_config, NULL, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE((w5500_config->int_gpio_num >= 0) != (w5500_config->poll_period_ms > 0), NULL, err, TAG, "invalid configuration argument combination");
    emac = calloc(1, sizeof(emac_w5500_t));
    ESP_GOTO_ON_FALSE(emac, NULL, err, TAG, "no mem for MAC instance");
    /* bind methods and attributes */
    emac->base.tag = TAG;
    emac->base.ops = &w5500_ops;
    emac->base.sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
    emac->base.tx_tmo = W5500_100M_TX_TMO_US;  // default to 100Mbps timeout
    emac->base.int_gpio_num = w5500_config->int_gpio_num;
    emac->base.poll_period_ms = w5500_config->poll_period_ms;
    emac->base.parent.set_mediator = emac_wiznet_set_mediator;
    emac->base.parent.init = emac_w5500_init;
    emac->base.parent.deinit = emac_wiznet_deinit;
    emac->base.parent.start = emac_w5500_start;
    emac->base.parent.stop = emac_w5500_stop;
    emac->base.parent.del = emac_wiznet_del;
    emac->base.parent.write_phy_reg = emac_w5500_write_phy_reg;
    emac->base.parent.read_phy_reg = emac_w5500_read_phy_reg;
    emac->base.parent.set_addr = emac_w5500_set_addr;
    emac->base.parent.get_addr = emac_wiznet_get_addr;
    emac->base.parent.add_mac_filter = emac_w5500_add_mac_filter;
    emac->base.parent.rm_mac_filter = emac_w5500_del_mac_filter;
    emac->base.parent.set_speed = emac_w5500_set_speed;
    emac->base.parent.set_duplex = emac_wiznet_set_duplex;
    emac->base.parent.set_link = emac_wiznet_set_link;
    emac->base.parent.set_promiscuous = emac_w5500_set_promiscuous;
    emac->base.parent.set_all_multicast = emac_w5500_set_all_multicast;
    emac->base.parent.set_peer_pause_ability = emac_wiznet_set_peer_pause_ability;
    emac->base.parent.enable_flow_ctrl = emac_wiznet_enable_flow_ctrl;
    emac->base.parent.transmit = emac_wiznet_transmit;
    emac->base.parent.receive = emac_wiznet_receive;

    if (w5500_config->custom_spi_driver.init != NULL && w5500_config->custom_spi_driver.deinit != NULL
            && w5500_config->custom_spi_driver.read != NULL && w5500_config->custom_spi_driver.write != NULL) {
        ESP_LOGD(TAG, "Using user's custom SPI Driver");
        emac->base.spi.init = w5500_config->custom_spi_driver.init;
        emac->base.spi.deinit = w5500_config->custom_spi_driver.deinit;
        emac->base.spi.read = w5500_config->custom_spi_driver.read;
        emac->base.spi.write = w5500_config->custom_spi_driver.write;
        /* Custom SPI driver device init */
        ESP_GOTO_ON_FALSE((emac->base.spi.ctx = emac->base.spi.init(w5500_config->custom_spi_driver.config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    } else {
        ESP_LOGD(TAG, "Using default SPI Driver");
        emac->base.spi.init = wiznet_spi_init;
        emac->base.spi.deinit = wiznet_spi_deinit;
        emac->base.spi.read = wiznet_spi_read;
        emac->base.spi.write = wiznet_spi_write;
        /* SPI device init */
        ESP_GOTO_ON_FALSE((emac->base.spi.ctx = emac->base.spi.init(w5500_config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    }

    /* create w5500 task */
    BaseType_t core_num = tskNO_AFFINITY;
    if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
        core_num = esp_cpu_get_core_id();
    }
    BaseType_t xReturned = xTaskCreatePinnedToCore(emac_wiznet_task, "w5500_tsk", mac_config->rx_task_stack_size, emac,
                                                   mac_config->rx_task_prio, &emac->base.rx_task_hdl, core_num);
    ESP_GOTO_ON_FALSE(xReturned == pdPASS, NULL, err, TAG, "create w5500 task failed");

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
