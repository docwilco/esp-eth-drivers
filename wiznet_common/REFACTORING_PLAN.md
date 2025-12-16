# WIZnet Common Component Refactoring Plan

## Overview

This document outlines the plan to extract common code from the W5500 and W6100 ESP-IDF Ethernet drivers into a shared `wiznet_common` component. The goal is to reduce code duplication while maintaining full functionality and API compatibility.

## Approach

1. Start by moving shared code from W6100 to `wiznet_common`
2. Test W6100 with the shared component
3. Update W5500 to use the shared component
4. The W5500 public API must remain unchanged
5. W6100 API can be adjusted to align with W5500 where needed

## Code Analysis

### Highly Similar Code (MAC Layer)

The following sections are nearly identical between W5500 and W6100 and are candidates for direct extraction:

| Function/Type | W5500 | W6100 | Notes |
|---------------|-------|-------|-------|
| `eth_spi_info_t` | ✓ | ✓ | Identical struct |
| `eth_spi_custom_driver_t` | ✓ | ✓ | Identical struct |
| `emac_wXXXX_auto_buf_info_t` | ✓ | ✓ | Identical struct, rename to generic |
| `wXXXX_spi_init()` | ✓ | ✓ | Nearly identical, parameterize config type |
| `wXXXX_spi_deinit()` | ✓ | ✓ | Identical |
| `wXXXX_spi_lock/unlock()` | ✓ | ✓ | Identical |
| `wXXXX_spi_read/write()` | ✓ | ✓ | Identical |
| `wXXXX_isr_handler()` | ✓ | ✓ | Identical |
| `wXXXX_poll_timer()` | ✓ | ✓ | Identical |
| `emac_wXXXX_set_mediator()` | ✓ | ✓ | Identical |
| `emac_wXXXX_set_addr()` | ✓ | ✓ | Identical |
| `emac_wXXXX_get_addr()` | ✓ | ✓ | Identical |
| `emac_wXXXX_set_speed()` | ✓ | ✓ | Identical (just timeout values differ) |
| `emac_wXXXX_set_duplex()` | ✓ | ✓ | Identical |
| `emac_wXXXX_enable_flow_ctrl()` | ✓ | ✓ | Identical |
| `emac_wXXXX_set_peer_pause_ability()` | ✓ | ✓ | Identical |
| `emac_wXXXX_set_link()` | ✓ | ✓ | Nearly identical |
| `emac_wXXXX_deinit()` | ✓ | ✓ | Nearly identical |
| `emac_wXXXX_del()` | ✓ | ✓ | Identical |

### Similar with Chip-Specific Differences (MAC Layer)

These functions share the same structure but have chip-specific register addresses or behaviors:

| Function | Differences |
|----------|-------------|
| `wXXXX_read/write()` | Register address encoding differs slightly |
| `wXXXX_send_command()` | Uses chip-specific command register |
| `wXXXX_get_tx_free_size()` | Uses chip-specific register |
| `wXXXX_get_rx_received_size()` | Uses chip-specific register |
| `wXXXX_write_buffer()` | Uses chip-specific buffer addresses |
| `wXXXX_read_buffer()` | Uses chip-specific buffer addresses |
| `wXXXX_set_mac_addr()` | Register address differs (W5500_REG_MAC vs W6100_REG_SHAR) |
| `wXXXX_reset()` | Different reset mechanism |
| `wXXXX_verify_id()` | Different chip ID registers/values |
| `wXXXX_setup_default()` | Different register setup |
| `emac_wXXXX_start()` | Uses chip-specific registers |
| `emac_wXXXX_stop()` | Uses chip-specific registers |
| `emac_wXXXX_transmit()` | Similar structure, different registers |
| `emac_wXXXX_receive()` | Similar structure, different registers |
| `emac_wXXXX_alloc_recv_buf()` | Similar structure, different registers |
| `emac_wXXXX_flush_recv_frame()` | Similar structure, different registers |
| `emac_wXXXX_task()` | Similar structure, different registers |
| `emac_wXXXX_init()` | Similar structure, different registers |
| `emac_wXXXX_write_phy_reg()` | Different PHY register handling |
| `emac_wXXXX_read_phy_reg()` | Different PHY register handling |
| `emac_wXXXX_set_promiscuous()` | Different register bits |
| `esp_eth_mac_new_wXXXX()` | Constructor - mostly identical structure |

### W5500-Only Functions

| Function | Notes |
|----------|-------|
| `emac_w5500_set_block_ip4_mcast()` | W5500-specific multicast handling |
| `emac_w5500_add_mac_filter()` | W5500-specific |
| `emac_w5500_del_mac_filter()` | W5500-specific |
| `emac_w5500_set_all_multicast()` | W5500-specific |

### PHY Layer Comparison

| Function | W5500 | W6100 | Notes |
|----------|-------|-------|-------|
| `wXXXX_set_mediator()` | ✓ | ✓ | Identical |
| `wXXXX_get_link()` | ✓ | ✓ | Identical |
| `wXXXX_set_link()` | ✓ | ✓ | Identical |
| `wXXXX_update_link_duplex_speed()` | ✓ | ✓ | Bit polarity differs |
| `wXXXX_reset()` | ✓ | ✓ | Register differs |
| `wXXXX_reset_hw()` | ✓ | ✓ | Different GPIO APIs used |
| `wXXXX_autonego_ctrl()` | ✓ | ✓ | Register layout differs |
| `wXXXX_pwrctl()` | ✓ | ✓ | W5500 no-op, W6100 has power down |
| `wXXXX_set_addr()` | ✓ | ✓ | Identical |
| `wXXXX_get_addr()` | ✓ | ✓ | Identical |
| `wXXXX_del()` | ✓ | ✓ | Identical |
| `wXXXX_advertise_pause_ability()` | ✓ | ✓ | Both no-op |
| `wXXXX_loopback()` | ✓ | ✓ | Both not supported |
| `wXXXX_set_speed()` | ✓ | ✓ | Register layout differs |
| `wXXXX_set_duplex()` | ✓ | ✓ | Register layout differs |
| `wXXXX_init()` | ✓ | ✓ | Similar structure |
| `esp_eth_phy_new_wXXXX()` | ✓ | ✓ | Mostly identical |

### Register/Constant Comparison

The SPI frame format is identical:
- Address phase: 16 bits
- Control phase: 8 bits (BSB[4:0] + RWB + OM[1:0])
- Data phase: Variable or fixed length

BSB (Block Select Bits) encoding is identical:
- Common register: 0x00
- Socket n register: (n << 2) + 1
- Socket n TX buffer: (n << 2) + 2
- Socket n RX buffer: (n << 2) + 3

## Proposed Architecture

### wiznet_common Component Structure

```
wiznet_common/
├── CMakeLists.txt
├── idf_component.yml
├── include/
│   └── wiznet_common.h          # Public types and shared declarations
└── src/
    ├── wiznet_spi.c             # SPI driver (init/deinit/read/write/lock)
    ├── wiznet_spi.h             # Internal SPI types
    ├── wiznet_mac_common.c      # Shared MAC functions
    ├── wiznet_mac_common.h      # Internal MAC types and ops struct
    └── wiznet_phy_common.c      # Shared PHY functions (if sufficient)
```

### Abstraction Strategy

Use a chip operations structure to abstract chip-specific behavior:

```c
// In wiznet_mac_common.h (internal)
typedef struct {
    // Chip identification
    const char *name;
    
    // Register addresses (as functions for socket-parameterized registers)
    uint32_t (*reg_sock_cr)(int sock);
    uint32_t (*reg_sock_ir)(int sock);
    uint32_t (*reg_sock_imr)(int sock);
    uint32_t (*reg_simr)(void);
    uint32_t (*reg_sock_tx_wr)(int sock);
    uint32_t (*reg_sock_tx_fsr)(int sock);
    uint32_t (*reg_sock_rx_rd)(int sock);
    uint32_t (*reg_sock_rx_rsr)(int sock);
    uint32_t (*reg_sock_mr)(int sock);
    uint32_t (*mem_sock_tx)(int sock, uint16_t addr);
    uint32_t (*mem_sock_rx)(int sock, uint16_t addr);
    
    // Chip-specific operations
    esp_err_t (*reset)(void *emac);
    esp_err_t (*verify_id)(void *emac);
    esp_err_t (*setup_default)(void *emac);
    esp_err_t (*set_mac_addr)(void *emac);
    bool (*is_sane_for_rxtx)(void *emac);
    esp_err_t (*clear_tx_done)(void *emac);
    esp_err_t (*clear_rx_done)(void *emac);
    
    // Register values
    uint8_t scr_open;
    uint8_t scr_close;
    uint8_t scr_send;
    uint8_t scr_recv;
    uint8_t sir_recv;
    uint8_t sir_send;
    uint8_t simr_sock0;
    
    // Timeouts
    uint32_t tx_tmo_10m;
    uint32_t tx_tmo_100m;
} wiznet_chip_ops_t;

// Common EMAC structure
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
    const wiznet_chip_ops_t *ops;  // Chip-specific operations
    void *chip_data;               // Chip-specific extra data (e.g., mcast_cnt for W5500)
} emac_wiznet_t;
```

## Implementation Phases

### Phase 1: Create wiznet_common Component (Foundation)

1. Create component directory structure
2. Add `CMakeLists.txt` and `idf_component.yml`
3. Move SPI driver code (types and functions):
   - `eth_spi_info_t`
   - `eth_spi_custom_driver_t`
   - `wiznet_spi_init()` (from `wXXXX_spi_init`)
   - `wiznet_spi_deinit()`
   - `wiznet_spi_lock()`
   - `wiznet_spi_unlock()`
   - `wiznet_spi_read()`
   - `wiznet_spi_write()`
4. Create public header with shared types

### Phase 2: Extract Common MAC Code

1. Create `wiznet_chip_ops_t` structure
2. Create common emac structure `emac_wiznet_t`
3. Move identical MAC functions:
   - `emac_wiznet_set_mediator()`
   - `emac_wiznet_set_addr()`
   - `emac_wiznet_get_addr()`
   - `emac_wiznet_set_speed()`
   - `emac_wiznet_set_duplex()`
   - `emac_wiznet_enable_flow_ctrl()`
   - `emac_wiznet_set_peer_pause_ability()`
   - `emac_wiznet_set_link()`
   - `emac_wiznet_del()`
   - `emac_wiznet_deinit()`
4. Move ISR and poll timer handlers

### Phase 3: Abstract Chip-Specific MAC Operations

1. Create chip ops registration for W6100
2. Move/refactor:
   - `wiznet_read()` / `wiznet_write()` (parameterized by ops)
   - `wiznet_send_command()` (uses ops for register)
   - `emac_wiznet_transmit()` (uses ops for registers)
   - `emac_wiznet_receive()` (uses ops for registers)
   - `emac_wiznet_alloc_recv_buf()`
   - `emac_wiznet_flush_recv_frame()`
   - `emac_wiznet_task()` (uses ops for registers)
   - `emac_wiznet_init()` (calls ops for chip-specific init)

### Phase 4: Update W6100 to Use wiznet_common

1. Update W6100 `CMakeLists.txt` to depend on `wiznet_common`
2. Remove duplicated code from W6100
3. Implement W6100-specific ops structure
4. Update `esp_eth_mac_new_w6100()` to use shared code
5. **Test W6100 thoroughly**

### Phase 5: Update W5500 to Use wiznet_common

1. Update W5500 `CMakeLists.txt` to depend on `wiznet_common`
2. Remove duplicated code from W5500
3. Implement W5500-specific ops structure
4. Keep W5500-specific multicast functions in w5500 component
5. Update `esp_eth_mac_new_w5500()` to use shared code
6. **Ensure W5500 public API remains unchanged**
7. **Test W5500 thoroughly**

### Phase 6: PHY Refactoring (Optional/Later)

The PHY layer has less code and more chip-specific differences (bit polarities, register layouts). Consider:
- Extract only truly common functions (set_mediator, get/set_addr, del)
- Or keep PHY code chip-specific for clarity

## Testing Strategy

After each phase, verify:

1. **Build test**: Both components compile without errors
2. **Functional test**: Use eth-test application
   - Test with W6100 (after Phase 4)
   - Test with W5500 (after Phase 5)
   - Test both polling and interrupt modes
   - Test link up/down detection
   - Test TX/RX functionality
   - Test speed/duplex negotiation

## Files to Create/Modify

### New Files (wiznet_common)
- `esp-eth-drivers/wiznet_common/CMakeLists.txt`
- `esp-eth-drivers/wiznet_common/idf_component.yml`
- `esp-eth-drivers/wiznet_common/include/wiznet_common.h`
- `esp-eth-drivers/wiznet_common/src/wiznet_spi.c`
- `esp-eth-drivers/wiznet_common/src/wiznet_spi.h`
- `esp-eth-drivers/wiznet_common/src/wiznet_mac_common.c`
- `esp-eth-drivers/wiznet_common/src/wiznet_mac_common.h`

### Modified Files (W6100)
- `esp-eth-drivers/w6100/CMakeLists.txt` - Add dependency
- `esp-eth-drivers/w6100/idf_component.yml` - Add dependency
- `esp-eth-drivers/w6100/src/esp_eth_mac_w6100.c` - Use shared code
- `esp-eth-drivers/w6100/include/esp_eth_mac_w6100.h` - May need adjustments

### Modified Files (W5500)
- `esp-eth-drivers/w5500/CMakeLists.txt` - Add dependency
- `esp-eth-drivers/w5500/idf_component.yml` - Add dependency
- `esp-eth-drivers/w5500/src/esp_eth_mac_w5500.c` - Use shared code

## Open Questions

1. **Naming convention**: Use `wiznet_` prefix or something else?
2. **PHY code**: Extract common PHY code or keep separate?
3. **W6100 API changes**: Any specific W6100 API changes needed to align with W5500?

## Risk Mitigation

1. **Incremental approach**: Complete each phase and test before moving to next
2. **Keep original code**: Don't delete until new code is verified
3. **API compatibility**: W5500 public API must not change
4. **Register verification**: Double-check all register addresses against datasheets

## References

- [W5500 Datasheet](https://docs.wiznet.io/Product/iEthernet/W5500/datasheet)
- [W6100 Datasheet](https://docs.wiznet.io/Product/iEthernet/W6100/datasheet)
- [ESP-IDF Ethernet Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_eth.html)
