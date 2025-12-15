# W6100 ESP-IDF Component

ESP-IDF component driver for the WIZnet W6100 hardwired TCP/IP Ethernet controller with IPv4/IPv6 dual-stack support.

## Features

- SPI communication with configurable clock speed (up to 80 MHz)
- Interrupt-driven or polling-based receive
- Full integration with ESP-NETIF
- Custom SPI driver support
- IPv4/IPv6 dual-stack capable hardware (via W6100's hardwired stack)
- MACRAW mode for use with ESP-IDF's software TCP/IP stack

## Hardware Requirements

- WIZnet W6100 Ethernet controller
- SPI interface: MOSI, MISO, SCLK, CS
- Optional: Interrupt pin (INT)
- Optional: Reset pin (RST)

## Supported ESP-IDF Versions

- ESP-IDF v5.3 and later

## Usage

### Installation

Add this component to your project's `idf_component.yml`:

```yaml
dependencies:
  w6100:
    git: https://github.com/drwilco/w6100.git
```

Or use the ESP Component Registry (once published):

```yaml
dependencies:
  w6100: "*"
```

### Basic Example

```c
#include "esp_eth.h"
#include "w6100.h"
#include "driver/spi_master.h"

// SPI bus configuration
spi_bus_config_t buscfg = {
    .mosi_io_num = GPIO_NUM_23,
    .miso_io_num = GPIO_NUM_19,
    .sclk_io_num = GPIO_NUM_18,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};
spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

// W6100 configuration
eth_w6100_config_t w6100_config = ETH_W6100_DEFAULT_CONFIG(
    GPIO_NUM_23,  // MOSI (not used by ETH_W6100_DEFAULT_CONFIG)
    GPIO_NUM_19,  // MISO (not used by ETH_W6100_DEFAULT_CONFIG)
    GPIO_NUM_18,  // SCLK (not used by ETH_W6100_DEFAULT_CONFIG)
    GPIO_NUM_5,   // CS
    GPIO_NUM_4,   // INT (-1 for polling mode)
    SPI2_HOST     // SPI host
);

// MAC configuration
eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
esp_eth_mac_t *mac = esp_eth_mac_new_w6100(&w6100_config, &mac_config);

// PHY configuration
eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
phy_config.reset_gpio_num = GPIO_NUM_17; // Optional reset pin
esp_eth_phy_t *phy = esp_eth_phy_new_w6100(&phy_config);

// Create Ethernet driver
esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
esp_eth_handle_t eth_handle = NULL;
esp_eth_driver_install(&eth_config, &eth_handle);

// Start Ethernet
esp_eth_start(eth_handle);
```

### Polling Mode

If you don't have an interrupt pin available, set `int_gpio_num` to -1:

```c
eth_w6100_config_t w6100_config = {
    .int_gpio_num = -1,
    .poll_period_ms = 10,  // Poll every 10ms
    .spi_host_id = SPI2_HOST,
    .spi_devcfg = &spi_devcfg,
    .custom_spi_driver = NULL,
};
```

### Custom SPI Driver

For advanced use cases, you can provide your own SPI driver:

```c
eth_w6100_custom_spi_driver_t custom_driver = {
    .ctx = my_spi_context,
    .init = my_spi_init,
    .deinit = my_spi_deinit,
    .read = my_spi_read,
    .write = my_spi_write,
};

eth_w6100_config_t w6100_config = {
    .int_gpio_num = GPIO_NUM_4,
    .poll_period_ms = 10,
    .spi_host_id = SPI2_HOST,
    .spi_devcfg = &spi_devcfg,
    .custom_spi_driver = &custom_driver,
};
```

## API Reference

### MAC Functions

#### `esp_eth_mac_new_w6100()`

Create W6100 Ethernet MAC instance.

```c
esp_eth_mac_t *esp_eth_mac_new_w6100(const eth_w6100_config_t *w6100_config,
                                      const eth_mac_config_t *mac_config);
```

**Parameters:**
- `w6100_config`: W6100 specific configuration
- `mac_config`: Ethernet MAC configuration

**Returns:** Pointer to MAC instance on success, NULL on failure.

### PHY Functions

#### `esp_eth_phy_new_w6100()`

Create W6100 PHY instance.

```c
esp_eth_phy_t *esp_eth_phy_new_w6100(const eth_phy_config_t *config);
```

**Parameters:**
- `config`: PHY configuration

**Returns:** Pointer to PHY instance on success, NULL on failure.

## W6100 vs W5500

The W6100 is the successor to the W5500 with added IPv6 support:

| Feature | W5500 | W6100 |
|---------|-------|-------|
| IPv4 | Yes | Yes |
| IPv6 | No | Yes |
| Sockets | 8 | 8 |
| TX/RX Buffer | 32KB | 32KB |
| SPI Speed | 80 MHz | 80 MHz |
| MACRAW Mode | Yes | Yes |

## References

- [W6100 Datasheet](https://docs.wiznet.io/Product/iEthernet/W6100/datasheet)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [ESP-IDF Ethernet Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_eth.html)

## License

Apache-2.0
