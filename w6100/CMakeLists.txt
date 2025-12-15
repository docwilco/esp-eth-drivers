set(priv_requires esp_eth esp_driver_gpio esp_driver_spi esp_timer)

idf_component_register(SRCS "src/w6100_mac.c"
                            "src/w6100_phy.c"
                            "src/w6100_init.c"
                       PRIV_REQUIRES ${priv_requires}
                       INCLUDE_DIRS "include")
