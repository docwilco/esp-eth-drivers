set(priv_requires esp_driver_spi)

idf_component_register(SRCS "src/wiznet_spi.c"
                       PRIV_REQUIRES ${priv_requires}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src")
