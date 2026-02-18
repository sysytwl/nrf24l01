#pragma once
#include <stdint.h>

void nrf_rx_init(void);
void nrf_rx_tick(uint8_t *data);
