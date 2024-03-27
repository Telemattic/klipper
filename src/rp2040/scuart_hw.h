#pragma once

#include <stdint.h>

struct scuart_hw;
typedef void (*scuart_hw_callback)(struct scuart_hw*, uint32_t);

struct scuart_hw {
    void* pio;
    uint32_t pin;
    uint8_t flags;
    uint8_t wr_data[64], wr_pos, wr_len;
    uint8_t rd_data[64], rd_len;
    scuart_hw_callback callback;
};

enum {
    SCUART_FLAG_WRITE = 1 << 0
};

enum {
    SCUART_NOTIFY_TX = 1 << 21
};

void scuart_hw_init(struct scuart_hw* u, uint32_t pio_num, uint32_t pin, uint32_t baud);
void scuart_hw_irq_handler(struct scuart_hw* u);

void scuart_hw_send(struct scuart_hw* u, uint8_t len, uint8_t* data);
uint8_t scuart_hw_recv(struct scuart_hw* u, uint8_t len, uint8_t* data);
