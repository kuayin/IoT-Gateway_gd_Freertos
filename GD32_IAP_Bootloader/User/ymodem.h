#ifndef YMODEM_H
#define YMODEM_H

#include <stdint.h>

#define W25Q64_UPDATE_START_ADDR    0x00100000U

extern uint32_t fw_total_size;

typedef enum {
    YMODEM_OK = 0,
    YMODEM_ERR_TIMEOUT,
    YMODEM_ERR_CANCEL,
    YMODEM_ERR_FLASH,
} ymodem_status_t;

ymodem_status_t ymodem_receive(void);

#endif