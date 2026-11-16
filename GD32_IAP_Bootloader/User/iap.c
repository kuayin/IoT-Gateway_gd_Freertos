#include "iap.h"
#include "gd32f10x.h"
#include "w25q64.h"
#include "ymodem.h"
#include <stdio.h>

typedef void (*pFunction)(void);

/* 必须定义在全局，如果定义在函数内会导致栈溢出死机 */
static uint8_t iap_buffer[1024]; 

int iap_copy_firmware(uint32_t fw_size)
{
    uint32_t i, j;
    uint32_t write_addr = APP_ADDRESS;
    uint32_t read_addr = W25Q64_UPDATE_START_ADDR;

    printf("[IAP] Start Copy, Size: %lu\r\n", (unsigned long)fw_size);

    if (fw_size == 0 || fw_size > APP_MAX_SIZE) {
        printf("[IAP] Size Error!\r\n");
        return -1;
    }

    printf("[IAP] Unlocking Flash...\r\n");
    fmc_unlock();

    printf("[IAP] Erasing Flash...\r\n");
    for (i = 0; i < fw_size; i += 1024) {
        fmc_page_erase(APP_ADDRESS + i);
    }
    printf("[IAP] Erase Done.\r\n");

    printf("[IAP] Copying Data...\r\n");
    for (i = 0; i < fw_size; i += 1024) {
        uint32_t read_len = (fw_size - i > 1024) ? 1024 : (fw_size - i);
        
        w25q64_read(read_addr + i, iap_buffer, read_len);

        for (j = 0; j < read_len; j += 4) {
            uint32_t word_data = iap_buffer[j] | 
                                 (iap_buffer[j+1] << 8) | 
                                 (iap_buffer[j+2] << 16) | 
                                 (iap_buffer[j+3] << 24);
            fmc_word_program(write_addr, word_data);
            write_addr += 4;
        }
    }

    fmc_lock();
    printf("[IAP] Copy Complete! Ready to Jump.\r\n");
    return 0;
}

void jump_to_app(void)
{
    uint32_t app_stack = *(volatile uint32_t *)APP_ADDRESS;
    uint32_t app_entry = *(volatile uint32_t *)(APP_ADDRESS + 4U);

    if ((app_stack & 0x2FFE0000U) != 0x20000000U) {
        printf("[IAP] ERROR: Invalid APP Stack!\r\n");
        return;
    }

    printf("[IAP] Jumping to APP...\r\n");

    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    for (uint8_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    
    __set_MSP(app_stack);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TC)) { }
    pFunction app_reset = (pFunction)app_entry;
    app_reset();
    
    while (1);
}