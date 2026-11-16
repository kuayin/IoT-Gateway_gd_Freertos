#include "gd32f10x.h"
#include "boot_uart.h"
#include "w25q64.h"
#include <stdio.h>
#include "ymodem.h"
#include "iap.h"

#define UPGRADE_MAGIC_FLAG  0x5A5AA5A5U

/* 检查 APP 是否合法（栈顶指针是否在 SRAM 范围内） */
static uint8_t is_app_valid(void)
{
    uint32_t app_stack = *(volatile uint32_t *)APP_ADDRESS;
    if ((app_stack & 0x2FFE0000U) == 0x20000000U) {
        return 1; // 合法
    }
    return 0; // 非法或未烧录
}

/* 检查并清除 BKP 升级标志 */
static uint8_t check_upgrade_flag(void)
{
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_BKPI);
    
    uint32_t flag = (bkp_data_read(BKP_DATA_1) << 16) | bkp_data_read(BKP_DATA_0);
    
    if (flag == UPGRADE_MAGIC_FLAG) {
        /* 清除标志，防止下次上电又进 Bootloader */
        pmu_backup_write_enable();
        bkp_data_write(BKP_DATA_0, 0x0000);
        bkp_data_write(BKP_DATA_1, 0x0000);
        return 1; // 有升级请求
    }
    return 0;
}

int main(void)
{
    boot_uart_init(115200);
    printf("\r\n\r\n[BTL] Bootloader Starting...\r\n");

    /* 1. 判断是 APP 请求升级，还是正常上电 */
    uint8_t need_upgrade = check_upgrade_flag();
    
    if (!need_upgrade) {
        /* 如果是正常上电，且 APP 合法，直接秒跳转，不浪费时间 */
        if (is_app_valid()) {
            printf("[BTL] Normal boot. Jumping to APP...\r\n");
            jump_to_app();
        } else {
            /* APP 无效（比如第一次烧录，或 APP 坏了），强制进入升级模式 */
            printf("[BTL] Invalid APP! Forced upgrade mode.\r\n");
        }
    } else {
        printf("[BTL] OTA Flag detected! Entering upgrade mode.\r\n");
    }

    /* 2. 只有升级标志有效 或 APP 无效时，才会走到这里 */
    if (w25q64_bus_init() != W25Q64_OK || w25q64_init() != W25Q64_OK) {
        printf("[BTL] SPI Flash Error! Cannot upgrade.\r\n");
        while(1); // 硬件故障，死机等待排查
    }

    printf("[BTL] Waiting for Ymodem upgrade...\r\n");
    
    /* 3. 接收固件并写入外部 Flash */
    if (ymodem_receive() == YMODEM_OK) {
        printf("[BTL] Firmware saved. Size: %lu\r\n", (unsigned long)fw_total_size);
        
        /* 4. 搬运到内部 Flash */
        if (iap_copy_firmware(fw_total_size) == 0) {
            printf("[BTL] IAP success! Rebooting system...\r\n");
            while (RESET == usart_flag_get(USART0, USART_FLAG_TC)); // 等打印完
            NVIC_SystemReset(); // 搬运成功，软复位，重新走正常启动流程进入新 APP
        } else {
            printf("[BTL] IAP Copy Failed! Staying in Bootloader.\r\n");
        }
    } else {
        printf("[BTL] Upgrade Failed or Canceled.\r\n");
    }

    /* 升级失败或取消，死循环等待用户重新发送 */
    while (1);
}