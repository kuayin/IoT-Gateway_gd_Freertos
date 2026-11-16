#include "bootloader_start.h"


void app_trigger_ota(void)
{
    
    /* 1. 开启 PMU 和 BKP 时钟 */
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_BKPI);
    
    /* 2. 允许写入 BKP 寄存器 */
    pmu_backup_write_enable();
    
    /* 3. 写入 32 位魔数标志 */
    bkp_data_write(BKP_DATA_0, (uint16_t)(UPGRADE_MAGIC_FLAG & 0xFFFF));
    bkp_data_write(BKP_DATA_1, (uint16_t)(UPGRADE_MAGIC_FLAG >> 16));
    
    /* 4. 等待串口最后一个字节发完，防止复位时打印乱码 */
    while (RESET == usart_flag_get(USART0, USART_FLAG_TC));
    
    /* 5. 触发系统软复位 */
    NVIC_SystemReset();
}