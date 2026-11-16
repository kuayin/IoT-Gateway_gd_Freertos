/* ============================================================================
 *                     FreeRTOS 钩子函数（工业级实现）
 * ============================================================================ */
#include "gd32f10x.h"
#include "FreeRTOS.h"
#include "task.h"


/**
 * @brief 断言失败回调
 * @note  工业级要求：
 *        1. 关中断，防止后续行为不确定
 *        2. 不喂狗，让 FWDGT 在 2 秒内复位，留下现场
 *        3. 如需记录 file/line，可写入后备寄存器(BKP)或未初始化的 RAM
 *           （复位后读取），或通过阻塞 UART 输出
 */
void vAssertCalled( const char *pcFile, unsigned long ulLine )
{
    (void)pcFile;
    (void)ulLine;

    __disable_irq();    /* 关全局中断 */

    /* 可选：把 pcFile/ulLine 写入未初始化 RAM 段供复位后读取
     * 例如：
     *   extern uint32_t g_assert_line;
     *   extern const char *g_assert_file;
     *   g_assert_file = pcFile;
     *   g_assert_line = ulLine;
     */

    /* 主动等待 FWDGT 复位（不调用 fwdgt_counter_reload()） */
    for ( ;; ) { /* 空转，让 FWDGT 完成复位 */ }
}

/**
 * @brief 栈溢出钩子
 * @note  当检测到栈溢出时调用。此时栈已损坏，
 *        该函数中不能调用任何可能使用栈的复杂函数
 *        （包括 printf、任务 API），只能执行最小动作。
 */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    (void)xTask;
    (void)pcTaskName;

    __disable_irq();
    /* 此处可选：通过最小化方式（例如直接写 GPIO）指示故障 */
    for ( ;; ) { /* 等待 FWDGT 复位 */ }
}

/**
 * @brief 堆分配失败钩子
 * @note  当 configTOTAL_HEAP_SIZE 不够（例如任务栈太大或任务数量过多）时调用。
 *        常见原因：任务栈太大、heap_4 碎片化、任务创建失败未检查返回值。
 */
void vApplicationMallocFailedHook( void )
{
    __disable_irq();
    for ( ;; ) { /* 等待 FWDGT 复位 */ }
}