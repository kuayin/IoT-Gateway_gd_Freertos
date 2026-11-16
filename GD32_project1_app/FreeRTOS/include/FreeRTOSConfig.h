/**
 * ============================================================================
 * @file    FreeRTOSConfig.h
 * @brief   GD32F103 + FreeRTOS 工业级配置
 * @target  GD32F103 (Cortex-M3)，系统主频 108 MHz
 * @note
 *   本配置文件按工业级标准组织，关键点：
 *     1. 中断优先级映射符合 Cortex-M3 的 4 位抢占优先级
 *     2. 开启栈溢出检测、堆分配失败钩子（配合 main.c 中的钩子函数）
 *     3. 开启互斥量、递归互斥量、计数信号量
 *     4. configASSERT 主动触发看门狗复位而非死循环
 *     5. 堆大小按 C8T6(20KB) 的预算给出，其他型号请参考注释自行调整
 *
 *   ⚠ 重要：本配置需要 main.c 提供以下钩子函数，否则链接报错：
 *       - void vAssertCalled(const char *pcFile, unsigned long ulLine);
 *       - void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
 *       - void vApplicationMallocFailedHook(void);
 *     具体实现见 main.c 补充片段。
 * ============================================================================
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ============================================================================
 *                       一、时钟与节拍（Tick）
 * ============================================================================ */

/** CPU 主频（Hz）：GD32F103 最高 108 MHz */
#define configCPU_CLOCK_HZ                        ( ( unsigned long ) 108000000 )

/** Tick 频率（Hz）：1000 → 1 ms 一个系统节拍，调度精度 1 ms */
#define configTICK_RATE_HZ                        ( ( TickType_t ) 1000 )

/** 使用 32 位 tick 计数器（0=32位，1=16位）。
 *  16 位 tick 在 1ms 节拍下 65.5s 即溢出，不适合长延时，故关闭。 */
#define configUSE_16_BIT_TICKS                    0

/** 最小任务栈（单位：word）。
 *  Cortex-M3 上 1 word = 4 字节，128 words = 512 字节。
 *  仅为空闲任务的默认栈；应用任务可按需分配更大值。 */
#define configMINIMAL_STACK_SIZE                  ( ( unsigned short ) 128 )

/** 任务名最大长度（含 '\0'），16 字节足够调试识别 */
#define configMAX_TASK_NAME_LEN                   ( 16 )


/* ============================================================================
 *                       二、调度器行为
 * ============================================================================ */

/** 抢占式调度（工业设计必须开启，否则实时性无法保证） */
#define configUSE_PREEMPTION                      1

/** 空闲任务钩子：不使用 → 0（开启后空闲任务每轮都会调用 vApplicationIdleHook） */
#define configUSE_IDLE_HOOK                       0

/** Tick 钩子：不使用 → 0（开启后每个 tick 中断都会调用 vApplicationTickHook） */
#define configUSE_TICK_HOOK                       0

/** 任务优先级档位：0..6 共 7 级。
 *  配置了 PORT_OPTIMISED_TASK_SELECTION=1 时，此值上限为 32。 */
#define configMAX_PRIORITIES                      ( 7 )

/** 同优先级任务间的空闲让出：1=空闲任务让出 CPU 给同优先级应用任务，
 *  提高无实际负载时的响应性。 */
#define configIDLE_SHOULD_YIELD                   1

/** 使用 Cortex-M3 硬件前导零指令加速任务选择。
 *  需要 configMAX_PRIORITIES <= 32。 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION   1

/** 向后兼容 7.x 之前的宏和 API（保留 vTaskDelayUntil 等旧名称的兼容性） */
#define configENABLE_BACKWARD_COMPATIBILITY       1


/* ============================================================================
 *                       三、内存管理
 * ============================================================================ */

/** FreeRTOS 堆大小（字节）。
 *  本工程任务栈合计：512 + 384 + 128 = 1024 words ≈ 4 KB，
 *  加 TCB、信号量、队列等控制块 ≈ 2 KB，
 *  DMA 缓冲 1 KB（静态分配，不从堆分配），
 *  主栈(MSP) 1-2 KB，全局变量 ≈ 1 KB，
 *  C8T6 (20 KB SRAM) 下取 8 KB 可留约 4-5 KB 安全余量。
 *  若你的芯片是 RB (32 KB) 或更大，可将此值提高到 12-16 KB。 */
#define configTOTAL_HEAP_SIZE                     ( ( size_t ) ( 8 * 1024 ) )

/** 使用 heap_4 的内存管理方案（在 FreeRTOS/Source/portable/MemMang/
 *  heap_4.c 中实现），支持内存碎片合并，适合长期运行。
 *  这里不通过宏配置，需在工程中添加 heap_4.c 到编译列表。 */

/** 堆分配失败钩子：1=开启。
 *  pvPortMalloc 失败时自动调用 vApplicationMallocFailedHook()，
 *  便于记录故障或触发复位。 */
#define configUSE_MALLOC_FAILED_HOOK              1

/** 栈溢出检测方式：
 *  0 = 不检测
 *  1 = 上下文切换时检查栈指针是否越界
 *  2 = 1 + 栈末填充魔数校验（推荐，可检测"栈增长未越界但已腐蚀"的隐蔽故障） */
#define configCHECK_FOR_STACK_OVERFLOW            2


/* ============================================================================
 *                       四、同步原语
 * ============================================================================ */

/** 互斥量（Mutex）：依赖此项，本工程已使用 */
#define configUSE_MUTEXES                         1

/** 递归互斥量：同一任务可重复获取，适合嵌套调用场景 */
#define configUSE_RECURSIVE_MUTEXES               1

/** 计数信号量：比二值信号量更通用，便于事件计数 */
#define configUSE_COUNTING_SEMAPHORES             1


/* ============================================================================
 *                       五、调试与追踪
 * ============================================================================ */

/** 开启追踪功能（uxTaskGetSystemState、vTaskList 等 API 依赖） */
#define configUSE_TRACE_FACILITY                  1

/** 队列/信号量注册表容量：可注册 8 个具名队列，便于调试器实时观察 */
#define configQUEUE_REGISTRY_SIZE                 8

/** 空闲任务与任务 tick 计数是否使能（若需 vTaskGetRunTimeStats 之类的功能，需要硬件定时器配合） */
#define configGENERATE_RUN_TIME_STATS             0


/* ============================================================================
 *                       六、中断优先级（Cortex-M3 关键配置）
 * ============================================================================ */

/** 内核最低优先级（数值越大越不紧急）。
 *  Cortex-M3 有 16 级可编程优先级（0-15），15 为最低。
 *  这是 PendSV、SysTick 使用的优先级。 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY          15

/** 可调用 FromISR API 的中断最高优先级（数值上必须 >= 5）。
 *  即抢占优先级 0..4 的中断不能调用 FromISR API（会触发断言），
 *  抢占优先级 5..15 的中断可以安全调用。
 *
 *  ⚠ 本工程 USART0_IRQHandler 使用了 xSemaphoreGiveFromISR，
 *    main.c 中通过 nvic_irq_enable(USART0_IRQn, 5, 0) 设置其抢占优先级为 5，
 *    恰好在此阈值之上，符合要求。 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     5

/** 内核优先级寄存器值：LIBRARY 值左移到 STM32/GD32 的高 4 位。
 *  FreeRTOS 的 PendSV/SysTick 使用该值。 */
#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - 4 ) )

/** 系统调用基线：ISR 中调用 FromISR 的判定阈值 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - 4 ) )


/* ============================================================================
 *                       七、必须的 API 声明
 * ============================================================================ */
/* FreeRTOS 采用"按需编译"策略：不声明则不编译对应函数，节省代码空间。
 * 以下各项为本工程或常见调试所需，按需保留。 */

#define INCLUDE_vTaskPrioritySet                  1   /* vTaskPrioritySet 动态调优先级 */
#define INCLUDE_uxTaskPriorityGet                 1   /* uxTaskPriorityGet 查询优先级 */
#define INCLUDE_vTaskDelete                       1   /* vTaskDelete 删除任务 */
#define INCLUDE_vTaskSuspend                      1   /* vTaskSuspend/Resume 挂起/恢复 */
#define INCLUDE_vTaskDelay                        1   /* vTaskDelay 相对延时 */
#define INCLUDE_xTaskDelayUntil                   1   /* xTaskDelayUntil 绝对周期延时 */
#define INCLUDE_xTaskGetSchedulerState            1   /* 查询调度器状态 */


/* ============================================================================
 *                       八、异常与中断映射
 * ============================================================================ */
/* 将 FreeRTOS 内部实现映射到 GD32 启动文件中已有的异常名。
 * 若不映射，这些异常会指向启动文件中的默认死循环。 */

#define vPortSVCHandler         SVC_Handler
#define xPortPendSVHandler      PendSV_Handler
#define xPortSysTickHandler     SysTick_Handler


/* ============================================================================
 *                       九、断言（工业级实现）
 * ============================================================================ */
/* 工业现场对 configASSERT 的要求：
 *   1) 必须能在故障时记录现场（文件、行号），便于故障回溯
 *   2) 不能无限死循环（会触发看门狗，但记录信息必须持久化）
 *   3) 主动等待 FWDGT 复位比空转死等更可靠
 *
 * 具体实现见 main.c 中的 vAssertCalled()。 */

extern void vAssertCalled( const char *pcFile, unsigned long ulLine );

#define configASSERT( x ) \
    if( ( x ) == 0 ) vAssertCalled( __FILE__, __LINE__ )


#endif /* FREERTOS_CONFIG_H */