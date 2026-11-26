/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_protocol.h"
#include "bt.h"
#include "chassis.h"
#include "arm_robot.h"
#include <stdio.h>
#include <string.h>

/*
 * 特别注意：本文件里"任务相关的一切"（句柄、属性、创建、函数定义）都放在
 * USER CODE 保护块内。因为这些任务没有登记到 .ioc（只有 CubeMX 模板自带的
 * defaultTask 在 .ioc 里），若放在保护块外，重新生成时会被 CubeMX 当成未知
 * 内容删除。放在保护块内则 CubeMX 永远不会触碰。以后每次重新生成都安全。
 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* huart1 访问互斥（print_lock）：
 * - PrintTask 的周期状态打印（printf 走 huart1）；
 * - ProtocolTask 发回 PC 的协议响应帧（目标为 USART1 时）。
 * 二者互斥，保证 PC 串口上响应帧与打印消息不会按字符交错。 */
osMutexId_t print_lock;
const osMutexAttr_t print_mutex_attributes = {
  .name = "printLock"
};

/* ---- 任务的句柄与属性（全在 USER CODE 内，重新生成被保留） ---- */
osThreadId_t ProtocolTaskHandle;
const osThreadAttr_t ProtocolTask_attributes = {
  .name = "ProtocolTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t PrintTaskHandle;
const osThreadAttr_t PrintTask_attributes = {
  .name = "PrintTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* 底盘任务（2×TB6612 + JGB37-520 1:90 四路编码器 + 80mm 麦轮）：
 * 20ms 标称一次，实际 dt 由 CHS_Tick 用 HAL tick 测量。
 * 与 ProtocolTask 同级，靠时间片轮转，互不饿死。 */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* 机械臂任务（六轴总线舵机 on USART3）：10ms 一拍，兼做「执行作业 + 温度/位置/电压轮询」。
 * 优先级刻意低于底盘任务：本任务有阻塞式舵机收发（一轮轮询 25~35ms，
 * J5 背隙补偿两段式可达 1s 以上），绝不能让 20ms 的底盘节拍给它让路。
 * 反过来，协议任务只投递作业不阻塞，所以底盘 500ms 心跳也不会被机械臂拖停。 */
osThreadId_t ArmTaskHandle;
const osThreadAttr_t ArmTask_attributes = {
  .name = "ArmTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartProtocolTask(void *argument);
void StartPrintTask(void *argument);
void StartChassisTask(void *argument);
void StartArmTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  print_lock = osMutexNew(&print_mutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  ProtocolTaskHandle = osThreadNew(StartProtocolTask, NULL, &ProtocolTask_attributes);
  PrintTaskHandle   = osThreadNew(StartPrintTask,   NULL, &PrintTask_attributes);
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);
  ArmTaskHandle     = osThreadNew(StartArmTask,     NULL, &ArmTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  串口协议后台任务：解析命令帧、发送待发响应、重启串口接收。必须高频运行：
  *         中断只收字节入队，命令解析/执行与发送（三路串口同一套逻辑，无模块专用分支）
  *         全部在本任务完成；发回 huart1 的响应与 printf 共用 print_lock 互斥。
  * @param  argument: Not used
  */
void StartProtocolTask(void *argument)
{
  for(;;)
  {
    UartProto_Process();
    osDelay(5);
  }
}

/**
  * @brief  周期遥测打印任务：每 3000ms 输出一次底盘状态（可用 $LOG,1/$LOG,0 开关）。
  *         持 print_lock 打印，避免与 ProtocolTask 响应帧交错。
  * @param  argument: Not used
  */
void StartPrintTask(void *argument)
{
  for(;;)
  {
    osDelay(3000);
    if (UartProto_GetStreamLog())
    {
      float x, y, yaw;

      CHS_GetOdometry(&g_chassis, &x, &y, &yaw);

      osMutexAcquire(print_lock, osWaitForever);
      printf("CHS odom %ldmm %ldmm %lddeg online=%u\r\n",
             (long)x, (long)y, (long)yaw, (unsigned)g_chassis.online);
      osMutexRelease(print_lock);
    }
  }
}

 /**
  * @brief  底盘任务：初始化 2×TB6612 + 4 路定时器编码器，然后 20ms 固定周期跑 CHS_Tick。
  *         初始化只依赖本板定时器/GPIO（纯 GPIO/定时器配置，无总线从机握手），所以不做重试：
  *         失败基本是时钟/引脚配置问题，重试也不会变好；打印一次后原地待机，
  *         其它任务（协议/打印）照常跑。
  * @param  argument: Not used
  */
void StartChassisTask(void *argument)
{
  uint32_t next_tick;

  if (CHS_Init(&g_chassis) != CHS_OK)
  {
    osMutexAcquire(print_lock, osWaitForever);
    printf("CHS: init FAIL (check TIM4 PD12~PD15 / TIM1,2,3,8 encoder pins)\r\n");
    osMutexRelease(print_lock);

    for (;;)
    {
      osDelay(1000);
    }
  }

  next_tick = osKernelGetTickCount();   /* 以初始化完成为节拍基准 */

  osMutexAcquire(print_lock, osWaitForever);
  printf("CHS: init OK (2xTB6612, TIM4 20kHz, 4xENC, JGB37-520 1:90)\r\n");
  osMutexRelease(print_lock);

#if CHS_SELFTEST_EN
  (void)CHS_SelfTest(&g_chassis);         /* ⚠️ 轮子必须悬空 */
  next_tick = osKernelGetTickCount();     /* 自检阻塞约 4.8s，节拍基准重新对齐 */
#endif

  for (;;)
  {
    /* 严格 20ms 节拍：读 4 路编码器 + 测速/PID/保护/下发，
       用绝对 tick 递增，不用 osDelay，避免长时间运行累积漂移 */
    CHS_Tick(&g_chassis);

    next_tick += (CHS_PERIOD_MS / portTICK_PERIOD_MS);

    /* 如果这一拍被高优先级任务拖过了整整一个周期，不要把欠下的拍子一次性补跑：
       补跑时编码器增量很小，但 CHS_Tick 仍按固定 20ms 解释 ⇒ 速度被算小、PID 抖。
       直接把节拍基准拉到「当前 + 1 拍」，宁可丢一拍，也不制造假速度。 */
    if ((int32_t)(osKernelGetTickCount() - next_tick) >
        (int32_t)(CHS_PERIOD_MS / portTICK_PERIOD_MS))
    {
      next_tick = osKernelGetTickCount() + (CHS_PERIOD_MS / portTICK_PERIOD_MS);
    }

    (void)osDelayUntil(next_tick);
  }
}

/**
  * @brief  机械臂任务：10ms 一拍。
  *         ① 先执行 $ARM,POS/J/HOME 投递下来的作业（阻塞到动作下发完或 J5 补偿走完）；
  *         ② 再做一拍 SB_Tick10ms 轮询（位置 1s、电压 5s、温度状态机）。
  *         ARM_Init() 已在调度器启动前由 main 调过，这里不再重复初始化。
  * @param  argument: Not used
  */
void StartArmTask(void *argument)
{
  for (;;)
  {
    ARM_TaskTick();
    osDelay(10);
  }
}

/* USER CODE END Application */
