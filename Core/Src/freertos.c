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
#include "pca9685.h"
#include "potentiometer.h"
#include "arm_control.h"
#include "uart_protocol.h"
#include "bt.h"
#include "wifi.h"
#include "teach.h"
#include "rfid.h"
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
/* 共享给任务组件的状态变量（原裸机 main.c 主循环静态量） */
static Pot_Data_t pot;

/* KEY0(PE4) 按键：低有效，软件消抖状态 */
static uint8_t key0_prev   = 1;   /* 上一次消抖电平(弹起=1) */
static uint8_t key0_count  = 0;   /* 连续采样计数 */
static uint8_t key0_active = 0;   /* 本次按下是否已处理 */

/* huart1 访问互斥（print_lock）：
 * - ServoTask 的 KEY0/RFID 事件打印、PrintTask 的周期状态打印（printf 走 huart1）；
 * - ProtocolTask 发回 PC 的协议响应帧（目标为 USART1 时）。
 * 三者互斥，保证 PC 串口上响应帧与打印消息不会按字符交错。 */
osMutexId_t print_lock;
const osMutexAttr_t print_mutex_attributes = {
  .name = "printLock"
};

/* 机械臂状态互斥（arm_lock）：arm_state、target_pulse、current_pulse、teach 等
 * 状态被 ServoTask（运动/Teach_Tick）与 ProtocolTask（命令处理）并发访问，
 * 所有 Arm 与 Teach 接口调用点都在锁内执行。锁独立获取、不与 print_lock
 * 嵌套，无死锁。 */
osMutexId_t arm_lock;
const osMutexAttr_t arm_mutex_attributes = {
  .name = "armLock"
};

/* ---- 机械臂三个任务的句柄与属性（全在 USER CODE 内，重新生成被保留） ---- */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

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
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartServoTask(void *argument);
void StartProtocolTask(void *argument);
void StartPrintTask(void *argument);
/* USER CODE END FunctionPrototypes */

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
  arm_lock   = osMutexNew(&arm_mutex_attributes);
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
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  ServoTaskHandle   = osThreadNew(StartServoTask,   NULL, &ServoTask_attributes);
  ProtocolTaskHandle = osThreadNew(StartProtocolTask, NULL, &ProtocolTask_attributes);
  PrintTaskHandle   = osThreadNew(StartPrintTask,   NULL, &PrintTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  机械臂运动控制任务：20ms 固定周期。
  *         内容为原裸机主循环中的 20ms 块。优先级最高，保证即使协议任务在
  *         阻塞发送串口响应时也不会拖延舵机更新。
  * @param  argument: Not used
  */
void StartServoTask(void *argument)
{
  for(;;)
  {
    /* ===== KEY0 按键：切换 手动/回HOME（20ms 周期软件消抖） ===== */
    {
      uint8_t key0 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);

      /* 连续 2 次(40ms)采样一致才认定电平稳定 */
      if (key0 == key0_prev)
      {
        if (key0_count < 2) key0_count++;
      }
      else
      {
        key0_count = 0;
        key0_prev = key0;
      }

      /* 下降沿(按下)触发一次切换；弹起后才允许下次触发 */
      if (key0 == 0 && key0_count == 2 && key0_active == 0)
      {
        key0_active = 1;
        osMutexAcquire(arm_lock, osWaitForever);
        if (Arm_GetState() == ARM_STATE_MANUAL)
        {
          Arm_CommandHome();
          osMutexRelease(arm_lock);
          osMutexAcquire(print_lock, osWaitForever);
          printf("KEY0: -> HOME\r\n");
          osMutexRelease(print_lock);
        }
        else if (Arm_GetState() == ARM_STATE_CMD)
        {
          Arm_CommandManual();
          osMutexRelease(arm_lock);
          osMutexAcquire(print_lock, osWaitForever);
          printf("KEY0: -> MANUAL\r\n");
          osMutexRelease(print_lock);
        }
        else
        {
          osMutexRelease(arm_lock);
        }
        /* 急停/示教态不响应，保持安全 */
      }
      else if (key0 == 1 && key0_count == 2)
      {
        key0_active = 0;   /* 弹起，允许下次触发 */
      }
    }

    /* 手动/示教记录态下，电位器作为目标输入 */
    osMutexAcquire(arm_lock, osWaitForever);
    if (Arm_GetState() == ARM_STATE_MANUAL || Arm_GetState() == ARM_STATE_RECORD)
    {
      Pot_ReadAll(&pot);
      Arm_SetManualTargets(&pot);
    }
    osMutexRelease(arm_lock);

    /* 示教记录采样 / 回放取点（内部按间隔推进，非记录/回放态无操作） */
    osMutexAcquire(arm_lock, osWaitForever);
    Teach_Tick();
    osMutexRelease(arm_lock);

    /* ===== RC522 刷卡切换模式：检测到卡且上次卡已移开 -> 触发一次 =====
     * 无卡时一次读卡约 20ms，若每拍都查会把 ServoTask 周期拖到 ~40ms、
     * 舵机刷新减半；故分频：每 5 拍(100ms)查一次，刷卡响应仍足够快。 */
    {
      static uint8_t rfid_scan_div = 0U;
      if (rfid_scan_div == 0U)
      {
        RFID_Card_t card;
        static uint8_t rfid_present = 0U;

        if (RFID_CheckCard(&card) == HAL_OK)
        {
          if (rfid_present == 0U)
          {
            rfid_present = 1U;

            static const uint8_t manual_uid[RFID_UID_LEN] = RFID_UID_CARD_MANUAL;
            static const uint8_t home_uid[RFID_UID_LEN]   = RFID_UID_CARD_HOME;

            if (memcmp(card.uid, manual_uid, RFID_UID_LEN) == 0)
            {
              osMutexAcquire(arm_lock, osWaitForever);
              if (Arm_GetState() == ARM_STATE_CMD)
              {
                Arm_CommandManual();
                osMutexRelease(arm_lock);
                osMutexAcquire(print_lock, osWaitForever);
                printf("RFID: -> MANUAL\r\n");
                osMutexRelease(print_lock);
              }
              else
              {
                osMutexRelease(arm_lock);
              }
            }
            else if (memcmp(card.uid, home_uid, RFID_UID_LEN) == 0)
            {
              osMutexAcquire(arm_lock, osWaitForever);
              if (Arm_GetState() == ARM_STATE_MANUAL)
              {
                Arm_CommandHome();
                osMutexRelease(arm_lock);
                osMutexAcquire(print_lock, osWaitForever);
                printf("RFID: -> HOME\r\n");
                osMutexRelease(print_lock);
              }
              else
              {
                osMutexRelease(arm_lock);
              }
            }
          }
        }
        else
        {
          rfid_present = 0U;   /* 卡已移开，允许下次刷卡触发 */
        }
      }
      rfid_scan_div++;
      if (rfid_scan_div >= 5U) rfid_scan_div = 0U;
    }

    /* 平滑过渡并输出（含急停冻结逻辑） */
    osMutexAcquire(arm_lock, osWaitForever);
    Arm_Update();
    osMutexRelease(arm_lock);

    osDelay(20);
  }
}

/**
  * @brief  串口协议后台任务：解析命令帧、发送待发响应、重启串口接收。必须高频运行：
  *         中断只收字节入队，命令解析/执行与发送（含 USART3 WiFi 的 CIPSEND 封装）
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
  * @brief  周期遥测打印任务：每 3000ms 输出一次状态（可用 $LOG,1/$LOG,0 开关）。
  *         先持 arm_lock 读一致的状态/脉宽快照，再持 print_lock 打印，
  *         两把锁先后独立获取（不同时持有），避免与 ServoTask/ProtocolTask 交错。
  * @param  argument: Not used
  */
void StartPrintTask(void *argument)
{
  for(;;)
  {
    osDelay(3000);
    if (UartProto_GetStreamLog())
    {
      uint32_t st;
      uint32_t p0, p1, p2, p3, p4, p5;

      osMutexAcquire(arm_lock, osWaitForever);
      st = (uint32_t)Arm_GetState();
      p0 = (uint32_t)Arm_GetCurrent(0);
      p1 = (uint32_t)Arm_GetCurrent(1);
      p2 = (uint32_t)Arm_GetCurrent(2);
      p3 = (uint32_t)Arm_GetCurrent(3);
      p4 = (uint32_t)Arm_GetCurrent(4);
      p5 = (uint32_t)Arm_GetCurrent(5);
      osMutexRelease(arm_lock);

      osMutexAcquire(print_lock, osWaitForever);
      printf("STATE %u | PULSE %u %u %u %u %u %u\r\n", st, p0, p1, p2, p3, p4, p5);
      osMutexRelease(print_lock);
    }
  }
}

/* USER CODE END Application */
