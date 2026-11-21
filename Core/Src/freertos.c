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
#include "safety.h"
#include "chassis.h"
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

/* I2C1 总线互斥（i2c_lock）：
 * I2C1(PB6/PB7) 上挂了两个从机 —— 机械臂 PCA9685(0x40) 与底盘驱动板(0x51)。
 * HAL 的 I2C 句柄不可重入：ServoTask(High) 若在 ChassisTask 传输途中抢占并去写
 * PCA9685，会拿到 HAL_BUSY 而丢一次舵机刷新；两者周期又都是 20ms，可能长期同相
 * 碰撞 → 表现为「车一动，机械臂就像卡住」。故所有 I2C1 访问统一套这把锁。
 * 锁序固定为 arm_lock → i2c_lock（ChassisTask 只拿 i2c_lock，不反向嵌套），无死锁。 */
osMutexId_t i2c_lock;
const osMutexAttr_t i2c_mutex_attributes = {
  .name = "i2cLock"
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

/* 安全监控任务（阶段 1 新增）：独立低优先级任务，每 200ms 调 Safety_Tick。
 * 为什么要独立、且优先级最低：Safety_Tick 里的 ACS712_Read_mA 会阻塞约 20ms
 * （跨 PWM 周期采样），塞进 ServoTask 会把舵机刷新拖慢一倍。单独放最低优先级，
 * 20ms 阻塞对其它任务零影响；过流检测延迟最多 200ms，对堵转保护绰绰有余。 */
osThreadId_t SafetyTaskHandle;
const osThreadAttr_t SafetyTask_attributes = {
  .name = "SafetyTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* 底盘任务（四路编码器 TT + 麦轮）：20ms 一次，与驱动板 20ms 速度环同节拍。
 * 优先级 Normal：低于 ServoTask(High)，保证舵机刷新优先；
 * 与 ProtocolTask 同级，靠时间片轮转，互不饿死。 */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartServoTask(void *argument);
void StartProtocolTask(void *argument);
void StartPrintTask(void *argument);
void StartSafetyTask(void *argument);
void StartChassisTask(void *argument);
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
  i2c_lock   = osMutexNew(&i2c_mutex_attributes);
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
  SafetyTaskHandle  = osThreadNew(StartSafetyTask,  NULL, &SafetyTask_attributes);
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);
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
    osMutexAcquire(i2c_lock, osWaitForever);   /* PCA9685 与底盘驱动板共用 I2C1，防 HAL_BUSY 丢刷新 */
    Arm_Update();
    osMutexRelease(i2c_lock);
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

/**
  * @brief  安全监控任务（阶段 1 新增）：每 200ms 调一次 Safety_Tick。
  *         检测 ACS712 过流，持续超阈值则触发 Arm_ReportFault 进 FAULT 态。
  *         优先级最低——Safety_Tick 内采样会阻塞约 20ms，不影响舵机/协议任务。
  * @param  argument: Not used
  */
void StartSafetyTask(void *argument)
{
  for(;;)
  {
    Safety_Tick();
    osDelay(200);
  }
}

/**
  * @brief  底盘任务：初始化四路编码器驱动板(0x51)，然后 20ms 固定周期跑 CHS_Tick。
  *         初始化失败（多半是驱动板 VM 没上电 / SCL SDA GND 线序 / 缺上拉）时
  *         不阻塞其它任务，每秒静默重试一次，接好线自动恢复。
  * @param  argument: Not used
  */
void StartChassisTask(void *argument)
{
  uint8_t inited = 0U;
  uint8_t warned = 0U;
  uint32_t next_tick = 0U;

  for (;;)
  {
    if (inited == 0U)
    {
      if (CHS_Init(&g_chassis) == CHS_OK)
      {
        inited = 1U;
        next_tick = osKernelGetTickCount();   /* 以初始化完成为节拍基准 */
        osMutexAcquire(print_lock, osWaitForever);
        printf("CHS: init OK (0x51), TT/1:90, wheel 8cm 13line\r\n");
        osMutexRelease(print_lock);

#if CHS_SELFTEST_EN
        (void)CHS_SelfTest(&g_chassis);   /* ⚠️ 轮子必须悬空 */
        next_tick = osKernelGetTickCount();  /* 自检阻塞约 8s，节拍基准重新对齐 */
#endif
      }
      else
      {
        if (warned == 0U)
        {
          warned = 1U;
          osMutexAcquire(print_lock, osWaitForever);
          printf("CHS: 0x51 no ACK (check VM power / SCL SDA GND / pull-up)\r\n");
          osMutexRelease(print_lock);
        }
        osDelay(1000);
        continue;
      }
    }

    /* 严格 20ms 节拍：每拍读 4 路编码器(≈4ms) + 下发速度，用绝对 tick 不累积漂移 */
    CHS_Tick(&g_chassis);

    next_tick += (CHS_PERIOD_MS / portTICK_PERIOD_MS);
    (void)osDelayUntil(next_tick);
  }
}

/* USER CODE END Application */
