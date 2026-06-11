/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus_polling.c
  * @brief          : Modbus 轮询任务 — 顺序扫描地址 0-128
  *
  * 任务流：
  *   TaskModbusPoll (每 50ms)
  *     └── 构造 FC 0x02 请求 (slave_addr=current, 130 bits)
  *           └── ModbusMaster_EnqueueRequest() → [xMasterSendQueue]
  *                 └── 发送任务发帧 → Controlboard 回复 → 接收任务解析
  *
  * 地址回绕时 (current_addr > 128)：
  *   └── HostReg_StepCycle() — 更新零地址标志和重复地址标志
  *
  * 参考 Controlboard/Core/Src/modbus_polling.c 架构。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "modbus_polling.h"
#include "uart1_modbus_master.h"
#include "hostboard_registers.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==================== 轮询任务 ==================== */

void TaskModbusPoll(void *arg)
{
    (void)arg;
    uint16_t current_addr = 0;

    for (;;)
    {
        /* ---- 轮询间隔 50ms ---- */
        vTaskDelay(pdMS_TO_TICKS(50));

        /* ---- 构造 FC 0x02 请求：读 130 bits (0-129) ---- */
        ModbusMasterRequest_t req;
        req.slave_addr = (uint8_t)current_addr;
        req.func_code  = MODBUS_FUNC_READ_DISCRETE_INPUTS;
        req.reg_addr   = 0;
        req.reg_value  = 130;
        ModbusMaster_EnqueueRequest(&req);

        /* ---- 地址推进 ---- */
        current_addr++;
        if (current_addr > MAX_CTRLBD_ADDR)
        {
            current_addr = 0;
            HostReg_StepCycle();
        }
    }
}

/* USER CODE END 0 */
