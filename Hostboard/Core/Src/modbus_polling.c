/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus_polling.c
  * @brief          : Modbus 轮询任务实现
  *
  * 任务流：
  *   TaskModbusPoll
  *     └── ModbusMaster_EnqueueRequest() → [xMasterSendQueue]
  *           └── 发送任务发帧 → Controlboard 回复 → 接收任务解析
  *
  * 每 1000ms 执行一次：
  *   1. 读离散输入 (FC 0x02, addr 0, 32 bits) → 获取在线/报警状态
  *   2. 读保持寄存器 (FC 0x03, addr 1, 10 regs) → 获取传感器类型和数据
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "modbus_polling.h"
#include "uart1_modbus_master.h"

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
    ModbusMasterRequest_t req;

    /* 请求模板 */
    req.slave_addr = (uint8_t)CONTROLBOARD_ADDR;

    for (;;)
    {
        /* ---- 轮询间隔 ---- */
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));

        /* ---- 读离散输入：获取传感器在线状态（位 0-31） ---- */
        req.func_code  = MODBUS_FUNC_READ_DISCRETE_INPUTS;
        req.reg_addr   = 0;
        req.reg_value  = 32;
        ModbusMaster_EnqueueRequest(&req);

        /* ---- 读保持寄存器：获取传感器 1-5 的类型和数据 ---- */
        req.func_code  = MODBUS_FUNC_READ_HOLDING_REGISTERS;
        req.reg_addr   = 1;      /* 从地址 1（传感器 1 类型）开始 */
        req.reg_value  = 5;      /* 读 5 个：传感器 1-5 的类型 */
        ModbusMaster_EnqueueRequest(&req);
    }
}

/* USER CODE END 0 */
