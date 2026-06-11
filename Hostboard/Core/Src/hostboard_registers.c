/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hostboard_registers.c
  * @brief          : 128 路 Controlboard 线圈数据存储 + 检测逻辑
  *
  * 存储由接收任务解析的 FC 0x02 响应数据，提供查询 API。
  * 维护零地址检测（三轮 OR）和地址重复检测（三轮错误计数累加 > 2）。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "hostboard_registers.h"
#include <string.h>

/* USER CODE BEGIN 0 */

/* ==================== 零地址检测 ==================== */

/**
 * @brief  零地址检测状态
 *
 * 每轮完整扫描 (0-128) 中: zero_addr_found_this_cycle 记录是否收到 addr0 有效响应
 * StepCycle() 时移入 zero_addr_history[3] 环形缓冲
 * zero_addr_present = history[0] || history[1] || history[2]
 */
static uint8_t  zero_addr_found_this_cycle;     /* 本轮发现标志 */
static uint8_t  zero_addr_history[3];            /* 三轮历史 */
static uint8_t  zero_addr_present;               /* 综合结果 */

/* ==================== 重复地址检测 ==================== */

/**
 * @brief  地址重复检测状态
 *
 * 每轮扫描中: crc_error_count 累计非超时错误
 * StepCycle() 时移入 crc_error_history[3] 环形缓冲
 * 三轮累加 > 2 → addr_conflict_flag = 1
 */
static uint32_t crc_error_count;                 /* 本轮错误计数 */
static uint32_t crc_error_history[3];            /* 三轮历史 */
static uint8_t  addr_conflict_flag;              /* 综合结果 */

/* ==================== 轮次跟踪 ==================== */

static uint8_t  current_cycle;                   /* 0, 1, 2 循环 */

/* ==================== 线圈数据存储 ==================== */

/**
 * board_coil_data[0] 预留（地址 0 有效响应不计入存储）
 * board_coil_data[1..128] 对应 Controlboard 地址 1-128
 */
static uint8_t  board_coil_data[MAX_CTRLBD_ADDR + 1][COIL_BYTE_COUNT];

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 1 */

/* ==================== 数据存储/读取 API ==================== */

/**
 * @brief  存储 Controlboard 的线圈数据
 * @param  addr  Controlboard 地址 (1-128)
 * @param  data  线圈数据指针
 * @param  len   数据长度（上限 COIL_BYTE_COUNT）
 */
void HostReg_StoreCoilData(uint8_t addr, const uint8_t *data, uint8_t len)
{
    if (addr < 1 || addr > MAX_CTRLBD_ADDR)
        return;
    if (len > COIL_BYTE_COUNT)
        len = COIL_BYTE_COUNT;
    memcpy(board_coil_data[addr], data, len);
}

/**
 * @brief  读取 Controlboard 线圈数据的一个字节
 * @param  addr       Controlboard 地址 (1-128)
 * @param  byte_idx   字节索引 (0-16)
 * @return 线圈数据字节，越界返回 0
 */
uint8_t HostReg_GetCoilByte(uint8_t addr, uint8_t byte_idx)
{
    if (addr < 1 || addr > MAX_CTRLBD_ADDR)
        return 0;
    if (byte_idx >= COIL_BYTE_COUNT)
        return 0;
    return board_coil_data[addr][byte_idx];
}

/**
 * @brief  读取 Controlboard 线圈数据的一个位
 * @param  addr       Controlboard 地址 (1-128)
 * @param  bit_idx    位索引 (0-129)
 * @return 线圈位值 (0/1)，越界返回 0
 */
uint8_t HostReg_GetCoilBit(uint8_t addr, uint16_t bit_idx)
{
    if (addr < 1 || addr > MAX_CTRLBD_ADDR)
        return 0;
    if (bit_idx >= COIL_BYTE_COUNT * 8)
        return 0;
    return (board_coil_data[addr][bit_idx / 8] >> (bit_idx % 8)) & 1;
}

/* ==================== 零地址检测 API ==================== */

/**
 * @brief  记录收到地址 0 的 CRC 有效响应
 */
void HostReg_RecordZeroAddrResponse(void)
{
    zero_addr_found_this_cycle = 1;
}

/**
 * @brief  查询零地址是否存在
 * @retval 1 存在 Controlboard 误设为地址 0
 */
uint8_t HostReg_GetZeroAddrPresent(void)
{
    return zero_addr_present;
}

/* ==================== 重复地址检测 API ==================== */

/**
 * @brief  记录一次非超时错误（CRC 失败/帧异常）
 */
void HostReg_RecordError(void)
{
    crc_error_count++;
}

/**
 * @brief  查询是否存在地址重复
 * @retval 1 存在地址重复
 */
uint8_t HostReg_GetAddrConflict(void)
{
    return addr_conflict_flag;
}

/* ==================== 周期更新 ==================== */

/**
 * @brief  每轮完整扫描结束时调用
 *
 * 1. 将本轮 CRC 错误数存入历史环形缓冲，清零计数
 * 2. 三轮累加 > 2 → addr_conflict_flag = 1
 * 3. 将本轮零地址发现标志移入历史，清零
 * 4. zero_addr_present = 三轮 OR
 */
void HostReg_StepCycle(void)
{
    /* 重复地址检测：三轮错误计数累加 > 2 */
    crc_error_history[current_cycle] = crc_error_count;
    crc_error_count = 0;

    uint32_t sum = crc_error_history[0] + crc_error_history[1] + crc_error_history[2];
    addr_conflict_flag = (sum > 2) ? 1 : 0;

    /* 零地址检测：三轮 OR */
    zero_addr_history[current_cycle] = zero_addr_found_this_cycle;
    zero_addr_found_this_cycle = 0;
    zero_addr_present = zero_addr_history[0]
                     || zero_addr_history[1]
                     || zero_addr_history[2];

    current_cycle = (current_cycle + 1) % 3;
}

/* USER CODE END 1 */
