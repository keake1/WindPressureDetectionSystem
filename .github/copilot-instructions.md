# Copilot instructions for this repository

## Build commands

This repository contains two independent STM32 CMake projects. Run commands from the board directory, not the repository root.

```powershell
Set-Location ControlBoard
cmake --preset Debug
cmake --build --preset Debug

Set-Location ..\HostBoard
cmake --preset Debug
cmake --build --preset Debug
```

Release builds use the same pattern with the `Release` preset. The presets use Ninja and `cmake/gcc-arm-none-eabi.cmake`; `arm-none-eabi-gcc`, `arm-none-eabi-g++`, `arm-none-eabi-objcopy`, and `arm-none-eabi-size` must be on `PATH`.

No test, single-test, or lint targets are configured in the repository.

## Architecture

The codebase is split into `ControlBoard` and `HostBoard`, each with its own `CMakeLists.txt`, `CMakePresets.json`, linker script, STM32CubeMX `.ioc`, startup assembly, HAL/CMSIS drivers, FreeRTOS copy, and generated `cmake/stm32cubemx/CMakeLists.txt`.

`ControlBoard` targets STM32F070xB/Cortex-M0. It runs an upstream Modbus RTU slave on `huart2`, a downstream sensor Modbus host on `huart1`, a send-only DWIN screen interface on `huart3`, and an IWDG watchdog gated by task heartbeats. Its application starts in `Core/Src/main.c`: initialize HAL/GPIO/USART/IWDG, initialize global status and Modbus/DWIN modules, create `ModbusSlaveTask`, `ModbusSensorTask`, `ControllerMaintenanceTask`, and `DwinSendTask`, then start the FreeRTOS scheduler. There is no `App_Start()` wrapper.

`ControlBoard/Core/Src/global_status.c` owns the shared Modbus/status backing arrays: `g_modbusCoils`, `g_modbusInputRegs`, `g_sensorInfos`, and `g_controllerInfo`. The input register array reserves a diagnostic segment at `GLOBAL_STATUS_DIAG_REG_BASE` (8 registers: error count, zero/duplicate address flags, startup-scan flag, controller address, DWIN drop count) refreshed by `GlobalStatus_RefreshDiagnostics()`. Application code accesses sensor fields through the `SENSOR_ONLINE/SENSOR_ALARM/SENSOR_MODEL/SENSOR_DATA` macros rather than dereferencing `SensorInfo_t` pointers directly.

`Core/Src/modbus_slave.c` handles upstream Modbus function parsing and uses the global backing arrays for coils and input registers. Invalid function code, addressing, or data-value requests return a Modbus exception frame (`function | 0x80` + exception code 01/02/03); CRC failures and address mismatches still return silently. `Core/Src/modbus_sensor_host.c` polls 64 downstream sensors and parses their custom RTU-shell responses. `Core/Src/controller_maintenance.c` handles controller alarm state, duplicate/zero-address status, DWIN icon updates, DWIN numeric value updates, and triggers diagnostic refresh each cycle. `Core/Src/dwin.c` owns the DWIN UART3 send pipeline. `Core/Src/freertos_hooks.c` contains FreeRTOS hook implementations.

DWIN sends are queue-based: `DWIN_WriteVar()` and the higher-level helpers (`DwinIcon_WriteValue`, `DwinValue_WriteU16`, `DwinValue_WriteFloat`, `DwinValue_UpdateSensor`) only enqueue a `DwinFrame_t` (12 B per slot, 16-slot queue). The dedicated `DwinSendTask` is the sole UART3 transmitter; it serializes the queue, frames each message into `5A A5 LEN 82 AH AL <data>`, and uses `HAL_UART_Transmit_IT` with a 100 ms TX-completion timeout (no portMAX_DELAY waits). Queue-full producers drop the frame and increment `g_dwinQueueDropCount` (exposed via `DwinSend_GetDropCount` and the diagnostic input registers). `DwinValue_UpdateSensor` only writes the fields relevant to the sensor's reported model (wind/CO/residual-pressure/seven-in-one), keeping UART3 traffic minimal.

`ModbusSensorTask` first performs `MODBUS_SENSOR_STARTUP_SCAN_ROUNDS` complete scans of addresses 1..64. After startup it repeats a fixed interleaved rhythm: scan `MODBUS_SENSOR_FULL_SCAN_BATCH_COUNT` addresses in `MODBUS_SENSOR_POLL_ALL` mode, then scan one online sensor in `MODBUS_SENSOR_POLL_ONLINE` mode, then delay for `MODBUS_SENSOR_POLL_CYCLE_INTERVAL_MS`. Do not reintroduce a timed full-scan trigger such as `MODBUS_SENSOR_FULL_SCAN_INTERVAL_MS`.

The IWDG watchdog (~1 s timeout, configured in CubeMX) is fed only inside `ControllerMaintenanceTask`. Both `ModbusSlaveTask` and `ModbusSensorTask` increment per-task heartbeat counters (`g_slaveTaskHeartbeat`, `g_sensorTaskHeartbeat`); the maintenance task only calls `HAL_IWDG_Refresh()` when both heartbeats have advanced within `IWDG_TASK_STALE_TIMEOUT_MS` (800 ms). The sensor task is exempt from this check while `g_controllerInfo.startupScanDone == 0` because the startup full-scan blocks the task for several seconds. Any task hang, scheduler deadlock, or trap into `Error_Handler` (`__disable_irq() + while(1)`) lets the IWDG reset the MCU.

`HostBoard` targets STM32F407xx/Cortex-M4F and runs a Modbus RTU host on `huart1`. Its initialization and `ModbusHostTask` creation happen directly in `Core/Src/main.c`; there is no `App_Start()` wrapper. `Core/Src/modbus_host.c` builds Modbus request frames, validates read responses, and treats write requests as no-response-expected operations.

Both boards have a local `Core/Src/modbus_rtu.c` / `Core/Inc/modbus_rtu.h` pair. The RTU layer receives UART bytes via `HAL_UART_RxCpltCallback`, uses a 5 ms inter-frame gap to decide when a frame is ready, copies received bytes under `taskENTER_CRITICAL()`, and sends frames with `HAL_UART_Transmit_IT()` guarded by a mutex and TX-completion semaphore. ControlBoard's `modbus_rtu.c` additionally exposes shared utilities (`ModbusRtu_Crc16` via 256-entry table, `ModbusRtu_ReadU16BE`, `ModbusRtu_WriteU16BE`) consumed by `modbus_slave.c` and `modbus_sensor_host.c` to avoid duplicate CRC/byte-order code.

Generated STM32CubeMX sources are added through `cmake/stm32cubemx/CMakeLists.txt`; custom Modbus sources and FreeRTOS kernel sources are added in each board's top-level `CMakeLists.txt`.

## Repository-specific conventions

- Preserve STM32CubeMX `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` blocks in generated files. Put application edits inside these blocks or in standalone files such as `modbus_*.c`.
- When adding custom source files, add them to the relevant board's top-level `target_sources()` list; the generated `cmake/stm32cubemx/CMakeLists.txt` only covers CubeMX-generated application and HAL driver sources.
- In `ControlBoard/Core/Src/main.c`, keep the user-code task section focused on FreeRTOS task definitions and the IWDG heartbeat watchdog logic. Put alarm logic, DWIN update helpers, sensor parsing, and other application helpers in standalone modules.
- Keep board-specific details separate: MCU macro, linker script, Cortex flags, FreeRTOS portable port, and UART instance differ between `ControlBoard` and `HostBoard`.
- Modbus helpers use fixed-width integer types, `U` suffixes for unsigned constants, `uint8_t` boolean-style return values, and `0U`/`HAL_ERROR` for invalid inputs or no frame/response. Use `ModbusRtu_Crc16/ReadU16BE/WriteU16BE` instead of redefining local copies.
- Access sensor backing fields through the `SENSOR_ONLINE/SENSOR_ALARM/SENSOR_MODEL/SENSOR_DATA` macros declared in `global_status.h`. The `SensorInfo_t` pointer layout is an implementation detail of the Modbus mapping.
- The two `modbus_rtu` implementations (per board) are deliberately duplicated. If changing RTU framing or callback behavior, update both boards intentionally; pure utility additions like the CRC table currently live only in ControlBoard's copy.
- Keep DWIN producers non-blocking. Do not call UART3 directly: enqueue with `DWIN_WriteVar` (or higher-level helpers) and let `DwinSendTask` drain. Tune queue length (`DWIN_QUEUE_LENGTH`) and `DWIN_FRAME_MAX_DATA_LEN` together if frame size or update rate changes.
- Alarm thresholds (CO2/CH2O/TVOC/PM2.5/PM10/CO ×100) are macros at the top of `controller_maintenance.c`; modify them there rather than in the calculation function.
- IWDG configuration lives in `ControlBoard.ioc` (CubeMX). Adjust `IWDG_PRESCALER_*` and reload value via CubeMX, not by hand-editing `iwdg.c`. After changing IWDG timing, re-evaluate `IWDG_TASK_STALE_TIMEOUT_MS` so the heartbeat staleness window stays well below the IWDG period.
- Keep global/file-scope variables documented with concise comments, including `static` globals and `extern` declarations for UART handles or shared status objects.
- Treat `Drivers/`, startup files, linker scripts, `.ioc`, and generated CubeMX sections as vendor/generated board-support code unless the task explicitly requires changes there.
- Text files are normalized to LF via `.gitattributes`; keep new `.c`, `.h`, `.cmake`, `.json`, `.md`, `.ioc`, `.s`, and `.ld` files LF-only.
