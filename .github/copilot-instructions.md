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

`ControlBoard` targets STM32F070xB/Cortex-M0 and runs a Modbus RTU slave on `huart2`. Its application starts in `Core/Src/main.c`: initialize HAL/GPIO/USART, call `App_Start()`, initialize slave data, initialize Modbus RTU, create `ModbusSlaveTask`, and start the FreeRTOS scheduler. `Core/Src/modbus_slave.c` owns the coil/discrete/input/holding-register backing arrays and implements Modbus function handling.

`HostBoard` targets STM32F407xx/Cortex-M4F and runs a Modbus RTU host on `huart1`. Its `App_Start()` initializes Modbus RTU, creates `ModbusHostTask`, and starts FreeRTOS. `Core/Src/modbus_host.c` builds Modbus request frames, validates read responses, and treats write requests as no-response-expected operations.

Both boards have a local `Core/Src/modbus_rtu.c` / `Core/Inc/modbus_rtu.h` pair. The RTU layer receives UART bytes via `HAL_UART_RxCpltCallback`, uses a 5 ms inter-frame gap to decide when a frame is ready, copies received bytes under `taskENTER_CRITICAL()`, and sends frames with blocking `HAL_UART_Transmit()`.

Generated STM32CubeMX sources are added through `cmake/stm32cubemx/CMakeLists.txt`; custom Modbus sources and FreeRTOS kernel sources are added in each board's top-level `CMakeLists.txt`.

## Repository-specific conventions

- Preserve STM32CubeMX `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` blocks in generated files. Put application edits inside these blocks or in standalone files such as `modbus_*.c`.
- When adding custom source files, add them to the relevant board's top-level `target_sources()` list; the generated `cmake/stm32cubemx/CMakeLists.txt` only covers CubeMX-generated application and HAL driver sources.
- Keep board-specific details separate: MCU macro, linker script, Cortex flags, FreeRTOS portable port, and UART instance differ between `ControlBoard` and `HostBoard`.
- Modbus helpers use fixed-width integer types, `U` suffixes for unsigned constants, `uint8_t` boolean-style return values, and `0U`/`HAL_ERROR` for invalid inputs or no frame/response.
- The two `modbus_rtu` implementations are currently duplicated. If changing RTU behavior, update both boards intentionally or extract shared code as a deliberate refactor.
- Treat `Drivers/`, startup files, linker scripts, `.ioc`, and generated CubeMX sections as vendor/generated board-support code unless the task explicitly requires changes there.
- Text files are normalized to LF via `.gitattributes`; keep new `.c`, `.h`, `.cmake`, `.json`, `.md`, `.ioc`, `.s`, and `.ld` files LF-only.
