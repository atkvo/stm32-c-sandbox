# STM32 SANDBOX

This project is just intended to become more familiar with the
bare metal STM32 environment including:

* Build system (Cmake)
* HAL interface development
* Device drivers
* Unit testing embedded firmware.
* Debugging via OpenOCD/GDB/ST-Link

There is no specific project/application in mind at the moment. Just an ongoing learning exercise.

The initial STM32 development board is the [WeAct Black Pill V2.0](https://stm32-base.org/boards/STM32F411CEU6-WeAct-Black-Pill-V2.0.html)


## Requirements

* Compiler: `arm-none-eabi-gcc`
* Build System: `CMake`
* Debugger:`st-link` or `OpenOCD`

## Build

```sh
# create a build folder for an out of source build
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../toolchains/arm-none-eabi-gcc.cmake
camke --build .
```

## Flashing & Debugging

### Flashing
The build system encapsulates the flashing logic. Ensure your ST-LINK is connected and the board is powered, then run:

```bash
cmake --build <build_dir> --target flash
```

*Note: The project is configured to flash to base address `0x08000000`. By default, this target utilizes `st-flash` for the transfer.*

### Debugging
Debugging is performed via **OpenOCD** acting as a GDB server.

#### 1. Start the GDB Server
In a dedicated terminal, launch OpenOCD with the target configuration:
```bash
openocd -f interface/stlink-dap.cfg -f target/stm32f4x.cfg
```

#### 2. Connect GDB
In a separate shell (or within your Neovim terminal), launch the ARM GDB client and connect to the local server (port `3333`):
```bash
arm-none-eabi-gdb build/your_project.elf
```

Once inside the GDB prompt, use the following commands to begin your session:
```gdb
(gdb) target remote localhost:3333
(gdb) monitor reset halt
(gdb) load
```

## Development Standards
### Commit Convention
This project follows the [Conventional Commits](https://www.conventionalcommits.org/) standard.
Format: `<type>(scope): <description>`

#### Allowed Types
- `feat`: A new feature or capability.
- `fix`: A bug fix.
- `refactor`: Structural changes (no logic/feature change).
- `build`: Build system or dependency updates (CMake/Meson).
- `docs`: Documentation updates.
- `style`: Cosmetic changes (does not effect binary).

### Allowed Scopes
- `core`: The task scheduler and timekeeping logic.
- `hal`: Hardware abstraction interfaces (GPIO, I2C, DMA).
- `drv`: External peripheral drivers (SSD1306).
- `app`: Application-specific tasks and logic.
- `common`: Utility libraries (pool_mgr, slice).

