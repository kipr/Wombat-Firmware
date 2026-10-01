# AGENTS.md

Instructions for AI coding agents working in this repository. The contribution
workflow for everyone, human or agent, is in [CONTRIBUTING.md](CONTRIBUTING.md).

## What this is

Bare-metal firmware for the STM32F427VIT6 (Cortex-M4F, 180 MHz) co-processor
on the KIPR Wombat robot controller. It runs on every Wombat, so a regression
here reaches every team and classroom that flashes it.

The STM32 is an SPI slave to the Wombat's Raspberry Pi, where
[libwallaby](https://github.com/kipr/libwallaby) runs user programs. The
firmware has no application logic: it drives motors and servos, samples
sensors, the IMU, and the battery, and exchanges everything with the Pi through
a shared register map. The source still uses the board's earlier name,
"wallaby".

## How we work

Follow [CONTRIBUTING.md](CONTRIBUTING.md). In particular:

- Before editing files, present a plan (what will change, which files, how
  you'll verify it) and wait for approval from the person you're working with.
- Keep the change to what was asked, within the existing architecture below.
  Don't refactor, reformat, or rename code you weren't asked to touch, and
  don't add code for hypothetical future needs.
- Comment only what the code can't say for itself: hardware quirks, timing
  budgets, datasheet constraints, the reason behind a non-obvious choice. Don't
  narrate what the code does, and don't leave commented-out code.
- Update README.md, this file, and `docs/` when your change makes them wrong.
- Branch names start with the name of the person who owns the change:
  `<name>/<topic>`.
- Report verification honestly. Say what you built and checked, and state
  plainly what wasn't tested on hardware.

## Build

The canonical build runs in Docker, so no local toolchain is needed:

```bash
docker compose build
docker compose run --rm build-wombat-firmware
```

Use `run`, not `up`: `docker compose up` doesn't return the build's exit code,
so a failed build looks like success. Output lands in `build/Firmware/`:
`wombat.bin` is the flashable image, alongside `.elf`, `.hex`, `.map`, and
`.lss`. CI runs the same commands on every pull request.

With `arm-none-eabi-gcc` and CMake 3.x installed, `./build.sh` builds natively.
CMake 4 rejects this project's `cmake_minimum_required(VERSION 2.8.12)`, which
is why the Docker image is pinned to Ubuntu 24.04.

There's no clean target; delete `build/` instead. Also delete it when switching
between Docker and native builds, because the CMake cache records absolute
paths.

The build already has a few dozen compiler warnings. Don't add new ones in
files you touch.

## Verifying a change

There are no automated tests. Before calling a change done:

1. The Docker build succeeds.
2. If you touched the register map, `scripts/check-register-map.sh` passes (see
   below).
3. If the change affects hardware behavior, it needs testing on a Wombat.
   Agents don't flash hardware: tell the person you're working with what to
   test, and say in the pull request whether it was tested.

To flash, copy `wombat.bin` to `/home/kipr/wombat-os/flashFiles/` on the Wombat
and run `sudo ./wallaby_flash` from that directory. The flashing tools live in
[wombat-os](https://github.com/kipr/wombat-os/tree/main/flashFiles).

## Architecture

### Main loop (`Firmware/src/main.c`)

A single `while (1)` superloop with a fixed time budget for consistent PID
timing. Each pass spends a 700 µs window on sensor updates (digital pins,
battery, IMU) and sleeps for whatever's left. Every 4th pass, the motors idle
in that window so back-EMF can be sampled, then PID and motor updates run; the
other passes sleep 222 µs instead, keeping passes at about 922 µs. Anything
added to the loop has to fit this budget.

### Host communication: the register map

This is the core abstraction, and the place to start reading.

- `aTxBuffer` and `aRxBuffer` (`REG_ALL_COUNT` = 153 bytes) are the entire
  interface to the Pi. Register indices are `#define`d in
  `Firmware/include/wallaby_spi_r4.h`, which `wallaby.h` includes.
- SPI2 runs as a circular DMA slave (`wallaby_dma.c`). After each transfer,
  `handle_dma()` (called from `DMA1_Stream3/4_IRQHandler`) checks the framing
  (first byte `'J'`, last readable byte `'S'`, protocol byte equal to
  `WALLABY_SPI_VERSION`) and applies the host's `(address, value)` writes to
  `aTxBuffer`. It ignores writes to the start and version registers or past
  `REG_ALL_COUNT`, and applies at most 42 writes per packet, the most that fit
  before the end byte. Some writes set `adc_dirty` or `dig_dirty`, which the
  main loop picks up to reconfigure peripherals.
- Multi-byte values are split into `_H`/`_L` (or `_B3`…`_B0`) registers, most
  significant byte first. Drivers read goals and PWM values from `aTxBuffer` and
  write sensor results back into it.
- `WALLABY_FIRMWARE_VERSION_R` (`wallaby.h`) is reported to the host through
  `REG_R_VERSION_H`/`_L`.

**Changing the register map is a breaking protocol change.** libwallaby keeps
its own copy in
[`module/core/protected/kipr/core/registers.hpp`](https://github.com/kipr/libwallaby/blob/master/module/core/protected/kipr/core/registers.hpp).
A change must bump `WALLABY_SPI_VERSION`, land with a matching libwallaby pull
request (link the two), and pass
`scripts/check-register-map.sh path/to/libwallaby` against that pull request's
branch. CI compares against libwallaby `master`, so it fails until the
libwallaby side merges; that's expected for a paired change.

### Board configuration

`wallaby.h` selects the hardware revision through the headers it includes:
`wallaby_r2.h` (pin, port, and ADC mapping) and `wallaby_spi_r4.h` (register
map). Names like `MOT0_DIR1_PORT` and `LED1_PIN` resolve through them. The
`wallaby_r0.h`, `wallaby_r1.h`, and `wallaby_spi_r1.h`–`_r3.h` headers are
unused.

### Drivers (`Firmware/src/wallaby_*.c`)

One file per subsystem, each with a header in `Firmware/include/`: `init`
(clock, GPIO, and peripheral bring-up in `init()`), `dma` (host link), `adc`,
`bemf`, `dig`, `motor`, `pid`, `servo`, `imu`, `i2c`, `uart`, and `spi`.

| Peripheral | Use |
|---|---|
| SPI2, DMA1 streams 3 and 4 | Host link (slave) |
| SPI3 | MPU-9250 IMU (master) |
| TIM1, TIM8 | Motor PWM |
| TIM3, TIM9 | Servo pulses |
| ADC1–3 | Analog ports, motor back-EMF, battery (pin mapping in `wallaby_r2.h`) |
| SysTick | 1 µs tick behind `usCount` and `delay_us()` |

The 180 MHz system clock comes from `init180MHz()` in `wallaby_init.c`, which
reprograms the PLL from the 24 MHz crystal. `system_stm32f4xx.c` is ST's stock
file; its PLL settings assume a 25 MHz crystal and are overridden.

### Vendor library

`libs/STM32F4xx_StdPeriph_Driver/` is ST's Standard Peripheral Library (not HAL
or LL), built as the `stm32f4xx` static library. Don't edit it. Compiler and
linker flags live in `CMake/GNU-ARM-Toolchain.cmake`; the linker script is
`linker/STM32F427VITx_FLASH.ld`.

## Conventions and gotchas

- **StdPeriph style.** Use `GPIO_Init`, `RCC_*PeriphClockCmd`, and direct
  register writes such as `PORT->BSRRL |= PIN` (set) and `PORT->BSRRH |= PIN`
  (reset), matching the surrounding code. Don't introduce HAL.
- **Line endings vary by file.** Most of `Firmware/` is CRLF; `main.c`,
  `system_stm32f4xx.c`, the startup file, and the build files are LF. Keep each
  file's existing endings, and check `git diff --stat` for whole-file churn.
- **Register names are inconsistent.** Some macros use a lowercase `w`
  (`REG_Rw_MOT_1_B2`, `REG_w_MOT_1_GOAL_B2`), and `REG_W_PID_1_D_H` is 119
  while `REG_W_PID_1_D_L` is 110. Grep for the exact name rather than assuming
  the obvious spelling, and don't rename them without a matching libwallaby
  change.
- **`debug_printf` is an empty stub** unless built with
  `USE_CROSS_STUDIO_DEBUG`. Don't rely on its output.
- **The `HSE_VALUE=16000000` comment in `main.c` is stale.** The real value,
  24 MHz, is set in `CMakeLists.txt` and matches the crystal (U38).

## Reference

- `docs/Datasheets/README.md` maps the datasheet files, which are named by
  reference designator (`U37.pdf`), to parts.
- `docs/Schematics/` has the board schematic in Eagle and PDF form;
  `docs/Wombat_BOM_rev4.xlsx` is the bill of materials.
- The [KIPR Development Toolkit](https://github.com/kipr/KIPR-Development-Toolkit)
  has the Wombat Developer Manual and STM32 pin explanations (PDFs in `Docs/`).
- Related repositories: [libwallaby](https://github.com/kipr/libwallaby) (host
  library and the other half of the register map) and
  [wombat-os](https://github.com/kipr/wombat-os) (flashing tools).
