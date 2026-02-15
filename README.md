<p align="center">
  <img src="https://raw.githubusercontent.com/h3cx/MuBus/main/images/mubus.png"
       width="30%" />
</p>

MuBus is a compact serial messaging library for embedded systems with explicit runtime selection and consistent packet framing.

## Choose your runtime (required)
Define **exactly one** runtime macro before including `MuBus.h`:

- `MUBUS_RUNTIME_ARDUINO`
- `MUBUS_RUNTIME_MBED`
- `MUBUS_RUNTIME_FREERTOS`

Compilation fails if none or multiple runtimes are selected.

## Quickstart by platform
- Arduino guide: [`docs/platforms/arduino.md`](docs/platforms/arduino.md)
- mbed guide: [`docs/platforms/mbed.md`](docs/platforms/mbed.md)
- FreeRTOS guide: [`docs/platforms/freertos.md`](docs/platforms/freertos.md)

## Core docs
- Docs index by user intent: [`docs/README.md`](docs/README.md)
- Runtime selection and troubleshooting: [`docs/runtime-selection.md`](docs/runtime-selection.md)
- API reference: [`docs/api/README.md`](docs/api/README.md)
- Migration from legacy APIs: [`docs/migration.md`](docs/migration.md)

## Install (PlatformIO)
```sh
pio pkg install -l "https://github.com/h3cx/MuBus.git"
```
