# Platforms and platform support files

This directory contains the implementation of platforms and support file
used by (multiple) platforms.

## Implementation directories

* bluepill:  Firmware Bluepill
* blackpill-f401cc: Firmware for the WeAct Studio [Black Pill F401CC](https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1)
* blackpill-f401ce: Firmware for the WeAct Studio [Black Pill F401CE](https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1)
* blackpill-f411ce: Firmware for the WeAct Studio [Black Pill F411CE](https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1)

## Support directories

* common: common platform support for all platforms except hosted (BMDA)
* common/blackpill-f4: blackpill-f4 specific common platform code
* common/stm32: STM32 specific libopencm3 common platform support
