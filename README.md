### nrf-lora-mesh

#### Usage

1. Get nRF5 SDK 15.3.0 from: http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/nRF5_SDK_15.3.0_59ac345.zip
2. Get gcc-arm-none-eabi toolchain from: https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/8-2019q3/RC1.1/gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2
3. Extract toolchain, setup PATH in `~/.bashrc`
4. Extract nRF5 SDK, modify `config.mk` to specify SDK PATH
5. Run `make` to build firmware
6. Run `make flash` to program via nrfjprog or `make flash_openocd` via openocd
