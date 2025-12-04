# **GETTING STARTED**
## Compilation
1. You need to install arm-none-eabi-gcc of any version, you can do this through the package manager on most distributions
2. `cd Release`
3. `make "VMM3Atrig.elf" -j4`
4. To clean after compilation you can use the `make clean` command
## Flash
1. You need to install st-info and st-flash utilities, you can do this through the package manager on most distributions
2. To flash the firmware into stm you need a binary file with the extension `.bin`, you can get it with the command `arm-none-eabi-objcopy -O binary VMM3Atrig.elf VMM3Atrig.bin`
3. Run the command `st-info --probe` to check the correct connection and proper operation of the st-link v2
4. Run `st-flash write VMM3Atrig.bin 0x08000000` to flash firmware
5. Run `st-info --reset` to reset board after flashing
# **CODE NAVIGATION**
## Configuration
You can find configuration settings in `VMM3Atrig/Core/Inc/vmm3a.h`
## Time filtering algorithms
You can find it in `VMM3Atrig/Core/Src/main.c` in infinite while loop
## FIFO (buffer) algorithms
You can find it in `VMM3Atrig/Core/Src/vmm3a.c`
## Trigger interrupt functions
You can find it in `VMM3Atrig/Core/Src/tim.c` and interrupt handler in `VMM3Atrig/Core/Src/stm32g4xx_it.c`
