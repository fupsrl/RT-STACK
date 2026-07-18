################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/board_safety_stm32.c \
../Core/Src/diagnostics_can.c \
../Core/Src/engine_control.c \
../Core/Src/engine_watchdog_stm32.c \
../Core/Src/injection.c \
../Core/Src/main.c \
../Core/Src/spark.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/trigger_capture_stm32.c \
../Core/Src/trigger_decoder.c \
../Core/Src/trigger_recorder.c 

OBJS += \
./Core/Src/board_safety_stm32.o \
./Core/Src/diagnostics_can.o \
./Core/Src/engine_control.o \
./Core/Src/engine_watchdog_stm32.o \
./Core/Src/injection.o \
./Core/Src/main.o \
./Core/Src/spark.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/trigger_capture_stm32.o \
./Core/Src/trigger_decoder.o \
./Core/Src/trigger_recorder.o 

C_DEPS += \
./Core/Src/board_safety_stm32.d \
./Core/Src/diagnostics_can.d \
./Core/Src/engine_control.d \
./Core/Src/engine_watchdog_stm32.d \
./Core/Src/injection.d \
./Core/Src/main.d \
./Core/Src/spark.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/trigger_capture_stm32.d \
./Core/Src/trigger_decoder.d \
./Core/Src/trigger_recorder.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/board_safety_stm32.cyclo ./Core/Src/board_safety_stm32.d ./Core/Src/board_safety_stm32.o ./Core/Src/board_safety_stm32.su ./Core/Src/diagnostics_can.cyclo ./Core/Src/diagnostics_can.d ./Core/Src/diagnostics_can.o ./Core/Src/diagnostics_can.su ./Core/Src/engine_control.cyclo ./Core/Src/engine_control.d ./Core/Src/engine_control.o ./Core/Src/engine_control.su ./Core/Src/engine_watchdog_stm32.cyclo ./Core/Src/engine_watchdog_stm32.d ./Core/Src/engine_watchdog_stm32.o ./Core/Src/engine_watchdog_stm32.su ./Core/Src/injection.cyclo ./Core/Src/injection.d ./Core/Src/injection.o ./Core/Src/injection.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/spark.cyclo ./Core/Src/spark.d ./Core/Src/spark.o ./Core/Src/spark.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/trigger_capture_stm32.cyclo ./Core/Src/trigger_capture_stm32.d ./Core/Src/trigger_capture_stm32.o ./Core/Src/trigger_capture_stm32.su ./Core/Src/trigger_decoder.cyclo ./Core/Src/trigger_decoder.d ./Core/Src/trigger_decoder.o ./Core/Src/trigger_decoder.su ./Core/Src/trigger_recorder.cyclo ./Core/Src/trigger_recorder.d ./Core/Src/trigger_recorder.o ./Core/Src/trigger_recorder.su

.PHONY: clean-Core-2f-Src

