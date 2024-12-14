################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Simulink/downpart_accel_ert_rtw/downpart_accel.c 

OBJS += \
./Drivers/Simulink/downpart_accel_ert_rtw/downpart_accel.o 

C_DEPS += \
./Drivers/Simulink/downpart_accel_ert_rtw/downpart_accel.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Simulink/downpart_accel_ert_rtw/%.o Drivers/Simulink/downpart_accel_ert_rtw/%.su Drivers/Simulink/downpart_accel_ert_rtw/%.cyclo: ../Drivers/Simulink/downpart_accel_ert_rtw/%.c Drivers/Simulink/downpart_accel_ert_rtw/subdir.mk
	arm-none-eabi-gcc -gdwarf-4 "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/Simulink -I../Drivers/Simulink/downpart_accel_ert_rtw -O0 -ffunction-sections -fdata-sections -Wall -gdwarf-3 -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Simulink-2f-downpart_accel_ert_rtw

clean-Drivers-2f-Simulink-2f-downpart_accel_ert_rtw:
	-$(RM) ./Drivers/Simulink/downpart_accel_ert_rtw/downpart_accel.cyclo ./Drivers/Simulink/downpart_accel_ert_rtw/downpart_accel.d ./Drivers/Simulink/downpart_accel_ert_rtw/downpart_accel.o ./Drivers/Simulink/downpart_accel_ert_rtw/downpart_accel.su

.PHONY: clean-Drivers-2f-Simulink-2f-downpart_accel_ert_rtw

