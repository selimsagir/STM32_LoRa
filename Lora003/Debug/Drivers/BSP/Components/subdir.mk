################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Drivers/BSP/Components/sx1276/sx1276.c 

OBJS += \
./Drivers/BSP/Components/sx1276.o 

C_DEPS += \
./Drivers/BSP/Components/sx1276.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/sx1276.o: C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Drivers/BSP/Components/sx1276/sx1276.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L072xx -I"C:/Users/Asus/SelimAll/STM32_works/Lora003/Inc" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Projects/B-L072Z-LRWAN1/Applications/LoRa/PingPong/Core" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Projects/STM32L073RZ-Nucleo/Applications/LoRa/PingPong/LoRaWAN/App/inc/hw.h" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Projects/STM32L073RZ-Nucleo/Applications/LoRa/PingPong/LoRaWAN/App/inc" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Drivers/BSP/X_NUCLEO_IKS01A1" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Middlewares/Third_Party/LoRaWAN/Utilities" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Middlewares/Third_Party/LoRaWAN/Phy" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Middlewares/Third_Party/LoRaWAN/Mac" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Middlewares/Third_Party/LoRaWAN/Crypto" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Middlewares/Third_Party/LoRaWAN/Core" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Drivers/CMSIS/Include" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Drivers/BSP/Components/sx1276" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Drivers/BSP/Components/lps25hb" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Drivers/BSP/Components/hts221" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Drivers/BSP/Components/Common" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Drivers/BSP/CMWX1ZZABZ-0xx" -I"C:/Users/Asus/SelimAll/LoRa_Code/en.i-cube_lrwan/STM32CubeExpansion_LRWAN_V1.2.1/Drivers/BSP/B-L072Z-LRWAN1" -I"C:/Users/Asus/SelimAll/STM32_works/Lora003/Drivers/STM32L0xx_HAL_Driver/Inc" -I"C:/Users/Asus/SelimAll/STM32_works/Lora003/Drivers/STM32L0xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Asus/SelimAll/STM32_works/Lora003/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"C:/Users/Asus/SelimAll/STM32_works/Lora003/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


