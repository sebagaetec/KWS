################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../X-CUBE-AI/App/sine_model.c \
../X-CUBE-AI/App/sine_model_data.c \
../X-CUBE-AI/App/sine_model_data_params.c 

OBJS += \
./X-CUBE-AI/App/sine_model.o \
./X-CUBE-AI/App/sine_model_data.o \
./X-CUBE-AI/App/sine_model_data_params.o 

C_DEPS += \
./X-CUBE-AI/App/sine_model.d \
./X-CUBE-AI/App/sine_model_data.d \
./X-CUBE-AI/App/sine_model_data_params.d 


# Each subdirectory must supply rules for building sources it contributes
X-CUBE-AI/App/%.o X-CUBE-AI/App/%.su X-CUBE-AI/App/%.cyclo: ../X-CUBE-AI/App/%.c X-CUBE-AI/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/AI/Inc -I../X-CUBE-AI/App -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-X-2d-CUBE-2d-AI-2f-App

clean-X-2d-CUBE-2d-AI-2f-App:
	-$(RM) ./X-CUBE-AI/App/sine_model.cyclo ./X-CUBE-AI/App/sine_model.d ./X-CUBE-AI/App/sine_model.o ./X-CUBE-AI/App/sine_model.su ./X-CUBE-AI/App/sine_model_data.cyclo ./X-CUBE-AI/App/sine_model_data.d ./X-CUBE-AI/App/sine_model_data.o ./X-CUBE-AI/App/sine_model_data.su ./X-CUBE-AI/App/sine_model_data_params.cyclo ./X-CUBE-AI/App/sine_model_data_params.d ./X-CUBE-AI/App/sine_model_data_params.o ./X-CUBE-AI/App/sine_model_data_params.su

.PHONY: clean-X-2d-CUBE-2d-AI-2f-App

