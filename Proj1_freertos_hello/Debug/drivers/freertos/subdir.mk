################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/freertos/fsl_dspi_freertos.c 

C_DEPS += \
./drivers/freertos/fsl_dspi_freertos.d 

OBJS += \
./drivers/freertos/fsl_dspi_freertos.o 


# Each subdirectory must supply rules for building sources it contributes
drivers/freertos/%.o: ../drivers/freertos/%.c drivers/freertos/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MK66FN2M0VMD18 -DCPU_MK66FN2M0VMD18_cm4 -DFRDM_K66F -DFREEDOM -DSERIAL_PORT_TYPE_UART=1 -DSDK_OS_FREE_RTOS -DMCUXPRESSO_SDK -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\drivers" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\device" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\utilities" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\component\uart" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\component\serial_manager" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\component\lists" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\CMSIS" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\freertos\freertos_kernel\include" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\freertos\freertos_kernel\portable\GCC\ARM_CM4F" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\drivers\freertos" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\board" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\source" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\source" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\drivers" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\device" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\utilities" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\component\uart" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\component\serial_manager" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\component\lists" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\CMSIS" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\freertos\freertos_kernel\include" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\freertos\freertos_kernel\portable\GCC\ARM_CM4F" -I"C:\Users\markanda\Documents\4DS4\Project1\Proj1_freertos_hello\board" -O0 -fno-common -g3 -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-drivers-2f-freertos

clean-drivers-2f-freertos:
	-$(RM) ./drivers/freertos/fsl_dspi_freertos.d ./drivers/freertos/fsl_dspi_freertos.o

.PHONY: clean-drivers-2f-freertos

