################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AIO/AIO.c \
../AIO/AIOUnixSocket.c 

OBJS += \
./AIO/AIO.o \
./AIO/AIOUnixSocket.o 

C_DEPS += \
./AIO/AIO.d \
./AIO/AIOUnixSocket.d 


# Each subdirectory must supply rules for building sources it contributes
AIO/%.o: ../AIO/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -D__GCC_POSIX__=1 -DDEBUG_BUILD=1 -DUSE_STDIO=1 -I../Common_Demo/include -I.. -I../FreeRTOS_Kernel/include -I../FreeRTOS_Kernel/portable/GCC/Posix -O0 -g -Wall -c -fmessage-length=0 -pthread -lrt -Wno-pointer-sign -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


