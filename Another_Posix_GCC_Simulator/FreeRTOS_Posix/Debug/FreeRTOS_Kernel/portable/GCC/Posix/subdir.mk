################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS_Kernel/portable/GCC/Posix/fifo.c \
../FreeRTOS_Kernel/portable/GCC/Posix/itc.c \
../FreeRTOS_Kernel/portable/GCC/Posix/port.c \
../FreeRTOS_Kernel/portable/GCC/Posix/rbsem.c \
../FreeRTOS_Kernel/portable/GCC/Posix/trace.c \
../FreeRTOS_Kernel/portable/GCC/Posix/util.c 

OBJS += \
./FreeRTOS_Kernel/portable/GCC/Posix/fifo.o \
./FreeRTOS_Kernel/portable/GCC/Posix/itc.o \
./FreeRTOS_Kernel/portable/GCC/Posix/port.o \
./FreeRTOS_Kernel/portable/GCC/Posix/rbsem.o \
./FreeRTOS_Kernel/portable/GCC/Posix/trace.o \
./FreeRTOS_Kernel/portable/GCC/Posix/util.o 

C_DEPS += \
./FreeRTOS_Kernel/portable/GCC/Posix/fifo.d \
./FreeRTOS_Kernel/portable/GCC/Posix/itc.d \
./FreeRTOS_Kernel/portable/GCC/Posix/port.d \
./FreeRTOS_Kernel/portable/GCC/Posix/rbsem.d \
./FreeRTOS_Kernel/portable/GCC/Posix/trace.d \
./FreeRTOS_Kernel/portable/GCC/Posix/util.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS_Kernel/portable/GCC/Posix/%.o: ../FreeRTOS_Kernel/portable/GCC/Posix/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -D__GCC_POSIX__=1 -DDEBUG_BUILD=1 -DUSE_STDIO=1 -I../Common_Demo/include -I.. -I../FreeRTOS_Kernel/include -I../FreeRTOS_Kernel/portable/GCC/Posix -O0 -g -Wall -c -fmessage-length=0 -pthread -lrt -Wno-pointer-sign -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


