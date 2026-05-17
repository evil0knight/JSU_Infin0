################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/small_driver_uart_control.c 

COMPILED_SRCS += \
code/small_driver_uart_control.src 

C_DEPS += \
code/small_driver_uart_control.d 

OBJS += \
code/small_driver_uart_control.o 


# Each subdirectory must supply rules for building sources it contributes
code/small_driver_uart_control.src: ../code/small_driver_uart_control.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/E02_07_foc_double_driver_demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/small_driver_uart_control.o: code/small_driver_uart_control.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/small_driver_uart_control.d code/small_driver_uart_control.o code/small_driver_uart_control.src

.PHONY: clean-code

