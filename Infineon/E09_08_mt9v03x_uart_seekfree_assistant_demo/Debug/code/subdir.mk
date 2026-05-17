################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/image.c 

COMPILED_SRCS += \
code/image.src 

C_DEPS += \
code/image.d 

OBJS += \
code/image.o 


# Each subdirectory must supply rules for building sources it contributes
code/image.src: ../code/image.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/E09_08_mt9v03x_uart_seekfree_assistant_demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/image.o: code/image.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/image.d code/image.o code/image.src

.PHONY: clean-code

