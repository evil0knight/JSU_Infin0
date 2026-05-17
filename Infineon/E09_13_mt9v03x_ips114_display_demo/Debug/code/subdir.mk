################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/search_line.c 

COMPILED_SRCS += \
code/search_line.src 

C_DEPS += \
code/search_line.d 

OBJS += \
code/search_line.o 


# Each subdirectory must supply rules for building sources it contributes
code/search_line.src: ../code/search_line.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/E09_13_mt9v03x_ips114_display_demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/search_line.o: code/search_line.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/search_line.d code/search_line.o code/search_line.src

.PHONY: clean-code

