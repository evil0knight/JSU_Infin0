################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/init.c \
../code/vmc.c 

COMPILED_SRCS += \
code/init.src \
code/vmc.src 

C_DEPS += \
code/init.d \
code/vmc.d 

OBJS += \
code/init.o \
code/vmc.o 


# Each subdirectory must supply rules for building sources it contributes
code/init.src: ../code/init.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/seekfree_assistant_parameter_debug_demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/init.o: code/init.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/vmc.src: ../code/vmc.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/seekfree_assistant_parameter_debug_demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/vmc.o: code/vmc.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/init.d code/init.o code/init.src code/vmc.d code/vmc.o code/vmc.src

.PHONY: clean-code

