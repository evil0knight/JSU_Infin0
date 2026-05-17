################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/TLD7002_driver/TLD7002FuncLayer.c \
../code/TLD7002_driver/TLD7002_ControlLayer.c \
../code/TLD7002_driver/TLD7002_ServiceLayer.c 

COMPILED_SRCS += \
code/TLD7002_driver/TLD7002FuncLayer.src \
code/TLD7002_driver/TLD7002_ControlLayer.src \
code/TLD7002_driver/TLD7002_ServiceLayer.src 

C_DEPS += \
code/TLD7002_driver/TLD7002FuncLayer.d \
code/TLD7002_driver/TLD7002_ControlLayer.d \
code/TLD7002_driver/TLD7002_ServiceLayer.d 

OBJS += \
code/TLD7002_driver/TLD7002FuncLayer.o \
code/TLD7002_driver/TLD7002_ControlLayer.o \
code/TLD7002_driver/TLD7002_ServiceLayer.o 


# Each subdirectory must supply rules for building sources it contributes
code/TLD7002_driver/TLD7002FuncLayer.src: ../code/TLD7002_driver/TLD7002FuncLayer.c code/TLD7002_driver/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/TLD7002_driver/TLD7002FuncLayer.o: code/TLD7002_driver/TLD7002FuncLayer.src code/TLD7002_driver/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/TLD7002_driver/TLD7002_ControlLayer.src: ../code/TLD7002_driver/TLD7002_ControlLayer.c code/TLD7002_driver/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/TLD7002_driver/TLD7002_ControlLayer.o: code/TLD7002_driver/TLD7002_ControlLayer.src code/TLD7002_driver/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/TLD7002_driver/TLD7002_ServiceLayer.src: ../code/TLD7002_driver/TLD7002_ServiceLayer.c code/TLD7002_driver/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/TLD7002_driver/TLD7002_ServiceLayer.o: code/TLD7002_driver/TLD7002_ServiceLayer.src code/TLD7002_driver/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-TLD7002_driver

clean-code-2f-TLD7002_driver:
	-$(RM) code/TLD7002_driver/TLD7002FuncLayer.d code/TLD7002_driver/TLD7002FuncLayer.o code/TLD7002_driver/TLD7002FuncLayer.src code/TLD7002_driver/TLD7002_ControlLayer.d code/TLD7002_driver/TLD7002_ControlLayer.o code/TLD7002_driver/TLD7002_ControlLayer.src code/TLD7002_driver/TLD7002_ServiceLayer.d code/TLD7002_driver/TLD7002_ServiceLayer.o code/TLD7002_driver/TLD7002_ServiceLayer.src

.PHONY: clean-code-2f-TLD7002_driver

