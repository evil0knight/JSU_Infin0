################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuCcu.c \
../libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuEru.c \
../libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuLbist.c \
../libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuRcu.c \
../libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuWdt.c 

COMPILED_SRCS += \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuCcu.src \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuEru.src \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuLbist.src \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuRcu.src \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuWdt.src 

C_DEPS += \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuCcu.d \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuEru.d \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuLbist.d \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuRcu.d \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuWdt.d 

OBJS += \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuCcu.o \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuEru.o \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuLbist.o \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuRcu.o \
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuWdt.o 


# Each subdirectory must supply rules for building sources it contributes
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuCcu.src: ../libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuCcu.c libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuCcu.o: libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuCcu.src libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuEru.src: ../libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuEru.c libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuEru.o: libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuEru.src libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuLbist.src: ../libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuLbist.c libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuLbist.o: libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuLbist.src libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuRcu.src: ../libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuRcu.c libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuRcu.o: libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuRcu.src libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuWdt.src: ../libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuWdt.c libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuWdt.o: libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuWdt.src libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-Scu-2f-Std

clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-Scu-2f-Std:
	-$(RM) libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuCcu.d libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuCcu.o libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuCcu.src libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuEru.d libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuEru.o libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuEru.src libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuLbist.d libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuLbist.o libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuLbist.src libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuRcu.d libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuRcu.o libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuRcu.src libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuWdt.d libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuWdt.o libraries/infineon_libraries/iLLD/TC37A/Tricore/Scu/Std/IfxScuWdt.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-Scu-2f-Std

