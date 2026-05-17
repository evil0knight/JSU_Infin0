################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.c \
../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.c \
../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.c \
../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.c 

COMPILED_SRCS += \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.src \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.src \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.src \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.src 

C_DEPS += \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.d \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.d \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.d \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.d 

OBJS += \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.o \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.o \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.o \
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.o 


# Each subdirectory must supply rules for building sources it contributes
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.src: ../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.c libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.o: libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.src libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.src: ../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.c libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.o: libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.src libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.src: ../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.c libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.o: libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.src libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.src: ../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.c libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.o: libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.src libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-infineon_libraries-2f-Infra-2f-Ssw-2f-TC37A-2f-Tricore

clean-libraries-2f-infineon_libraries-2f-Infra-2f-Ssw-2f-TC37A-2f-Tricore:
	-$(RM) libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.d libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.o libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.src libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.d libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.o libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.src libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.d libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.o libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.src libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.d libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.o libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-Infra-2f-Ssw-2f-TC37A-2f-Tricore

