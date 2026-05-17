################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/IfxCif.c 

COMPILED_SRCS += \
libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/IfxCif.src 

C_DEPS += \
libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/IfxCif.d 

OBJS += \
libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/IfxCif.o 


# Each subdirectory must supply rules for building sources it contributes
libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/IfxCif.src: ../libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/IfxCif.c libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/E09_08_mt9v03x_uart_seekfree_assistant_demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/IfxCif.o: libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/IfxCif.src libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-Cif-2f-Std

clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-Cif-2f-Std:
	-$(RM) libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/IfxCif.d libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/IfxCif.o libraries/infineon_libraries/iLLD/TC26B/Tricore/Cif/Std/IfxCif.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-Cif-2f-Std

