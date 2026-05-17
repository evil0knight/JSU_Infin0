################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/IfxAsclin_Spi.c 

COMPILED_SRCS += \
libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/IfxAsclin_Spi.src 

C_DEPS += \
libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/IfxAsclin_Spi.d 

OBJS += \
libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/IfxAsclin_Spi.o 


# Each subdirectory must supply rules for building sources it contributes
libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/IfxAsclin_Spi.src: ../libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/IfxAsclin_Spi.c libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/E05_03_imu660ra_demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/IfxAsclin_Spi.o: libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/IfxAsclin_Spi.src libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-Asclin-2f-Spi

clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-Asclin-2f-Spi:
	-$(RM) libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/IfxAsclin_Spi.d libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/IfxAsclin_Spi.o libraries/infineon_libraries/iLLD/TC26B/Tricore/Asclin/Spi/IfxAsclin_Spi.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-Asclin-2f-Spi

