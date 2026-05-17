################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/IfxI2c_I2c.c 

COMPILED_SRCS += \
libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/IfxI2c_I2c.src 

C_DEPS += \
libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/IfxI2c_I2c.d 

OBJS += \
libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/IfxI2c_I2c.o 


# Each subdirectory must supply rules for building sources it contributes
libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/IfxI2c_I2c.src: ../libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/IfxI2c_I2c.c libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/E05_03_imu660ra_demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/IfxI2c_I2c.o: libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/IfxI2c_I2c.src libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-I2c-2f-I2c

clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-I2c-2f-I2c:
	-$(RM) libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/IfxI2c_I2c.d libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/IfxI2c_I2c.o libraries/infineon_libraries/iLLD/TC26B/Tricore/I2c/I2c/IfxI2c_I2c.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-I2c-2f-I2c

