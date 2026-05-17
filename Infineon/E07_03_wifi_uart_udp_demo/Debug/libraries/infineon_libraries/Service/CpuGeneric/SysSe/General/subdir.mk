################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/Ifx_GlobalResources.c 

COMPILED_SRCS += \
libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/Ifx_GlobalResources.src 

C_DEPS += \
libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/Ifx_GlobalResources.d 

OBJS += \
libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/Ifx_GlobalResources.o 


# Each subdirectory must supply rules for building sources it contributes
libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/Ifx_GlobalResources.src: ../libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/Ifx_GlobalResources.c libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/E07_03_wifi_uart_udp_demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/Ifx_GlobalResources.o: libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/Ifx_GlobalResources.src libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-infineon_libraries-2f-Service-2f-CpuGeneric-2f-SysSe-2f-General

clean-libraries-2f-infineon_libraries-2f-Service-2f-CpuGeneric-2f-SysSe-2f-General:
	-$(RM) libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/Ifx_GlobalResources.d libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/Ifx_GlobalResources.o libraries/infineon_libraries/Service/CpuGeneric/SysSe/General/Ifx_GlobalResources.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-Service-2f-CpuGeneric-2f-SysSe-2f-General

