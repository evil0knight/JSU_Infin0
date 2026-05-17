################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/PID.c \
../code/ekf.c \
../code/image.c \
../code/init.c \
../code/island.c \
../code/isp200_show.c \
../code/key_status.c \
../code/leg_flexible.c \
../code/matrix.c \
../code/menu.c \
../code/search_line.c \
../code/small_driver_uart_control.c \
../code/vmc.c 

COMPILED_SRCS += \
code/PID.src \
code/ekf.src \
code/image.src \
code/init.src \
code/island.src \
code/isp200_show.src \
code/key_status.src \
code/leg_flexible.src \
code/matrix.src \
code/menu.src \
code/search_line.src \
code/small_driver_uart_control.src \
code/vmc.src 

C_DEPS += \
code/PID.d \
code/ekf.d \
code/image.d \
code/init.d \
code/island.d \
code/isp200_show.d \
code/key_status.d \
code/leg_flexible.d \
code/matrix.d \
code/menu.d \
code/search_line.d \
code/small_driver_uart_control.d \
code/vmc.d 

OBJS += \
code/PID.o \
code/ekf.o \
code/image.o \
code/init.o \
code/island.o \
code/isp200_show.o \
code/key_status.o \
code/leg_flexible.o \
code/matrix.o \
code/menu.o \
code/search_line.o \
code/small_driver_uart_control.o \
code/vmc.o 


# Each subdirectory must supply rules for building sources it contributes
code/PID.src: ../code/PID.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/PID.o: code/PID.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/ekf.src: ../code/ekf.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/ekf.o: code/ekf.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/image.src: ../code/image.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/image.o: code/image.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/init.src: ../code/init.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/init.o: code/init.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/island.src: ../code/island.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/island.o: code/island.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/isp200_show.src: ../code/isp200_show.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/isp200_show.o: code/isp200_show.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/key_status.src: ../code/key_status.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/key_status.o: code/key_status.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/leg_flexible.src: ../code/leg_flexible.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/leg_flexible.o: code/leg_flexible.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/matrix.src: ../code/matrix.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/matrix.o: code/matrix.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/menu.src: ../code/menu.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/menu.o: code/menu.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/search_line.src: ../code/search_line.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/search_line.o: code/search_line.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/small_driver_uart_control.src: ../code/small_driver_uart_control.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/small_driver_uart_control.o: code/small_driver_uart_control.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/vmc.src: ../code/vmc.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Infineon/Seekfree_TC264_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/vmc.o: code/vmc.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/PID.d code/PID.o code/PID.src code/ekf.d code/ekf.o code/ekf.src code/image.d code/image.o code/image.src code/init.d code/init.o code/init.src code/island.d code/island.o code/island.src code/isp200_show.d code/isp200_show.o code/isp200_show.src code/key_status.d code/key_status.o code/key_status.src code/leg_flexible.d code/leg_flexible.o code/leg_flexible.src code/matrix.d code/matrix.o code/matrix.src code/menu.d code/menu.o code/menu.src code/search_line.d code/search_line.o code/search_line.src code/small_driver_uart_control.d code/small_driver_uart_control.o code/small_driver_uart_control.src code/vmc.d code/vmc.o code/vmc.src

.PHONY: clean-code

