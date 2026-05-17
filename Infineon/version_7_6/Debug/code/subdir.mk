################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/balance.c \
../code/color_tracer.c \
../code/dianzhen.c \
../code/extern.c \
../code/garage.c \
../code/guandao.c \
../code/gyro.c \
../code/karman.c \
../code/my_math.c \
../code/rampway.c \
../code/roadblock.c \
../code/roundabout.c \
../code/rtk.c \
../code/zf_device_dot_matrix_screen.c \
../code/zf_device_tld7002.c 

COMPILED_SRCS += \
code/balance.src \
code/color_tracer.src \
code/dianzhen.src \
code/extern.src \
code/garage.src \
code/guandao.src \
code/gyro.src \
code/karman.src \
code/my_math.src \
code/rampway.src \
code/roadblock.src \
code/roundabout.src \
code/rtk.src \
code/zf_device_dot_matrix_screen.src \
code/zf_device_tld7002.src 

C_DEPS += \
code/balance.d \
code/color_tracer.d \
code/dianzhen.d \
code/extern.d \
code/garage.d \
code/guandao.d \
code/gyro.d \
code/karman.d \
code/my_math.d \
code/rampway.d \
code/roadblock.d \
code/roundabout.d \
code/rtk.d \
code/zf_device_dot_matrix_screen.d \
code/zf_device_tld7002.d 

OBJS += \
code/balance.o \
code/color_tracer.o \
code/dianzhen.o \
code/extern.o \
code/garage.o \
code/guandao.o \
code/gyro.o \
code/karman.o \
code/my_math.o \
code/rampway.o \
code/roadblock.o \
code/roundabout.o \
code/rtk.o \
code/zf_device_dot_matrix_screen.o \
code/zf_device_tld7002.o 


# Each subdirectory must supply rules for building sources it contributes
code/balance.src: ../code/balance.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/balance.o: code/balance.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/color_tracer.src: ../code/color_tracer.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/color_tracer.o: code/color_tracer.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/dianzhen.src: ../code/dianzhen.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/dianzhen.o: code/dianzhen.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/extern.src: ../code/extern.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/extern.o: code/extern.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/garage.src: ../code/garage.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/garage.o: code/garage.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/guandao.src: ../code/guandao.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/guandao.o: code/guandao.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/gyro.src: ../code/gyro.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/gyro.o: code/gyro.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/karman.src: ../code/karman.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/karman.o: code/karman.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/my_math.src: ../code/my_math.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/my_math.o: code/my_math.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/rampway.src: ../code/rampway.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/rampway.o: code/rampway.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/roadblock.src: ../code/roadblock.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/roadblock.o: code/roadblock.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/roundabout.src: ../code/roundabout.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/roundabout.o: code/roundabout.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/rtk.src: ../code/rtk.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/rtk.o: code/rtk.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/zf_device_dot_matrix_screen.src: ../code/zf_device_dot_matrix_screen.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/zf_device_dot_matrix_screen.o: code/zf_device_dot_matrix_screen.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/zf_device_tld7002.src: ../code/zf_device_tld7002.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/WHY/Desktop/智能车/verison_7_6/verison_7_6/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/zf_device_tld7002.o: code/zf_device_tld7002.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/balance.d code/balance.o code/balance.src code/color_tracer.d code/color_tracer.o code/color_tracer.src code/dianzhen.d code/dianzhen.o code/dianzhen.src code/extern.d code/extern.o code/extern.src code/garage.d code/garage.o code/garage.src code/guandao.d code/guandao.o code/guandao.src code/gyro.d code/gyro.o code/gyro.src code/karman.d code/karman.o code/karman.src code/my_math.d code/my_math.o code/my_math.src code/rampway.d code/rampway.o code/rampway.src code/roadblock.d code/roadblock.o code/roadblock.src code/roundabout.d code/roundabout.o code/roundabout.src code/rtk.d code/rtk.o code/rtk.src code/zf_device_dot_matrix_screen.d code/zf_device_dot_matrix_screen.o code/zf_device_dot_matrix_screen.src code/zf_device_tld7002.d code/zf_device_tld7002.o code/zf_device_tld7002.src

.PHONY: clean-code

