################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/Adaptive_binarization.c \
../code/Atime.c \
../code/Base.c \
../code/Inverse_kinematics.c \
../code/KF_imu_encoder_fusion.c \
../code/KalmanFilter.c \
../code/Kalman_fusion_of_imu963ra.c \
../code/LQR.c \
../code/PID.c \
../code/barrier.c \
../code/circle.c \
../code/common.c \
../code/cross.c \
../code/database.c \
../code/date.c \
../code/enconder.c \
../code/foc.c \
../code/garage.c \
../code/get_corners.c \
../code/handle_img.c \
../code/img_process.c \
../code/interactive_interface.c \
../code/key_status.c \
../code/logger.c \
../code/lost.c \
../code/matrix.c \
../code/menu.c \
../code/state.c \
../code/steering_engine.c \
../code/tracking.c \
../code/transform_table.c \
../code/universal_filter.c \
../code/wifi_lineless.c 

COMPILED_SRCS += \
code/Adaptive_binarization.src \
code/Atime.src \
code/Base.src \
code/Inverse_kinematics.src \
code/KF_imu_encoder_fusion.src \
code/KalmanFilter.src \
code/Kalman_fusion_of_imu963ra.src \
code/LQR.src \
code/PID.src \
code/barrier.src \
code/circle.src \
code/common.src \
code/cross.src \
code/database.src \
code/date.src \
code/enconder.src \
code/foc.src \
code/garage.src \
code/get_corners.src \
code/handle_img.src \
code/img_process.src \
code/interactive_interface.src \
code/key_status.src \
code/logger.src \
code/lost.src \
code/matrix.src \
code/menu.src \
code/state.src \
code/steering_engine.src \
code/tracking.src \
code/transform_table.src \
code/universal_filter.src \
code/wifi_lineless.src 

C_DEPS += \
code/Adaptive_binarization.d \
code/Atime.d \
code/Base.d \
code/Inverse_kinematics.d \
code/KF_imu_encoder_fusion.d \
code/KalmanFilter.d \
code/Kalman_fusion_of_imu963ra.d \
code/LQR.d \
code/PID.d \
code/barrier.d \
code/circle.d \
code/common.d \
code/cross.d \
code/database.d \
code/date.d \
code/enconder.d \
code/foc.d \
code/garage.d \
code/get_corners.d \
code/handle_img.d \
code/img_process.d \
code/interactive_interface.d \
code/key_status.d \
code/logger.d \
code/lost.d \
code/matrix.d \
code/menu.d \
code/state.d \
code/steering_engine.d \
code/tracking.d \
code/transform_table.d \
code/universal_filter.d \
code/wifi_lineless.d 

OBJS += \
code/Adaptive_binarization.o \
code/Atime.o \
code/Base.o \
code/Inverse_kinematics.o \
code/KF_imu_encoder_fusion.o \
code/KalmanFilter.o \
code/Kalman_fusion_of_imu963ra.o \
code/LQR.o \
code/PID.o \
code/barrier.o \
code/circle.o \
code/common.o \
code/cross.o \
code/database.o \
code/date.o \
code/enconder.o \
code/foc.o \
code/garage.o \
code/get_corners.o \
code/handle_img.o \
code/img_process.o \
code/interactive_interface.o \
code/key_status.o \
code/logger.o \
code/lost.o \
code/matrix.o \
code/menu.o \
code/state.o \
code/steering_engine.o \
code/tracking.o \
code/transform_table.o \
code/universal_filter.o \
code/wifi_lineless.o 


# Each subdirectory must supply rules for building sources it contributes
code/Adaptive_binarization.src: ../code/Adaptive_binarization.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/Adaptive_binarization.o: code/Adaptive_binarization.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/Atime.src: ../code/Atime.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/Atime.o: code/Atime.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/Base.src: ../code/Base.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/Base.o: code/Base.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/Inverse_kinematics.src: ../code/Inverse_kinematics.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/Inverse_kinematics.o: code/Inverse_kinematics.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/KF_imu_encoder_fusion.src: ../code/KF_imu_encoder_fusion.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/KF_imu_encoder_fusion.o: code/KF_imu_encoder_fusion.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/KalmanFilter.src: ../code/KalmanFilter.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/KalmanFilter.o: code/KalmanFilter.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/Kalman_fusion_of_imu963ra.src: ../code/Kalman_fusion_of_imu963ra.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/Kalman_fusion_of_imu963ra.o: code/Kalman_fusion_of_imu963ra.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/LQR.src: ../code/LQR.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/LQR.o: code/LQR.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/PID.src: ../code/PID.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/PID.o: code/PID.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/barrier.src: ../code/barrier.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/barrier.o: code/barrier.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/circle.src: ../code/circle.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/circle.o: code/circle.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/common.src: ../code/common.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/common.o: code/common.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/cross.src: ../code/cross.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/cross.o: code/cross.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/database.src: ../code/database.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/database.o: code/database.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/date.src: ../code/date.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/date.o: code/date.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/enconder.src: ../code/enconder.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/enconder.o: code/enconder.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/foc.src: ../code/foc.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/foc.o: code/foc.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/garage.src: ../code/garage.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/garage.o: code/garage.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/get_corners.src: ../code/get_corners.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/get_corners.o: code/get_corners.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/handle_img.src: ../code/handle_img.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/handle_img.o: code/handle_img.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/img_process.src: ../code/img_process.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/img_process.o: code/img_process.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/interactive_interface.src: ../code/interactive_interface.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/interactive_interface.o: code/interactive_interface.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/key_status.src: ../code/key_status.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/key_status.o: code/key_status.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/logger.src: ../code/logger.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/logger.o: code/logger.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/lost.src: ../code/lost.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/lost.o: code/lost.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/matrix.src: ../code/matrix.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/matrix.o: code/matrix.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/menu.src: ../code/menu.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/menu.o: code/menu.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/state.src: ../code/state.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/state.o: code/state.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/steering_engine.src: ../code/steering_engine.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/steering_engine.o: code/steering_engine.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/tracking.src: ../code/tracking.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/tracking.o: code/tracking.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/transform_table.src: ../code/transform_table.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/transform_table.o: code/transform_table.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/universal_filter.src: ../code/universal_filter.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/universal_filter.o: code/universal_filter.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/wifi_lineless.src: ../code/wifi_lineless.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Infineon/wheel_legged_robot_2.7/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++11 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/wifi_lineless.o: code/wifi_lineless.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/Adaptive_binarization.d code/Adaptive_binarization.o code/Adaptive_binarization.src code/Atime.d code/Atime.o code/Atime.src code/Base.d code/Base.o code/Base.src code/Inverse_kinematics.d code/Inverse_kinematics.o code/Inverse_kinematics.src code/KF_imu_encoder_fusion.d code/KF_imu_encoder_fusion.o code/KF_imu_encoder_fusion.src code/KalmanFilter.d code/KalmanFilter.o code/KalmanFilter.src code/Kalman_fusion_of_imu963ra.d code/Kalman_fusion_of_imu963ra.o code/Kalman_fusion_of_imu963ra.src code/LQR.d code/LQR.o code/LQR.src code/PID.d code/PID.o code/PID.src code/barrier.d code/barrier.o code/barrier.src code/circle.d code/circle.o code/circle.src code/common.d code/common.o code/common.src code/cross.d code/cross.o code/cross.src code/database.d code/database.o code/database.src code/date.d code/date.o code/date.src code/enconder.d code/enconder.o code/enconder.src code/foc.d code/foc.o code/foc.src code/garage.d code/garage.o code/garage.src code/get_corners.d code/get_corners.o code/get_corners.src code/handle_img.d code/handle_img.o code/handle_img.src code/img_process.d code/img_process.o code/img_process.src code/interactive_interface.d code/interactive_interface.o code/interactive_interface.src code/key_status.d code/key_status.o code/key_status.src code/logger.d code/logger.o code/logger.src code/lost.d code/lost.o code/lost.src code/matrix.d code/matrix.o code/matrix.src code/menu.d code/menu.o code/menu.src code/state.d code/state.o code/state.src code/steering_engine.d code/steering_engine.o code/steering_engine.src code/tracking.d code/tracking.o code/tracking.src code/transform_table.d code/transform_table.o code/transform_table.src code/universal_filter.d code/universal_filter.o code/universal_filter.src code/wifi_lineless.d code/wifi_lineless.o code/wifi_lineless.src

.PHONY: clean-code

