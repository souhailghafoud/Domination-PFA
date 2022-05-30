idf_component_register(
    SRCS "imu.c" "mpu6050.c"
    INCLUDE_DIRS "."
    REQUIRES Peripherals)