#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

typedef struct {
    float x;
    float y;
    float z;
} imu_val;//xyz轴

typedef struct {
    /* Gyroscope values are in deg/s; accelerometer values are in g. */
    imu_val gyro;//角速度
    imu_val accel;//加速度
    float roll;//翻滚角
    float pitch;//俯仰角
    float yaw;//偏航角
} Imu_t;

typedef struct {
    imu_val gyro_bias;//陀螺仪零偏
    float yaw_rate;//滤波后的偏航角速度
    float temperature_c;//温度
    uint32_t sample_count;//成功更新的采样次数
    uint32_t read_error_count;//读取失败次数
    uint8_t calibrated;//是否已经完成初始化和校准
    uint8_t stationary;//当前车辆是否处于静止状态
} MPU6050_Status_t;//MPU6050状态

/*
 * Mounting convention used by the software fusion:
 *   sensor +Y: vehicle forward
 *   sensor +X: vehicle right
 *   sensor +Z: vehicle up
 * Therefore vehicle yaw is rotation about sensor Z, not sensor Y.
 * Change this sign only if a clockwise physical turn produces the wrong
 * yaw sign for the route controller.
 */
#define MPU6050_YAW_GYRO_SIGN       (1.0f)

extern Imu_t mpu6050;
extern MPU6050_Status_t mpu6050_status;

/*
 * Configure raw sampling at 200 Hz and perform a stationary gyro calibration.
 * Keep the vehicle completely still during the approximately 4-second
 * calibration. Returns 1 on success and 0 on an I2C/device/calibration error.
 */
uint8_t MPU6050_initialize(void);

/*
 * Read one raw sample and run the software filters.
 * vehicle_stopped must be non-zero only when the chassis is commanded to stop;
 * this allows safe online gyro-bias tracking without cancelling slow turns.
 */
uint8_t MPU6050_Update(uint8_t vehicle_stopped);

/* Set the current software yaw reference, normally to 0 degrees. */
void MPU6050_ResetYaw(float yaw_deg);

uint8_t MPU6050_testConnection(void);
int Read_Temperature(void);

#endif
