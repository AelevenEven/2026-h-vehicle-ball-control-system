#include "MPU6050.h"

#include "board.h"
#include "bsp_siic.h"

#include <math.h>
#include <stddef.h>

#define MPU6050_ADDRESS_8BIT            (0x68U << 1)

#define MPU6050_REG_SMPLRT_DIV          0x19U            //采样率分频
#define MPU6050_REG_CONFIG              0x1AU						 //DLPF（数字低通滤波）低通滤波配置
#define MPU6050_REG_GYRO_CONFIG         0x1BU						 //陀螺仪量程
#define MPU6050_REG_ACCEL_CONFIG        0x1CU						 //加速度计量程
#define MPU6050_REG_FIFO_EN             0x23U
#define MPU6050_REG_INT_ENABLE          0x38U
#define MPU6050_REG_ACCEL_XOUT_H        0x3BU
#define MPU6050_REG_USER_CTRL           0x6AU
#define MPU6050_REG_PWR_MGMT_1          0x6BU
#define MPU6050_REG_PWR_MGMT_2          0x6CU
#define MPU6050_REG_WHO_AM_I            0x75U

#define MPU6050_WHO_AM_I_VALUE          0x68U
#define MPU6050_SAMPLE_PERIOD_S         0.005f
#define MPU6050_GYRO_SCALE              65.5f
#define MPU6050_ACCEL_SCALE             16384.0f
#define MPU6050_RAD_TO_DEG              57.2957795f

#define MPU6050_CALIBRATION_SAMPLES     800U
#define MPU6050_CALIBRATION_MAX_READS   2400U
#define MPU6050_CALIBRATION_GYRO_LIMIT  8.0f

#define MPU6050_COMPLEMENTARY_TAU_S     0.35f
#define MPU6050_STILL_GYRO_LIMIT_DPS    1.2f
#define MPU6050_STILL_ACCEL_MIN_G       0.94f
#define MPU6050_STILL_ACCEL_MAX_G       1.06f
#define MPU6050_STILL_CONFIRM_SAMPLES   80U
#define MPU6050_YAW_DEADBAND_DPS        0.12f
#define MPU6050_BIAS_TRACK_GAIN         0.0025f
#define MPU6050_YAW_HOLD_GAIN           0.025f

#define MPU6050_KALMAN_Q_ANGLE          0.0025f
#define MPU6050_KALMAN_Q_BIAS           0.00008f
#define MPU6050_KALMAN_R_ANGLE          0.80f
#define MPU6050_RATE_KALMAN_Q           8.0f
#define MPU6050_RATE_KALMAN_R           3.0f

typedef struct {
    float angle;//当前估计角度
    float bias;//当前估计的陀螺仪偏置
	/*协方差矩阵*/
    float p00;
    float p01;
    float p10;
    float p11;
} AngleKalman_t;//姿态角卡尔曼滤波

typedef struct {
    float value;
    float variance;
} ScalarKalman_t;//一维卡尔曼滤波器

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} RawSample_t;

Imu_t mpu6050;
MPU6050_Status_t mpu6050_status;

static pIICInterface_t s_i2c = &User_sIICDev;
static AngleKalman_t s_roll_kalman;
static AngleKalman_t s_pitch_kalman;
static ScalarKalman_t s_yaw_rate_kalman;
/* 互补滤波的中间角度 */
static float s_complementary_roll;
static float s_complementary_pitch;
//------------------------------
static float s_yaw_hold;//静止时保存的yaw角度，用于防止停车时yaw慢慢漂移
static uint32_t s_last_update_ms;//上一次更新时的系统时间，用于计算dt
static uint16_t s_still_sample_count;//连续静止样本计数（只有连续满足静止条件达到一定次数，才认为车辆真的静止）

static float clampf_local(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float wrap_angle(float angle)//把角度限制在-180-180之间
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float max_abs3(float x, float y, float z)
{
    float result = fabsf(x);
    if (fabsf(y) > result) {
        result = fabsf(y);
    }
    if (fabsf(z) > result) {
        result = fabsf(z);
    }
    return result;
}

static uint8_t write_register(uint8_t reg, uint8_t value)
{
    return (s_i2c->write_reg(
        MPU6050_ADDRESS_8BIT, reg, &value, 1U, 20U) == IIC_OK) ? 1U : 0U;
}

static uint8_t read_registers(uint8_t reg, uint8_t *data, uint16_t length)
{
    return (s_i2c->read_reg(
        MPU6050_ADDRESS_8BIT, reg, data, length, 20U) == IIC_OK) ? 1U : 0U;
}

static int16_t make_i16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static uint8_t read_raw_sample(RawSample_t *sample)
{
    uint8_t data[14];

    if ((sample == NULL) ||
        (read_registers(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data)) == 0U)) {
        return 0U;
    }

    sample->accel_x = make_i16(data[0], data[1]);
    sample->accel_y = make_i16(data[2], data[3]);
    sample->accel_z = make_i16(data[4], data[5]);
    sample->temperature = make_i16(data[6], data[7]);
    sample->gyro_x = make_i16(data[8], data[9]);
    sample->gyro_y = make_i16(data[10], data[11]);
    sample->gyro_z = make_i16(data[12], data[13]);
    return 1U;
}

static float angle_kalman_update(
    AngleKalman_t *filter, float measured_angle, float measured_rate, float dt)
{
    float innovation;
    float innovation_variance;
    float gain0;
    float gain1;
    float p00_temp;
    float p01_temp;

    filter->angle += dt * (measured_rate - filter->bias);//新角度 = 旧角度 + 去零偏后的角速度 × 时间

    filter->p00 += dt *
        (dt * filter->p11 - filter->p01 - filter->p10 +
         MPU6050_KALMAN_Q_ANGLE);
    filter->p01 -= dt * filter->p11;
    filter->p10 -= dt * filter->p11;
    filter->p11 += MPU6050_KALMAN_Q_BIAS * dt;

    innovation = wrap_angle(measured_angle - filter->angle);
    innovation_variance = filter->p00 + MPU6050_KALMAN_R_ANGLE;
    gain0 = filter->p00 / innovation_variance;
    gain1 = filter->p10 / innovation_variance;

    filter->angle += gain0 * innovation;
    filter->bias += gain1 * innovation;

    p00_temp = filter->p00;
    p01_temp = filter->p01;
    filter->p00 -= gain0 * p00_temp;
    filter->p01 -= gain0 * p01_temp;
    filter->p10 -= gain1 * p00_temp;
    filter->p11 -= gain1 * p01_temp;

    return wrap_angle(filter->angle);
}

static float rate_kalman_update(ScalarKalman_t *filter, float measurement, float dt)
{
    float gain;

    filter->variance += MPU6050_RATE_KALMAN_Q * dt;
    gain = filter->variance /
        (filter->variance + MPU6050_RATE_KALMAN_R);
    filter->value += gain * (measurement - filter->value);
    filter->variance *= (1.0f - gain);
    return filter->value;
}

static uint8_t calibrate_gyro(void)//陀螺仪校准
{
    RawSample_t sample;
    double gyro_sum_x = 0.0;
    double gyro_sum_y = 0.0;
    double gyro_sum_z = 0.0;
    uint32_t accepted = 0U;
    uint32_t reads = 0U;

    /*
     * Discard the first 0.5 s after wake-up so the analogue filters and
     * temperature sensor can settle before bias estimation.
     */
    for (reads = 0U; reads < 100U; ++reads) {
        if (read_raw_sample(&sample) == 0U) {
            return 0U;
        }
        delay_ms(5U);
    }

    reads = 0U;
    while ((accepted < MPU6050_CALIBRATION_SAMPLES) &&
           (reads < MPU6050_CALIBRATION_MAX_READS)) {
        float gx;
        float gy;
        float gz;
        float ax;
        float ay;
        float az;
        float accel_magnitude;

        ++reads;
        if (read_raw_sample(&sample) == 0U) {
            delay_ms(5U);
            continue;
        }

        gx = (float)sample.gyro_x / MPU6050_GYRO_SCALE;
        gy = (float)sample.gyro_y / MPU6050_GYRO_SCALE;
        gz = (float)sample.gyro_z / MPU6050_GYRO_SCALE;
        ax = (float)sample.accel_x / MPU6050_ACCEL_SCALE;
        ay = (float)sample.accel_y / MPU6050_ACCEL_SCALE;
        az = (float)sample.accel_z / MPU6050_ACCEL_SCALE;
        accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);

        if ((max_abs3(gx, gy, gz) < MPU6050_CALIBRATION_GYRO_LIMIT) &&
            (accel_magnitude > 0.90f) && (accel_magnitude < 1.10f)) {
            gyro_sum_x += gx;
            gyro_sum_y += gy;
            gyro_sum_z += gz;
            ++accepted;
        }
        delay_ms(5U);
    }

    if (accepted < MPU6050_CALIBRATION_SAMPLES) {
        return 0U;
    }

    mpu6050_status.gyro_bias.x = (float)(gyro_sum_x / (double)accepted);
    mpu6050_status.gyro_bias.y = (float)(gyro_sum_y / (double)accepted);
    mpu6050_status.gyro_bias.z = (float)(gyro_sum_z / (double)accepted);
    return 1U;
}

uint8_t MPU6050_testConnection(void)
{
    uint8_t identity = 0U;

    if (read_registers(MPU6050_REG_WHO_AM_I, &identity, 1U) == 0U) {
        return 0U;
    }
    return (identity == MPU6050_WHO_AM_I_VALUE) ? 1U : 0U;
}

uint8_t MPU6050_initialize(void)
{
    RawSample_t sample;
    float ax;
    float ay;
    float az;
    float initial_roll;
    float initial_pitch;

    mpu6050 = (Imu_t){0};
    mpu6050_status = (MPU6050_Status_t){0};
    s_roll_kalman = (AngleKalman_t){0};
    s_pitch_kalman = (AngleKalman_t){0};
    s_yaw_rate_kalman = (ScalarKalman_t){0};
    s_yaw_rate_kalman.variance = 1.0f;
    s_last_update_ms = 0U;
    s_still_sample_count = 0U;
    if ((s_i2c == NULL) || (s_i2c->init == NULL)) {
        return 0U;
    }
    s_i2c->init();

    if (MPU6050_testConnection() == 0U) {
        return 0U;
    }

    if (write_register(MPU6050_REG_PWR_MGMT_1, 0x80U) == 0U) {
        return 0U;
    }
    delay_ms(100U);

    /*
     * 200 Hz raw sampling:
     * - PLL clock, all axes enabled
     * - DLPF 42/44 Hz
     * - gyro +/-500 deg/s (65.5 LSB/(deg/s))
     * - accelerometer +/-2 g
     * - FIFO and DMP disabled
     */
    if ((write_register(MPU6050_REG_PWR_MGMT_1, 0x01U) == 0U) ||
        (write_register(MPU6050_REG_PWR_MGMT_2, 0x00U) == 0U) ||
        (write_register(MPU6050_REG_SMPLRT_DIV, 0x04U) == 0U) ||
        (write_register(MPU6050_REG_CONFIG, 0x03U) == 0U) ||
        (write_register(MPU6050_REG_GYRO_CONFIG, 0x08U) == 0U) ||
        (write_register(MPU6050_REG_ACCEL_CONFIG, 0x00U) == 0U) ||
        (write_register(MPU6050_REG_FIFO_EN, 0x00U) == 0U) ||
        (write_register(MPU6050_REG_USER_CTRL, 0x00U) == 0U) ||
        (write_register(MPU6050_REG_INT_ENABLE, 0x00U) == 0U)) {
        return 0U;
    }
    delay_ms(50U);

    if (calibrate_gyro() == 0U) {
        return 0U;
    }

    if (read_raw_sample(&sample) == 0U) {
        return 0U;
    }

    ax = (float)sample.accel_x / MPU6050_ACCEL_SCALE;
    ay = (float)sample.accel_y / MPU6050_ACCEL_SCALE;
    az = (float)sample.accel_z / MPU6050_ACCEL_SCALE;

    /*
     * Vehicle +Y is forward, +X is right and +Z is up.
     * Roll is about the forward Y axis and pitch is about X.
     */
    initial_roll = atan2f(ax, az) * MPU6050_RAD_TO_DEG;
    initial_pitch = atan2f(-ay, sqrtf(ax * ax + az * az)) *
        MPU6050_RAD_TO_DEG;
    s_complementary_roll = initial_roll;
    s_complementary_pitch = initial_pitch;
    s_roll_kalman.angle = initial_roll;
    s_pitch_kalman.angle = initial_pitch;
    mpu6050.roll = initial_roll;
    mpu6050.pitch = initial_pitch;
    mpu6050.yaw = 0.0f;
    s_yaw_hold = 0.0f;
    mpu6050_status.calibrated = 1U;
    mpu6050_status.stationary = 1U;

    /* TASK2 polls at 200 Hz, so the optional INT pin is not required. */
    s_last_update_ms = Board_GetMillis();
    return 1U;
}

uint8_t MPU6050_Update(uint8_t vehicle_stopped)
{
    RawSample_t sample;
    uint32_t now_ms;
    uint32_t elapsed_ms;
    float dt;
    float ax;
    float ay;
    float az;
    float gx_raw;
    float gy_raw;
    float gz_raw;
    float gx;
    float gy;
    float gz;
    float accel_magnitude;
    float accel_roll;
    float accel_pitch;
    float complementary_alpha;
    float yaw_rate;
    uint8_t sensor_still;

    if (mpu6050_status.calibrated == 0U) {
        return 0U;
    }

    now_ms = Board_GetMillis();
    elapsed_ms = now_ms - s_last_update_ms;
    if (elapsed_ms < 4U) {
        return 1U;
    }

    if (read_raw_sample(&sample) == 0U) {
        ++mpu6050_status.read_error_count;
        return 0U;
    }

    s_last_update_ms = now_ms;
    if ((elapsed_ms == 0U) || (elapsed_ms > 25U)) {
        dt = MPU6050_SAMPLE_PERIOD_S;
    } else {
        dt = (float)elapsed_ms * 0.001f;
    }
    dt = clampf_local(dt, 0.002f, 0.025f);

    ax = (float)sample.accel_x / MPU6050_ACCEL_SCALE;
    ay = (float)sample.accel_y / MPU6050_ACCEL_SCALE;
    az = (float)sample.accel_z / MPU6050_ACCEL_SCALE;
    gx_raw = (float)sample.gyro_x / MPU6050_GYRO_SCALE;
    gy_raw = (float)sample.gyro_y / MPU6050_GYRO_SCALE;
    gz_raw = (float)sample.gyro_z / MPU6050_GYRO_SCALE;

    gx = gx_raw - mpu6050_status.gyro_bias.x;
    gy = gy_raw - mpu6050_status.gyro_bias.y;
    gz = (gz_raw - mpu6050_status.gyro_bias.z) *
        MPU6050_YAW_GYRO_SIGN;

    accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);
    sensor_still =
        ((vehicle_stopped != 0U) &&
         (accel_magnitude > MPU6050_STILL_ACCEL_MIN_G) &&
         (accel_magnitude < MPU6050_STILL_ACCEL_MAX_G) &&
         (max_abs3(gx, gy, gz) < MPU6050_STILL_GYRO_LIMIT_DPS)) ? 1U : 0U;

    if (sensor_still != 0U) {
        if (s_still_sample_count < MPU6050_STILL_CONFIRM_SAMPLES) {
            ++s_still_sample_count;
        }
    } else {
        s_still_sample_count = 0U;
        mpu6050_status.stationary = 0U;
    }

    if ((s_still_sample_count >= MPU6050_STILL_CONFIRM_SAMPLES) &&
        (mpu6050_status.stationary == 0U)) {
        mpu6050_status.stationary = 1U;
        s_yaw_hold = mpu6050.yaw;
    }

    if (mpu6050_status.stationary != 0U) {
        /*
         * Zero-angular-rate update: slowly learn temperature-dependent bias
         * only while both the command state and IMU agree that the car is
         * stopped. This avoids learning a real slow turn as bias.
         */
        mpu6050_status.gyro_bias.x +=
            MPU6050_BIAS_TRACK_GAIN *
            (gx_raw - mpu6050_status.gyro_bias.x);
        mpu6050_status.gyro_bias.y +=
            MPU6050_BIAS_TRACK_GAIN *
            (gy_raw - mpu6050_status.gyro_bias.y);
        mpu6050_status.gyro_bias.z +=
            MPU6050_BIAS_TRACK_GAIN *
            (gz_raw - mpu6050_status.gyro_bias.z);

        gx = gx_raw - mpu6050_status.gyro_bias.x;
        gy = gy_raw - mpu6050_status.gyro_bias.y;
        gz = (gz_raw - mpu6050_status.gyro_bias.z) *
            MPU6050_YAW_GYRO_SIGN;
    }

    accel_roll = atan2f(ax, az) * MPU6050_RAD_TO_DEG;
    accel_pitch = atan2f(-ay, sqrtf(ax * ax + az * az)) *
        MPU6050_RAD_TO_DEG;
    complementary_alpha =
        MPU6050_COMPLEMENTARY_TAU_S /
        (MPU6050_COMPLEMENTARY_TAU_S + dt);

    s_complementary_roll = wrap_angle(
        complementary_alpha * wrap_angle(s_complementary_roll + gy * dt) +
        (1.0f - complementary_alpha) * accel_roll);
    s_complementary_pitch = wrap_angle(
        complementary_alpha * wrap_angle(s_complementary_pitch + gx * dt) +
        (1.0f - complementary_alpha) * accel_pitch);

    /*
     * The complementary result rejects vehicle vibration before it becomes
     * the angle measurement for the 2-state Kalman filters. The Kalman bias
     * states then remove residual X/Y gyro drift.
     */
    mpu6050.roll = angle_kalman_update(
        &s_roll_kalman, s_complementary_roll, gy, dt);
    mpu6050.pitch = angle_kalman_update(
        &s_pitch_kalman, s_complementary_pitch, gx, dt);

    yaw_rate = rate_kalman_update(&s_yaw_rate_kalman, gz, dt);
    if ((mpu6050_status.stationary != 0U) &&
        (fabsf(yaw_rate) < MPU6050_YAW_DEADBAND_DPS)) {
        yaw_rate = 0.0f;
        s_yaw_rate_kalman.value = 0.0f;
    }

    mpu6050.yaw = wrap_angle(mpu6050.yaw + yaw_rate * dt);
    if (mpu6050_status.stationary != 0U) {
        /*
         * Complement the integrated yaw with the heading captured on entry
         * to the stopped state. This removes residual standstill wander but
         * never pulls a valid turned heading back to power-on zero.
         */
        mpu6050.yaw = wrap_angle(
            mpu6050.yaw +
            MPU6050_YAW_HOLD_GAIN *
            wrap_angle(s_yaw_hold - mpu6050.yaw));
    }

    mpu6050.gyro.x = gx;
    mpu6050.gyro.y = gy;
    mpu6050.gyro.z = gz;
    mpu6050.accel.x = ax;
    mpu6050.accel.y = ay;
    mpu6050.accel.z = az;
    mpu6050_status.yaw_rate = yaw_rate;
    mpu6050_status.temperature_c =
        ((float)sample.temperature / 340.0f) + 36.53f;
    ++mpu6050_status.sample_count;
    return 1U;
}

void MPU6050_ResetYaw(float yaw_deg)
{
    mpu6050.yaw = wrap_angle(yaw_deg);
    s_yaw_hold = mpu6050.yaw;
    s_yaw_rate_kalman.value = 0.0f;
}

int Read_Temperature(void)
{
    return (int)(mpu6050_status.temperature_c * 10.0f);
}
