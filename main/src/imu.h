#ifndef __IMU_H__
#define __IMU_H__

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include "esp_err.h"

// I2C 配置宏
#define IMU_I2C_ADDR                0x69      // IMU I2C 地址 (AD0=1)
#define I2C_MASTER_FREQ_HZ          1000000    // I2C Master 时钟频率 (400kHz, 1MHz可能不稳定)
#define I2C_MASTER_SCL_IO           GPIO_NUM_19 // 这里自己改一下
#define I2C_MASTER_SDA_IO           GPIO_NUM_18 // 这里自己改一下
#define I2C_MASTER_NUM              I2C_NUM_1 // 这里自己改一下

typedef void *imu_handle_t;

// 外部 I2C 句柄声明
extern i2c_master_dev_handle_t dev_handle;

// IMU 寄存器地址映射 (仅列出常用的 Bank 0 寄存器)
typedef enum {
    IMU_TEMP_DATA1      = 0x09,
    IMU_TEMP_DATA0      = 0x0A,
    IMU_ACCEL_DATA_X1   = 0x0B,
    IMU_ACCEL_DATA_Y1   = 0x0D,
    IMU_ACCEL_DATA_Z1   = 0x0F,
    IMU_GYRO_DATA_X1    = 0x11,
    IMU_GYRO_DATA_Y1    = 0x13,
    IMU_GYRO_DATA_Z1    = 0x15,
    IMU_PWR_MGMT0       = 0x1F,
    IMU_GYRO_CONFIG0    = 0x20,
    IMU_ACCEL_CONFIG0   = 0x21,
    IMU_GYRO_CONFIG1    = 0x23,
    IMU_ACCEL_CONFIG1   = 0x24,
    IMU_WHO_AM_I        = 0x75,
} imu_reg_bank0_map_t;

// 原始加速度数据结构 (int16_t)
typedef struct {
    int16_t acce_raw_x;
    int16_t acce_raw_y;
    int16_t acce_raw_z;
} imu_acce_raw_data_t;

// 原始陀螺仪数据结构 (int16_t)
typedef struct {
    int16_t gyro_raw_x;
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;
} imu_gyro_raw_data_t;

// 转换后的加速度数据结构 (float, 单位 g)
typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
} imu_acce_data_t;

// 转换后的陀螺仪数据结构 (float, 单位 dps)
typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_gyro_data_t;

// 互补滤波角度结构 (float, 单位 度)
typedef struct {
    float pitch_angle;
    float roll_angle;
    float yaw_angle; // 互补滤波无法准确计算, 但保留结构体字段
} imu_comp_angle_t;

// 加速度计满量程选择
typedef enum {
    ACCEL_UI_FS_SEL_16G = 0x00,
    ACCEL_UI_FS_SEL_8G  = 0x01,
    ACCEL_UI_FS_SEL_4G  = 0x02,
    ACCEL_UI_FS_SEL_2G  = 0x03,
} ACCEL_UI_FS_SEL;

// 陀螺仪满量程选择
typedef enum {
    GYRO_UI_FS_SEL_2000DPS = 0x00,
    GYRO_UI_FS_SEL_1000DPS = 0x01,
    GYRO_UI_FS_SEL_500DPS  = 0x02,
    GYRO_UI_FS_SEL_250DPS  = 0x03,
} GYRO_UI_FS_SEL;

// 加速度计输出数据率 (ODR)
typedef enum {
    ACCEL_ODR_1600HZ    = 0x05,
    ACCEL_ODR_800HZ     = 0x06,
    ACCEL_ODR_400HZ     = 0x07,
    ACCEL_ODR_200HZ     = 0x08,
    ACCEL_ODR_100HZ     = 0x09,
    ACCEL_ODR_50HZ      = 0x0A,
    ACCEL_ODR_25HZ      = 0x0B,
    ACCEL_ODR_12_5HZ    = 0x0C,
    ACCEL_ODR_6_25HZ    = 0x0D,
    ACCEL_ODR_3_125HZ   = 0x0E,
    ACCEL_ODR_1_5625HZ  = 0x0F,
} ACCEL_ODR;

// 陀螺仪输出数据率 (ODR)
typedef enum {
    GYRO_ODR_1600HZ = 0x05,
    GYRO_ODR_800HZ  = 0x06,
    GYRO_ODR_400HZ  = 0x07,
    GYRO_ODR_200HZ  = 0x08,
    GYRO_ODR_100HZ  = 0x09,
    GYRO_ODR_50HZ   = 0x0A,
    GYRO_ODR_25HZ   = 0x0B,
    GYRO_ODR_12_5HZ = 0x0C,
} GYRO_ODR;


// ===================================
// 函数原型
// ===================================

/**
 * @brief 初始化 I2C 总线和 IMU 设备
 * @return ESP_OK: 成功, 其他: 失败
 */
esp_err_t imu_i2c_init(void);

/**
 * @brief 清理 I2C 资源
 */
void imu_i2c_deinit(void);

/**
 * @brief 读取 IMU 寄存器
 */
esp_err_t imu_reg_read(imu_handle_t imu_handle, uint8_t reg_addr, uint8_t *data, const uint8_t len);

/**
 * @brief 写入 IMU 寄存器
 */
esp_err_t imu_reg_write(imu_handle_t imu_handle, uint8_t reg_addr, uint8_t const *data, const uint8_t len);

/**
 * @brief 检查设备 ID 并唤醒设备
 */
esp_err_t imu_self_test(void);

/**
 * @brief 打印关键寄存器值进行调试
 */
esp_err_t imu_debug_registers(void);

/**
 * @brief 打印原始传感器数据进行调试
 */
esp_err_t imu_debug_raw_data(void);

/**
 * @brief 唤醒 IMU 设备 (使能加速度计和陀螺仪)
 */
esp_err_t imu_wake_up(void);

/**
 * @brief 使 IMU 进入休眠模式 (关闭加速度计和陀螺仪)
 */
esp_err_t imu_sleep(void);

/**
 * @brief 获取加速度计原始数据
 */
esp_err_t imu_get_raw_acce_data(imu_acce_raw_data_t *raw_acce_val);

/**
 * @brief 获取陀螺仪原始数据
 */
esp_err_t imu_get_raw_gyro_data(imu_gyro_raw_data_t *raw_gyro_val);

/**
 * @brief 获取加速度计灵敏度（每g的LSB数）
 */
esp_err_t imu_get_acce_sensitivity(float *const acce_sensitivity);

/**
 * @brief 获取陀螺仪灵敏度（每dps的LSB数）
 */
esp_err_t imu_get_gyro_sensitivity(float *const gyro_sensitivity);

/**
 * @brief 获取转换后的加速度数据 (g)
 */
esp_err_t imu_get_acce_data(imu_acce_data_t *acce_val);

/**
 * @brief 获取转换后的陀螺仪数据 (dps)
 */
esp_err_t imu_get_gyro_data(imu_gyro_data_t *gyro_val);

/**
 * @brief 获取转换后的温度数据 (摄氏度)
 */
esp_err_t imu_get_temp_data(float *const temp);

/**
 * @brief 批量读取加速度、陀螺仪和温度原始数据，并转换为物理值 (推荐)
 */
esp_err_t imu_get_all_data(imu_acce_data_t *acce_val, imu_gyro_data_t *gyro_val, float *temp);

/**
 * @brief 使用互补滤波融合加速度和陀螺仪数据，计算角度
 */
esp_err_t imu_comp_filter(imu_acce_data_t *const acce_val, imu_gyro_data_t *const gyro_val, imu_comp_angle_t *const comp_angle);

/**
 * @brief 设置加速度计采样率
 */
esp_err_t imu_acce_set_sampling_rate(double rate_hz);

/**
 * @brief 设置陀螺仪采样率
 */
esp_err_t imu_gyro_set_sampling_rate(double rate_hz);

#endif // __IMU_H__