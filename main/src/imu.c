#include "imu.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_timer.h" // 使用标准的 esp_timer_get_time() 替代 rtc.h
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define _PI 3.1415926535f
#define _RAD2DEG 57.2957795130f
#define _ALPHA 0.98f // 互补滤波系数
static const char *TAG = "IMU";

i2c_master_dev_handle_t dev_handle = NULL;
i2c_master_bus_handle_t mst_bus_handle = NULL;
float acce_sensitivity = 0, gyro_sensitivity = 0;
float acce_multiplier = 0, gyro_multiplier = 0;

esp_err_t imu_reg_write(imu_handle_t imu_handle, uint8_t reg_addr, uint8_t const *data, const uint8_t len) {
    if (len == 0 || dev_handle == NULL) return ESP_ERR_INVALID_ARG;
    
    uint8_t *write_buf = (uint8_t *)malloc(len + 1);
    if (write_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, data, len);
    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, len + 1, 1000 / portTICK_PERIOD_MS);
    free(write_buf);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Reg Write Fail: Addr=0x%02X, Error=0x%x", reg_addr, ret);
    }
    return ret;
}

esp_err_t imu_reg_read(imu_handle_t imu_handle, uint8_t reg_addr, uint8_t *data, const uint8_t len) {
    if (len == 0 || dev_handle == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, 
                                              &reg_addr, 1, 
                                              data, len, 
                                              1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Reg Read Fail: Addr=0x%02X, Error=0x%x", reg_addr, ret);
    }
    return ret;
}


/**
 * @brief 检查设备 ID 并打印关键寄存器状态
 */
esp_err_t imu_self_test(void){
    ESP_LOGI(TAG, "Starting IMU Self-Test...");
    
    uint8_t who_am_i;
    esp_err_t ret = imu_reg_read(NULL, IMU_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read WHO_AM_I failed: 0x%x", ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "Device ID: 0x%02X", who_am_i);

    if (who_am_i != 0x67 && who_am_i != 0xEA) { 
        ESP_LOGE(TAG, "Device ID mismatch, Expected: 0x67 or 0xEA, Actual: 0x%02X", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Device ID verified.");
    
    ret = imu_wake_up();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device wake up failed");
        return ret;
    }
    ESP_LOGI(TAG, "Device woke up successfully.");
    
    vTaskDelay(pdMS_TO_TICKS(100)); 
    
    return ESP_OK;
}

/**
 * @brief 增强的初始化函数
 */
esp_err_t imu_i2c_init(void) {
    ESP_LOGI(TAG, "Starting IMU Initialization...");
    
    i2c_master_bus_config_t master_conf = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 1,
        .flags.enable_internal_pullup = 0, 
    };
    
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_new_master_bus(&master_conf, &mst_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: 0x%x", ret);
        return ret;
    }
    
    ret = i2c_master_bus_add_device(mst_bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: 0x%x", ret);
        i2c_del_master_bus(mst_bus_handle);
        mst_bus_handle = NULL;
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C initialization successful");
    vTaskDelay(pdMS_TO_TICKS(100)); 
    
    ret = imu_self_test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU Self-Test failed. De-initializing I2C.");
        imu_i2c_deinit(); 
        return ret;
    }
    
    ESP_LOGI(TAG, "Configuring sensor parameters...");
    
    uint8_t acce_config = (ACCEL_UI_FS_SEL_8G << 5); 
    ret = imu_reg_write(NULL, IMU_ACCEL_CONFIG0, &acce_config, 1);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Accel config failed: 0x%x", ret);
    
    uint8_t gyro_config = (GYRO_UI_FS_SEL_1000DPS << 5); 
    ret = imu_reg_write(NULL, IMU_GYRO_CONFIG0, &gyro_config, 1);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Gyro config failed: 0x%x", ret);
    
    imu_acce_set_sampling_rate(100.0);
    imu_gyro_set_sampling_rate(100.0);
    
    ESP_LOGI(TAG, "IMU initialization complete");
    return ESP_OK;
}

/**
 * @brief 清理 I2C 资源
 */
void imu_i2c_deinit(void){
    if (dev_handle) {
        i2c_master_bus_rm_device(dev_handle);
        dev_handle = NULL;
    }
    if (mst_bus_handle) {
        i2c_del_master_bus(mst_bus_handle);
        mst_bus_handle = NULL;
    }
    ESP_LOGI(TAG, "I2C deinitialized");
}

/**
 * @brief 唤醒 IMU 设备 (使能加速度计和陀螺仪)
 */
esp_err_t imu_wake_up(void) {
    ESP_LOGI(TAG, "Waking up IMU device...");
    
    uint8_t mode = 0x0F; 
    esp_err_t ret = imu_reg_write(NULL, IMU_PWR_MGMT0, &mode, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Wake up failed: 0x%x", ret);
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); 
    return ret;
}

/**
 * @brief 使 IMU 进入休眠模式 (关闭加速度计和陀螺仪)
 */
esp_err_t imu_sleep(void){
    uint8_t mode = 0x01; 
    esp_err_t ret = imu_reg_write(NULL, IMU_PWR_MGMT0, &mode, 1);
    return ret;
}


esp_err_t imu_debug_registers(void) {
    uint8_t reg_data;
    esp_err_t ret;
    
    ESP_LOGI(TAG, "=== Register Debug Info ===");
    
    uint8_t debug_regs[] = {
        IMU_WHO_AM_I, IMU_PWR_MGMT0, IMU_ACCEL_CONFIG0, 
        IMU_GYRO_CONFIG0, IMU_ACCEL_CONFIG1, IMU_GYRO_CONFIG1
    };
    const char* reg_names[] = {
        "WHO_AM_I", "PWR_MGMT0", "ACCEL_CONFIG0", 
        "GYRO_CONFIG0", "ACCEL_CONFIG1", "GYRO_CONFIG1"
    };
    
    for (int i = 0; i < sizeof(debug_regs)/sizeof(debug_regs[0]); i++) {
        ret = imu_reg_read(NULL, debug_regs[i], &reg_data, 1);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "%-15s: 0x%02X", reg_names[i], reg_data);
        } else {
            ESP_LOGE(TAG, "Read %s failed: 0x%x", reg_names[i], ret);
        }
    }
    
    return ESP_OK;
}

esp_err_t imu_debug_raw_data(void) {
    uint8_t raw_data[6];
    esp_err_t ret;
    
    ESP_LOGI(TAG, "=== Raw Data Debug ===");
    
    ret = imu_reg_read(NULL, IMU_ACCEL_DATA_X1, raw_data, 6);
    if (ret == ESP_OK) {
        int16_t raw_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        int16_t raw_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        int16_t raw_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
        ESP_LOGI(TAG, "Accel Raw: X=%d, Y=%d, Z=%d", raw_x, raw_y, raw_z);
    } else {
        ESP_LOGE(TAG, "Read accel raw data failed");
    }
    ret = imu_reg_read(NULL, IMU_GYRO_DATA_X1, raw_data, 6);
    if (ret == ESP_OK) {
        int16_t raw_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        int16_t raw_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        int16_t raw_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
        ESP_LOGI(TAG, "Gyro Raw: X=%d, Y=%d, Z=%d", raw_x, raw_y, raw_z);
    } else {
        ESP_LOGE(TAG, "Read gyro raw data failed");
    }
    uint8_t temp_data[2];
    ret = imu_reg_read(NULL, IMU_TEMP_DATA1, temp_data, 2);
    if (ret == ESP_OK) {
        int16_t temp_raw = (int16_t)((temp_data[0] << 8) | temp_data[1]);
        ESP_LOGI(TAG, "Temp Raw: %d", temp_raw);
    } else {
        ESP_LOGE(TAG, "Read temp raw data failed");
    }
    
    return ESP_OK;
}

esp_err_t imu_get_acce_sensitivity(float *const acce_sensitivity) {
    uint8_t data;
    esp_err_t ret = imu_reg_read(NULL, IMU_ACCEL_CONFIG0, &data, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t fs_sel = (data >> 5) & 0x03;
    switch(fs_sel) {
        case ACCEL_UI_FS_SEL_2G:
            *acce_sensitivity = 16384.0f;   // LSB/g
            break;
        case ACCEL_UI_FS_SEL_4G:
            *acce_sensitivity = 8192.0f;    // LSB/g
            break;
        case ACCEL_UI_FS_SEL_8G:
            *acce_sensitivity = 4096.0f;    // LSB/g
            break;
        case ACCEL_UI_FS_SEL_16G:
            *acce_sensitivity = 2048.0f;    // LSB/g
            break;
        default:
            *acce_sensitivity = 4096.0f;    // 默认值 (8G)
            break;
    }
    return ESP_OK;
}

esp_err_t imu_get_gyro_sensitivity(float *const gyro_sensitivity) {
    uint8_t data;
    esp_err_t ret = imu_reg_read(NULL, IMU_GYRO_CONFIG0, &data, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t fs_sel = (data >> 5) & 0x03;
    switch(fs_sel) {
        case GYRO_UI_FS_SEL_250DPS:
            *gyro_sensitivity = 131.0f;     // LSB/dps
            break;
        case GYRO_UI_FS_SEL_500DPS:
            *gyro_sensitivity = 65.5f;      // LSB/dps
            break;
        case GYRO_UI_FS_SEL_1000DPS:
            *gyro_sensitivity = 32.8f;      // LSB/dps
            break;
        case GYRO_UI_FS_SEL_2000DPS:
            *gyro_sensitivity = 16.4f;      // LSB/dps
            break;
        default:
            *gyro_sensitivity = 32.8f;      // 默认值 (1000dps)
            break;
    }
    return ESP_OK;
}

esp_err_t imu_get_raw_acce_data(imu_acce_raw_data_t *raw_acce_val){
    uint8_t acce_data[6];
    esp_err_t ret = imu_reg_read(NULL, IMU_ACCEL_DATA_X1, acce_data, sizeof(acce_data));
    if (ret != ESP_OK) return ret;
    
    raw_acce_val->acce_raw_x = (int16_t)((acce_data[0] << 8) | acce_data[1]);
    raw_acce_val->acce_raw_y = (int16_t)((acce_data[2] << 8) | acce_data[3]);
    raw_acce_val->acce_raw_z = (int16_t)((acce_data[4] << 8) | acce_data[5]);
    return ret;
}

esp_err_t imu_get_raw_gyro_data(imu_gyro_raw_data_t *raw_gyro_val){
    uint8_t gyro_data[6];
    esp_err_t ret = imu_reg_read(NULL, IMU_GYRO_DATA_X1, gyro_data, sizeof(gyro_data));
    if (ret != ESP_OK) return ret;
    
    raw_gyro_val->gyro_raw_x = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
    raw_gyro_val->gyro_raw_y = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
    raw_gyro_val->gyro_raw_z = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);
    return ret;
}

esp_err_t imu_get_acce_data(imu_acce_data_t *acce_val){
    imu_acce_raw_data_t raw_acce_val;
    float acce_sensitivity;
    esp_err_t ret = imu_get_acce_sensitivity(&acce_sensitivity);
    if(ret != ESP_OK) return ret;

    ret = imu_get_raw_acce_data(&raw_acce_val);
    if(ret != ESP_OK) return ret;
    
    float divisor = 32768.0f / acce_sensitivity;
    acce_val->acce_x = raw_acce_val.acce_raw_x / divisor;
    acce_val->acce_y = raw_acce_val.acce_raw_y / divisor;
    acce_val->acce_z = raw_acce_val.acce_raw_z / divisor;
    return ret;
}

esp_err_t imu_get_gyro_data(imu_gyro_data_t *gyro_val){
    imu_gyro_raw_data_t raw_gyro_val;
    float gyro_sensitivity;
    esp_err_t ret = imu_get_gyro_sensitivity(&gyro_sensitivity);
    if(ret != ESP_OK) return ret;

    ret = imu_get_raw_gyro_data(&raw_gyro_val);
    if(ret != ESP_OK) return ret;
    
    float divisor = 32768.0f / gyro_sensitivity;
    gyro_val->gyro_x = raw_gyro_val.gyro_raw_x / divisor;
    gyro_val->gyro_y = raw_gyro_val.gyro_raw_y / divisor;
    gyro_val->gyro_z = raw_gyro_val.gyro_raw_z / divisor;
    return ret;
}

esp_err_t imu_get_temp_data(float *const temp){
    uint8_t temp_data[2];
    esp_err_t ret = imu_reg_read(NULL, IMU_TEMP_DATA1, temp_data, sizeof(temp_data));
    if(ret != ESP_OK) return ret;
    
    int16_t temp_raw = (int16_t)((temp_data[0] << 8) | temp_data[1]);
    // Temperature in Degrees Centigrade = (TEMP_DATA / 128) + 25
    *temp = temp_raw/128.0f + 25.0f;
    return ret;
}


/**
 * @brief 批量读取所有传感器数据并转换
 */
esp_err_t imu_get_all_data(imu_acce_data_t *acce_val, imu_gyro_data_t *gyro_val, float *temp) {
    // 读取加速度 (6字节) + 温度 (2字节) + 陀螺仪 (6字节)
    uint8_t sensor_data[14]; 
    
    esp_err_t ret = imu_reg_read(NULL, IMU_ACCEL_DATA_X1, sensor_data, sizeof(sensor_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Batch read sensor data failed: 0x%x", ret);
        return ret;
    }
    int16_t accel_raw[3];
    accel_raw[0] = (int16_t)((sensor_data[0] << 8) | sensor_data[1]); 
    accel_raw[1] = (int16_t)((sensor_data[2] << 8) | sensor_data[3]); 
    accel_raw[2] = (int16_t)((sensor_data[4] << 8) | sensor_data[5]); 
    
    int16_t gyro_raw[3];
    gyro_raw[0] = (int16_t)((sensor_data[6] << 8) | sensor_data[7]);  
    gyro_raw[1] = (int16_t)((sensor_data[8] << 8) | sensor_data[9]);   
    gyro_raw[2] = (int16_t)((sensor_data[10] << 8) | sensor_data[11]); 
    
    int16_t temp_raw = (int16_t)((sensor_data[12] << 8) | sensor_data[13]); 

    // 转换为物理量
    if (acce_sensitivity == 0) {
        imu_get_acce_sensitivity(&acce_sensitivity);
        imu_get_gyro_sensitivity(&gyro_sensitivity);
        acce_multiplier = acce_sensitivity / 32768.0f;
        gyro_multiplier = gyro_sensitivity / 32768.0f;
    }
    
    acce_val->acce_x = (float)accel_raw[0] * acce_multiplier;
    acce_val->acce_y = (float)accel_raw[1] * acce_multiplier;
    acce_val->acce_z = (float)accel_raw[2] * acce_multiplier;
    
    gyro_val->gyro_x = (float)gyro_raw[0] * gyro_multiplier;
    gyro_val->gyro_y = (float)gyro_raw[1] * gyro_multiplier;
    gyro_val->gyro_z = (float)gyro_raw[2] * gyro_multiplier;
    
    *temp = (float)temp_raw / 128.0f + 25.0f;
    
    return ESP_OK;
}


/**
 * @brief 互补滤波融合加速度和陀螺仪数据，计算角度
 */
esp_err_t imu_comp_filter(imu_acce_data_t *const acce_val, imu_gyro_data_t *const gyro_val, imu_comp_angle_t *const comp_angle) {
    static float comp_angle_prev[2] = {0.0f, 0.0f}; 
    static uint64_t last_time_us = 0;
    
    float acce_angle[2] = {0.0f, 0.0f};
    float gyro_angle[2] = {0.0f, 0.0f};
    
    acce_angle[1] = atan2f(acce_val->acce_y, acce_val->acce_z) * _RAD2DEG;
    acce_angle[0] = atan2f(-acce_val->acce_x, sqrtf(acce_val->acce_y * acce_val->acce_y + acce_val->acce_z * acce_val->acce_z)) * _RAD2DEG;
    uint64_t current_time_us = esp_timer_get_time();
    
    if (last_time_us == 0) {
        last_time_us = current_time_us;
        comp_angle->roll_angle = acce_angle[0];
        comp_angle->pitch_angle = acce_angle[1];
        comp_angle_prev[0] = acce_angle[0]; 
        comp_angle_prev[1] = acce_angle[1];
        return ESP_OK;
    }
    
    float dt = (float)(current_time_us - last_time_us) / 1000000.0f;
    last_time_us = current_time_us;
    
    if (dt <= 0 || dt > 0.1f) { 
        dt = 0.01f; 
    }
    
    gyro_angle[0] = comp_angle_prev[0] + gyro_val->gyro_y * dt; 
    gyro_angle[1] = comp_angle_prev[1] + gyro_val->gyro_x * dt; 
    
    comp_angle->roll_angle = _ALPHA * gyro_angle[0] + (1.0f - _ALPHA) * acce_angle[0];
    comp_angle->pitch_angle = _ALPHA * gyro_angle[1] + (1.0f - _ALPHA) * acce_angle[1];
    
    comp_angle_prev[0] = comp_angle->roll_angle;
    comp_angle_prev[1] = comp_angle->pitch_angle;
    
    return ESP_OK;
}


/**
 * @brief 设置加速度计采样率
 */
esp_err_t imu_acce_set_sampling_rate(double rate_hz){
    esp_err_t ret;
    uint8_t current_config;
    
    ret = imu_reg_read(NULL, IMU_ACCEL_CONFIG1, &current_config, 1);
    if(ESP_OK != ret) return ret;
    int rate_int_10000 = (int)(rate_hz * 10000.0);
    uint8_t odr_val;
    
    switch(rate_int_10000){
        case 16000000: odr_val = ACCEL_ODR_1600HZ; break;
        case 8000000: odr_val = ACCEL_ODR_800HZ; break;
        case 4000000: odr_val = ACCEL_ODR_400HZ; break;
        case 2000000: odr_val = ACCEL_ODR_200HZ; break;
        case 1000000: odr_val = ACCEL_ODR_100HZ; break;
        case 500000: odr_val = ACCEL_ODR_50HZ; break;
        case 250000: odr_val = ACCEL_ODR_25HZ; break;
        case 125000: odr_val = ACCEL_ODR_12_5HZ; break;
        case 62500: odr_val = ACCEL_ODR_6_25HZ; break;
        case 31250: odr_val = ACCEL_ODR_3_125HZ; break;
        case 15625: odr_val = ACCEL_ODR_1_5625HZ; break;
        default: 
            ESP_LOGW(TAG, "Accel ODR %f Hz not supported. Using 100Hz default.", rate_hz);
            odr_val = ACCEL_ODR_100HZ;
            break;
    }
    uint8_t new_config = (current_config & 0xF0) | odr_val;
    ret = imu_reg_write(NULL, IMU_ACCEL_CONFIG1, &new_config, 1);
    return ret;
}

/**
 * @brief 设置陀螺仪采样率
 */
esp_err_t imu_gyro_set_sampling_rate(double rate_hz){
    esp_err_t ret;
    uint8_t current_config;
    
    ret = imu_reg_read(NULL, IMU_GYRO_CONFIG1, &current_config, 1);
    if(ESP_OK != ret) return ret;
    
    int rate_int_10000 = (int)(rate_hz * 10000.0);
    uint8_t odr_val;
    
    switch(rate_int_10000){
        case 16000000: odr_val = GYRO_ODR_1600HZ; break;
        case 8000000: odr_val = GYRO_ODR_800HZ; break;
        case 4000000: odr_val = GYRO_ODR_400HZ; break;
        case 2000000: odr_val = GYRO_ODR_200HZ; break;
        case 1000000: odr_val = GYRO_ODR_100HZ; break;
        case 500000: odr_val = GYRO_ODR_50HZ; break;
        case 250000: odr_val = GYRO_ODR_25HZ; break;
        case 125000: odr_val = GYRO_ODR_12_5HZ; break;
        default: 
            ESP_LOGW(TAG, "Gyro ODR %f Hz not supported. Using 100Hz default.", rate_hz);
            odr_val = GYRO_ODR_100HZ;
            break;
    }

    uint8_t new_config = (current_config & 0xF0) | odr_val;
    ret = imu_reg_write(NULL, IMU_GYRO_CONFIG1, &new_config, 1);
    return ret;
}

esp_err_t imu_who_am_i(void){
    uint8_t who_am_i;
    return imu_reg_read(NULL, IMU_WHO_AM_I, &who_am_i, 1);
}

esp_err_t imu_intr_config(void){
    // 中断配置功能暂未实现
    return ESP_OK;
}