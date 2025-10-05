#include "imu.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_mac.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "rtc.h"
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "esp_err.h"
// SCL:IO19
// SDA:IO18
// Address:0x69(AD0=1) 0x68(AD0=0)
// 通过AD0引脚选择地址。默认接上拉10K电阻，电平逻辑置1
#define _PI 3.1415926535f
#define _RAD2DEG 57.2957795130f
#define _ALPHA 0.98f
static const char *TAG = "IMU";
//i2c_master_bus_handle_t mst_bus_handle;
i2c_master_dev_handle_t dev_handle;
i2c_master_bus_handle_t mst_bus_handle;

esp_err_t imu_self_test(void){
    ESP_LOGI(TAG, "开始IMU自检...");

    uint8_t who_am_i;
    esp_err_t ret = imu_reg_read(dev_handle, IMU_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取WHO_AM_I失败: 0x%x", ret);
        return ret;
    }
    
    ESP_LOGI(TAG, "设备ID: 0x%02X", who_am_i);
    
    if (who_am_i != 0x67) { 
        ESP_LOGE(TAG, "设备ID不匹配，期望: 0x67, 实际: 0x%02X", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "设备ID验证通过");
    
    uint8_t pwr_mgmt0;
    ret = imu_reg_read(dev_handle, IMU_PWR_MGMT0, &pwr_mgmt0, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "电源管理状态: 0x%02X", pwr_mgmt0);
    }

    ret = imu_wake_up();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "唤醒设备失败");
        return ret;
    }
    ESP_LOGI(TAG, "设备唤醒成功");
    
    vTaskDelay(pdMS_TO_TICKS(100)); 
    
    return ESP_OK;
}

esp_err_t imu_i2c_init(void) {
    ESP_LOGI(TAG, "开始IMU初始化...");
    
    i2c_master_bus_config_t master_conf = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 1,
        .flags.enable_internal_pullup = 1,
    };
    
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    // 初始化I2C总线
    esp_err_t ret = i2c_new_master_bus(&master_conf, &mst_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C总线初始化失败: 0x%x", ret);
        return ret;
    }
    
    // 添加设备
    ret = i2c_master_bus_add_device(mst_bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C设备添加失败: 0x%x", ret);
        i2c_del_master_bus(mst_bus_handle);
        mst_bus_handle = NULL;
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C初始化成功");
    
    // 等待设备稳定
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 执行自检
    ret = imu_self_test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU自检失败");
        imu_i2c_deinit();
        return ret;
    }
    
    // 配置传感器
    ESP_LOGI(TAG, "配置传感器参数...");
    
    // 配置加速度计 (±8g)
    uint8_t acce_config = (ACCEL_UI_FS_SEL_8G << 5);
    ret = imu_reg_write(dev_handle, IMU_ACCEL_CONFIG0, &acce_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "加速度计配置失败: 0x%x", ret);
    }
    
    // 配置陀螺仪 (±1000dps)
    uint8_t gyro_config = (GYRO_UI_FS_SEL_1000DPS << 5);
    ret = imu_reg_write(dev_handle, IMU_GYRO_CONFIG0, &gyro_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "陀螺仪配置失败: 0x%x", ret);
    }
    
    // 设置采样率
    imu_acce_set_sampling_rate(100.0);
    imu_gyro_set_sampling_rate(100.0);
    
    // 唤醒设备
    ret = imu_wake_up();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设备唤醒失败");
        imu_i2c_deinit();
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    ESP_LOGI(TAG, "IMU初始化完成");
    return ESP_OK;
}
void imu_i2c_deinit(void){
    if (dev_handle) {
        i2c_master_bus_rm_device(dev_handle);
        dev_handle = NULL;
    }
    if (mst_bus_handle) {
        i2c_del_master_bus(mst_bus_handle);
        mst_bus_handle = NULL;
    }
    printf("I2C deinitialized");
}
// static uint8_t imu_base_address = 0;
// static inline esp_err_t imu_reg_write(imu_handle_t imu_handle,imu_reg_bank0_map_t reg,uint8_t const *data,const uint8_t len){
//     // 写入寄存器数据
//     volatile uint8_t* reg_addr = (volatile uint8_t*)(imu_base_address + reg);
//     ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &reg_addr, 1, 100));
//     ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, len, 100));
//     return ESP_OK;
// }

// static inline esp_err_t imu_reg_read(imu_handle_t imu_handle,uint8_t reg,uint8_t const *data,const uint8_t len){
//     // 读取寄存器数据
//     volatile uint8_t* reg_addr = (volatile uint8_t*)(imu_base_address + reg);
//     ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, 100));
//     return ESP_OK;
// }

esp_err_t imu_debug_registers(void) {
    uint8_t reg_data;
    esp_err_t ret;
    
    ESP_LOGI(TAG, "=== 寄存器调试信息 ===");
    
    // 检查关键寄存器
    uint8_t debug_regs[] = {
        IMU_WHO_AM_I, IMU_PWR_MGMT0, IMU_ACCEL_CONFIG0, 
        IMU_GYRO_CONFIG0, IMU_ACCEL_CONFIG1, IMU_GYRO_CONFIG1
    };
    const char* reg_names[] = {
        "WHO_AM_I", "PWR_MGMT0", "ACCEL_CONFIG0", 
        "GYRO_CONFIG0", "ACCEL_CONFIG1", "GYRO_CONFIG1"
    };
    
    for (int i = 0; i < sizeof(debug_regs); i++) {
        ret = imu_reg_read(dev_handle, debug_regs[i], &reg_data, 1);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "%s: 0x%02X", reg_names[i], reg_data);
        } else {
            ESP_LOGE(TAG, "读取 %s 失败: 0x%x", reg_names[i], ret);
        }
    }
    
    return ESP_OK;
}
// esp_err_t imu_reg_write(imu_handle_t imu_handle, uint8_t reg_addr, uint8_t const *data, const uint8_t len){
//     uint8_t write_buf[len + 1];
//     write_buf[0] = reg_addr;
//     memcpy(&write_buf[1], data, len);
//     return i2c_master_transmit(dev_handle, write_buf, len + 1, 100);
// }

// esp_err_t imu_reg_read(imu_handle_t imu_handle, uint8_t reg_addr, uint8_t *data, const uint8_t len){
//     esp_err_t ret = i2c_master_transmit(dev_handle, &reg_addr, 1, 100);
//     if (ret != ESP_OK) return ret;
//     return i2c_master_receive(dev_handle, data, len, 100);
// }
// 修正寄存器读写函数
// 更健壮的寄存器读写函数
esp_err_t imu_reg_write(imu_handle_t imu_handle, uint8_t reg_addr, uint8_t const *data, const uint8_t len) {
    if (len == 0) return ESP_ERR_INVALID_ARG;
    
    uint8_t *write_buf = malloc(len + 1);
    if (write_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, data, len);
    
    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, len + 1, 1000);
    free(write_buf);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "寄存器写入失败: 地址=0x%02X, 错误=0x%x", reg_addr, ret);
    }
    return ret;
}

esp_err_t imu_reg_read(imu_handle_t imu_handle, uint8_t reg_addr, uint8_t *data, const uint8_t len) {
    if (len == 0) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, 
                                              &reg_addr, 1, 
                                              data, len, 
                                              1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "寄存器读取失败: 地址=0x%02X, 错误=0x%x", reg_addr, ret);
    }
    return ret;
}

esp_err_t imu_debug_raw_data(void) {
    uint8_t raw_data[6];
    esp_err_t ret;
    
    ESP_LOGI(TAG, "=== 原始数据调试 ===");
    
    ret = imu_reg_read(dev_handle, IMU_ACCEL_DATA_X1, raw_data, 6);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "加速度原始数据:");
        ESP_LOGI(TAG, "X1=0x%02X, X0=0x%02X", raw_data[0], raw_data[1]);
        ESP_LOGI(TAG, "Y1=0x%02X, Y0=0x%02X", raw_data[2], raw_data[3]);
        ESP_LOGI(TAG, "Z1=0x%02X, Z0=0x%02X", raw_data[4], raw_data[5]);
        
        int16_t raw_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        int16_t raw_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        int16_t raw_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
        ESP_LOGI(TAG, "加速度原始值: X=%d, Y=%d, Z=%d", raw_x, raw_y, raw_z);
    } else {
        ESP_LOGE(TAG, "读取加速度原始数据失败");
    }
    
    // 读取陀螺仪原始数据
    ret = imu_reg_read(dev_handle, IMU_GYRO_DATA_X1, raw_data, 6);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "陀螺仪原始数据:");
        ESP_LOGI(TAG, "X1=0x%02X, X0=0x%02X", raw_data[0], raw_data[1]);
        ESP_LOGI(TAG, "Y1=0x%02X, Y0=0x%02X", raw_data[2], raw_data[3]);
        ESP_LOGI(TAG, "Z1=0x%02X, Z0=0x%02X", raw_data[4], raw_data[5]);
        
        int16_t raw_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        int16_t raw_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        int16_t raw_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
        ESP_LOGI(TAG, "陀螺仪原始值: X=%d, Y=%d, Z=%d", raw_x, raw_y, raw_z);
    } else {
        ESP_LOGE(TAG, "读取陀螺仪原始数据失败");
    }
    
    // 读取温度原始数据
    uint8_t temp_data[2];
    ret = imu_reg_read(dev_handle, IMU_TEMP_DATA1, temp_data, 2);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "温度原始数据: TEMP_DATA1=0x%02X, TEMP_DATA0=0x%02X", 
                temp_data[0], temp_data[1]);
        int16_t temp_raw = (int16_t)((temp_data[0] << 8) | temp_data[1]);
        ESP_LOGI(TAG, "温度原始值: %d", temp_raw);
    } else {
        ESP_LOGE(TAG, "读取温度原始数据失败");
    }
    
    return ESP_OK;
}
// 修正灵敏度获取函数
esp_err_t imu_get_acce_sensitivity(float *const acce_sensitivity) {
    uint8_t data;
    esp_err_t ret = imu_reg_read(dev_handle, IMU_ACCEL_CONFIG0, &data, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t fs_sel = (data >> 5) & 0x03;
    switch(fs_sel) {
        case ACCEL_UI_FS_SEL_2G:
            *acce_sensitivity = 16384.0f;  // LSB/g
            break;
        case ACCEL_UI_FS_SEL_4G:
            *acce_sensitivity = 8192.0f;   // LSB/g
            break;
        case ACCEL_UI_FS_SEL_8G:
            *acce_sensitivity = 4096.0f;   // LSB/g
            break;
        case ACCEL_UI_FS_SEL_16G:
            *acce_sensitivity = 2048.0f;   // LSB/g
            break;
        default:
            *acce_sensitivity = 4096.0f;   // 默认值
            break;
    }
    return ESP_OK;
}

esp_err_t imu_get_gyro_sensitivity(float *const gyro_sensitivity) {
    uint8_t data;
    esp_err_t ret = imu_reg_read(dev_handle, IMU_GYRO_CONFIG0, &data, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    uint8_t fs_sel = (data >> 5) & 0x03;
    switch(fs_sel) {
        case GYRO_UI_FS_SEL_250DPS:
            *gyro_sensitivity = 131.0f;    // LSB/dps
            break;
        case GYRO_UI_FS_SEL_500DPS:
            *gyro_sensitivity = 65.5f;     // LSB/dps
            break;
        case GYRO_UI_FS_SEL_1000DPS:
            *gyro_sensitivity = 32.8f;     // LSB/dps
            break;
        case GYRO_UI_FS_SEL_2000DPS:
            *gyro_sensitivity = 16.4f;     // LSB/dps
            break;
        default:
            *gyro_sensitivity = 32.8f;     // 默认值
            break;
    }
    return ESP_OK;
}

esp_err_t imu_get_raw_acce_data(imu_acce_raw_data_t *raw_acce_val){
    uint8_t acce_data[6];
    esp_err_t ret = imu_reg_read(dev_handle,IMU_ACCEL_DATA_X1,acce_data,sizeof(acce_data));
    raw_acce_val->acce_raw_x = (int16_t)((acce_data[0] << 8) | acce_data[1]);// TODO: 验证：位或代替加法？
    raw_acce_val->acce_raw_y = (int16_t)((acce_data[2] << 8) | acce_data[3]);
    raw_acce_val->acce_raw_z = (int16_t)((acce_data[4] << 8) | acce_data[5]);
    return ret;
}

esp_err_t imu_get_raw_gyro_data(imu_gyro_raw_data_t *raw_gyro_val){
    uint8_t gyro_data[6];
    esp_err_t ret = imu_reg_read(dev_handle, IMU_GYRO_DATA_X1, gyro_data, sizeof(gyro_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取陀螺仪原始数据失败: 0x%x", ret);
        return ret;
    }
    
    raw_gyro_val->gyro_raw_x = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
    raw_gyro_val->gyro_raw_y = (int16_t)((gyro_data[2] << 8) | gyro_data[3]); 
    raw_gyro_val->gyro_raw_z = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);
    
    return ret;
}

esp_err_t imu_get_acce_data(imu_acce_data_t *acce_val){
    imu_acce_raw_data_t raw_acce_val;
    float acce_sensitivity = 0;
    esp_err_t ret = imu_get_acce_sensitivity(&acce_sensitivity);
    if(ret != ESP_OK){
        return ret;
    }
    ret = imu_get_raw_acce_data(&raw_acce_val);
    if(ret != ESP_OK){
        return ret;
    }
    acce_val->acce_x = raw_acce_val.acce_raw_x / (32768.0f / acce_sensitivity);
    acce_val->acce_y = raw_acce_val.acce_raw_y / (32768.0f / acce_sensitivity);
    acce_val->acce_z = raw_acce_val.acce_raw_z / (32768.0f / acce_sensitivity);
    return ret;
}

esp_err_t imu_get_gyro_data(imu_gyro_data_t *gyro_val){
    imu_gyro_raw_data_t raw_gyro_val;
    float gyro_sensitivity = 0;
    esp_err_t ret = imu_get_gyro_sensitivity(&gyro_sensitivity);
    if(ret != ESP_OK){
        return ret;
    }
    ret = imu_get_raw_gyro_data(&raw_gyro_val);
    if(ret != ESP_OK){
        return ret;
    }
    
    gyro_val->gyro_x = raw_gyro_val.gyro_raw_x / (32768.0f / gyro_sensitivity);
    gyro_val->gyro_y = raw_gyro_val.gyro_raw_y / (32768.0f / gyro_sensitivity);
    gyro_val->gyro_z = raw_gyro_val.gyro_raw_z / (32768.0f / gyro_sensitivity);
    return ret;
}

esp_err_t imu_get_temp_data(float *const temp){
    uint8_t temp_data[2];
    esp_err_t ret = imu_reg_read(dev_handle,IMU_TEMP_DATA1,temp_data,sizeof(temp_data));
    if(ret != ESP_OK){
        return ret;
    }
    int16_t temp_raw = (int16_t)((temp_data[0] << 8) | temp_data[1]);
    //Temperature in Degrees Centigrade = (TEMP_DATA / 128) + 25
    *temp = temp_raw/128.0f + 25.0f;
    return ret;
}


esp_err_t imu_comp_filter(imu_acce_data_t *const acce_val, imu_gyro_data_t *const gyro_val, imu_comp_angle_t *const comp_angle) {
    static float comp_angle_prev[2] = {0.0f, 0.0f};  
    static uint64_t last_time_us = 0;
    
    float acce_angle[2] = {0.0f, 0.0f};
    float gyro_angle[2] = {0.0f, 0.0f};
    
    // 计算加速度计角度
    acce_angle[0] = atan2f(acce_val->acce_y, acce_val->acce_z) * _RAD2DEG;  // 横滚
    acce_angle[1] = atan2f(-acce_val->acce_x, sqrtf(acce_val->acce_y * acce_val->acce_y + acce_val->acce_z * acce_val->acce_z)) * _RAD2DEG;  // 俯仰
    
    uint64_t current_time_us = esp_rtc_get_time_us();
    
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
    
    if (dt <= 0 || dt > 1.0f) {  // 无效时间间隔
        dt = 0.01f;  // 使用默认值
    }
    
    // 陀螺仪积分角度
    gyro_angle[0] = comp_angle_prev[0] + gyro_val->gyro_x * dt;
    gyro_angle[1] = comp_angle_prev[1] + gyro_val->gyro_y * dt;
    
    // 互补滤波融合
    comp_angle->roll_angle = _ALPHA * gyro_angle[0] + (1.0f - _ALPHA) * acce_angle[0];
    comp_angle->pitch_angle = _ALPHA * gyro_angle[1] + (1.0f - _ALPHA) * acce_angle[1];
    
    // 更新历史值
    comp_angle_prev[0] = comp_angle->roll_angle;
    comp_angle_prev[1] = comp_angle->pitch_angle;
    
    return ESP_OK;
}
// 批量读取传感器数据（提高效率）
esp_err_t imu_get_all_data(imu_acce_data_t *acce_val, imu_gyro_data_t *gyro_val, float *temp) {
    uint8_t sensor_data[14]; // 6加速度 + 6陀螺仪 + 2温度
    
    esp_err_t ret = imu_reg_read(dev_handle, IMU_ACCEL_DATA_X1, sensor_data, sizeof(sensor_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "批量读取传感器数据失败: 0x%x", ret);
        return ret;
    }
    
    // 解析加速度数据
    int16_t accel_raw[3];
    accel_raw[0] = (int16_t)((sensor_data[0] << 8) | sensor_data[1]);
    accel_raw[1] = (int16_t)((sensor_data[2] << 8) | sensor_data[3]);
    accel_raw[2] = (int16_t)((sensor_data[4] << 8) | sensor_data[5]);
    
    // 解析陀螺仪数据
    int16_t gyro_raw[3];
    gyro_raw[0] = (int16_t)((sensor_data[6] << 8) | sensor_data[7]);
    gyro_raw[1] = (int16_t)((sensor_data[8] << 8) | sensor_data[9]);
    gyro_raw[2] = (int16_t)((sensor_data[10] << 8) | sensor_data[11]);

    int16_t temp_raw = (int16_t)((sensor_data[12] << 8) | sensor_data[13]);

    float acce_sensitivity, gyro_sensitivity;
    imu_get_acce_sensitivity(&acce_sensitivity);
    imu_get_gyro_sensitivity(&gyro_sensitivity);

    acce_val->acce_x = (float)accel_raw[0] / 4096.0f;
    acce_val->acce_y = (float)accel_raw[1] / 4096.0f;
    acce_val->acce_z = (float)accel_raw[2] / 4096.0f;

    gyro_val->gyro_x = (float)gyro_raw[0] / 32.8f;
    gyro_val->gyro_y = (float)gyro_raw[1] / 32.8f;
    gyro_val->gyro_z = (float)gyro_raw[2] / 32.8f;
    
    *temp = (float)temp_raw / 128.0f + 25.0f;
    
    return ESP_OK;
}
// esp_err_t imu_wake_up(void){
//     // ACCEL工作在LP模式下
//     esp_err_t ret;
//     uint8_t mode;
//     ret=imu_reg_read(dev_handle,IMU_PWR_MGMT0,&mode,1);
//     if(ESP_OK != ret)
//     {
//         return ret;
//     }
//     mode = (mode & 0xF0) | 0x06; // 设置ACCEL工作在LP模式下，GYRO工作在standby模式下
//     ret=imu_reg_write(dev_handle,IMU_PWR_MGMT0,&mode,1);
//     return ret;
    
// }
esp_err_t imu_wake_up(void) {
    ESP_LOGI(TAG, "唤醒IMU设备...");
    
    // 设置加速度计和陀螺仪都工作在低噪声模式
    uint8_t mode = 0x0F;  // 加速度计和陀螺仪都使能
    esp_err_t ret = imu_reg_write(dev_handle, IMU_PWR_MGMT0, &mode, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "唤醒设备失败: 0x%x", ret);
    } else {
        ESP_LOGI(TAG, "设备唤醒成功");
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); // 等待设备稳定
    return ret;
}
esp_err_t imu_sleep(void){
    esp_err_t ret;
    uint8_t mode;
    ret=imu_reg_read(dev_handle,IMU_PWR_MGMT0,&mode,1);
    if(ESP_OK != ret)
    {
        return ret;
    }
    mode = (mode & 0xF0) | 0x01; // 设置ACCEL和GYRO都关闭
    ret=imu_reg_write(dev_handle,IMU_PWR_MGMT0,&mode,1);
    return ret;
}

esp_err_t imu_who_am_i(void){
    esp_err_t ret;
    uint8_t who_am_i;
    ret=imu_reg_read(dev_handle,IMU_WHO_AM_I,&who_am_i,1);
    if(ESP_OK != ret)
    {
        return ret;
    }
    return 0;
}

esp_err_t imu_acce_set_sampling_rate(double rate_hz){
    esp_err_t ret;
    uint8_t rate;
    ret=imu_reg_read(dev_handle,IMU_ACCEL_CONFIG1,&rate,1);
    if(ESP_OK != ret)
    {
        return ret;
    }
    int rate_int = (int)(rate_hz*10000);
    switch(rate_int){
        case 16000000: rate = (rate & 0xF0) | (ACCEL_ODR_1600HZ); break;
        case 8000000: rate = (rate & 0xF0) | (ACCEL_ODR_800HZ); break;
        case 4000000: rate = (rate & 0xF0) | (ACCEL_ODR_400HZ); break;
        case 2000000: rate = (rate & 0xF0) | (ACCEL_ODR_200HZ); break;
        case 1000000: rate = (rate & 0xF0) | (ACCEL_ODR_100HZ); break;
        case 500000: rate = (rate & 0xF0) | (ACCEL_ODR_50HZ); break;
        case 250000: rate = (rate & 0xF0) | (ACCEL_ODR_25HZ); break;
        case 125000: rate = (rate & 0xF0) | (ACCEL_ODR_12_5HZ); break;
        case 62500: rate = (rate & 0xF0) | (ACCEL_ODR_6_25HZ); break;
        case 31250: rate = (rate & 0xF0) | (ACCEL_ODR_3_125HZ); break;
        case 15625: rate = (rate & 0xF0) | (ACCEL_ODR_1_5625HZ); break;
        default: return ESP_ERR_INVALID_ARG;break;
    }
    ret = imu_reg_write(dev_handle,IMU_ACCEL_CONFIG1,&rate,1);
    return ret;
}

esp_err_t imu_gyro_set_sampling_rate(double rate_hz){
    esp_err_t ret;
    uint8_t rate;
    ret=imu_reg_read(dev_handle,IMU_GYRO_CONFIG1,&rate,1);
    if(ESP_OK != ret)
    {
        return ret;
    }
    int rate_int = (int)(rate_hz*10000);
    switch(rate_int){
        case 16000000: rate = (rate & 0xF0) | (GYRO_ODR_1600HZ); break;
        case 8000000: rate = (rate & 0xF0) | (GYRO_ODR_800HZ); break;
        case 4000000: rate = (rate & 0xF0) | (GYRO_ODR_400HZ); break;
        case 2000000: rate = (rate & 0xF0) | (GYRO_ODR_200HZ); break;
        case 1000000: rate = (rate & 0xF0) | (GYRO_ODR_100HZ); break;
        case 500000: rate = (rate & 0xF0) | (GYRO_ODR_50HZ); break;
        case 250000: rate = (rate & 0xF0) | (GYRO_ODR_25HZ); break;
        case 125000: rate = (rate & 0xF0) | (GYRO_ODR_12_5HZ); break;
        default: return ESP_ERR_INVALID_ARG;break;
    }
    ret = imu_reg_write(dev_handle,IMU_GYRO_CONFIG1,&rate,1);
    return ret;
}

esp_err_t imu_intr_config(void){
    // 我使用的硬件默认不连接中断引脚，暂缓更新
    return ESP_OK;
}

void kalman_init(KalmanFilter* kf, float Q_angle, float Q_gyro, float R_angle) {
    kf->Q_angle = Q_angle;
    kf->Q_gyro = Q_gyro;
    kf->R_angle = R_angle;
    kf->x_angle = 0;
    kf->x_bias = 0;
    kf->P[0][0] = 0;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 0;
}

float kalman_update(KalmanFilter* kf, float angle_m, float gyro_m, float dt) {
    // 预测步骤
    kf->x_angle += dt * (gyro_m - kf->x_bias);
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_gyro * dt;

    // 更新步骤
    kf->y = angle_m - kf->x_angle;
    kf->S = kf->P[0][0] + kf->R_angle;
    kf->K[0] = kf->P[0][0] / kf->S;
    kf->K[1] = kf->P[1][0] / kf->S;

    kf->x_angle += kf->K[0] * kf->y;
    kf->x_bias += kf->K[1] * kf->y;

    kf->P[0][0] -= kf->K[0] * kf->P[0][0];
    kf->P[0][1] -= kf->K[0] * kf->P[0][1];
    kf->P[1][0] -= kf->K[1] * kf->P[0][0];
    kf->P[1][1] -= kf->K[1] * kf->P[0][1];

    return kf->x_angle;
}