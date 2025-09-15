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
// SCL:IO19
// SDA:IO18
// Address:0x69(AD0=1) 0x68(AD0=0)
// 通过AD0引脚选择地址。默认接上拉10K电阻，电平逻辑置1
#define _PI 3.1415926535f
#define _RAD2DEG 57.2957795130f
#define _ALPHA 0.98f
i2c_master_bus_handle_t mst_bus_handle;
i2c_master_dev_handle_t dev_handle;

void imu_i2c_init(void){
    // 用新的i2c_master.h库初始化I2C主机
    i2c_master_bus_handle_t mst_bus_handle;
    i2c_master_bus_config_t master_conf ={
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 1,
    };
    master_conf.flags.enable_internal_pullup = 0;// 不启用内部上拉,外部已经有10K上拉电阻
    master_conf.flags.allow_pd = 0;// 不允许掉电
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&master_conf,&mst_bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(mst_bus_handle,&dev_cfg,&dev_handle));
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
    printf("IMU", "I2C deinitialized");
}

static esp_err_t imu_reg_write(imu_handle_t imu_handle,uint8_t reg_addr,uint8_t const *data,const uint8_t len){
    // 写入寄存器数据
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &reg_addr, 1, 100));
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, len, 100));
    return ESP_OK;
}

static esp_err_t imu_reg_read(imu_handle_t imu_handle,uint8_t reg_addr,uint8_t const *data,const uint8_t len){
    // 读取寄存器数据
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, 100));
    return ESP_OK;
}

esp_err_t imu_get_acce_sensitivity(float *const acce_sensitivity){
    uint8_t data;
    esp_err_t ret = imu_reg_read(dev_handle,IMU_ACCEL_CONFIG0,&data,1);
    data = (data >> 5) & 0x03;
    switch(data){
        case ACCEL_UI_FS_SEL_2G:
            *acce_sensitivity = 2.0f;
            break;
        case ACCEL_UI_FS_SEL_4G:
            *acce_sensitivity = 4.0f;
            break;
        case ACCEL_UI_FS_SEL_8G:
            *acce_sensitivity = 8.0f;
            break;
        case ACCEL_UI_FS_SEL_16G:
            *acce_sensitivity = 16.0f;
            break;
        default:
            break;
    }
    return ret;
}

esp_err_t imu_get_gyro_sensitivity(float *const gyro_sensitivity){
    uint8_t data;
    esp_err_t ret = imu_reg_read(dev_handle,IMU_GYRO_CONFIG0,&data,1);
    data = (data >> 5) & 0x03;
    switch(data){
        case GYRO_UI_FS_SEL_250DPS:
            *gyro_sensitivity = 250.0f;
            break;
        case GYRO_UI_FS_SEL_500DPS:
            *gyro_sensitivity = 500.0f;
            break;
        case GYRO_UI_FS_SEL_1000DPS:
            *gyro_sensitivity = 1000.0f;
            break;
        case GYRO_UI_FS_SEL_2000DPS:
            *gyro_sensitivity = 2000.0f;
            break;
        default:
            break;
    }
    return ret;
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
    esp_err_t ret = imu_reg_read(dev_handle,IMU_GYRO_DATA_X1,gyro_data,sizeof(gyro_data));
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
    acce_val->acce_x = raw_acce_val.acce_raw_x/acce_sensitivity;
    acce_val->acce_y = raw_acce_val.acce_raw_y/acce_sensitivity;
    acce_val->acce_z = raw_acce_val.acce_raw_z/acce_sensitivity;
    return ret;
}

esp_err_t imu_get_gyro_data(imu_gyro_data_t *gyro_val){
    imu_gyro_raw_data_t raw_gyro_val;
    float gyro_sensitivity = 0;
    esp_err_t ret = imu_get_raw_gyro_data(&raw_gyro_val);
    if(ret != ESP_OK){
        return ret;
    }
    ret = imu_get_raw_gyro_data(&raw_gyro_val);
    if(ret != ESP_OK){
        return ret;
    }
    gyro_val->gyro_x = raw_gyro_val.gyro_raw_x/gyro_sensitivity;
    gyro_val->gyro_y = raw_gyro_val.gyro_raw_y/gyro_sensitivity;
    gyro_val->gyro_z = raw_gyro_val.gyro_raw_z/gyro_sensitivity;
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

esp_err_t imu_comp_filter(imu_acce_data_t *const acce_val,imu_gyro_data_t *const gyro_val,imu_comp_angle_t *const comp_angle){
    float acce_angle[2]= {0.0f,0.0f};
    float gyro_angle[2]= {0.0f,0.0f};
    float comp_angle_prev[2] = {0.0f, 0.0f};
    acce_angle[0] = atan2(acce_val->acce_y,acce_val->acce_z) * _RAD2DEG;
    acce_angle[1] = atan2(acce_val->acce_x,acce_val->acce_z) * _RAD2DEG;
    static uint64_t last_time_us = 0;
    uint64_t current_time_us = esp_rtc_get_time_us();
    if(last_time_us==0){
        last_time_us = current_time_us;
        comp_angle->roll_angle = acce_angle[0];
        comp_angle->pitch_angle = acce_angle[1];
        return ESP_OK;
    }
    float _dt = (float)(float)(current_time_us - last_time_us) / 1000000.0f;

    gyro_angle[0] = gyro_val->gyro_x * _dt;
    gyro_angle[1] = gyro_val->gyro_y * _dt;

    comp_angle->roll_angle = _ALPHA * (comp_angle_prev[0] + gyro_angle[0]) + (1 - _ALPHA) * acce_angle[0];
    comp_angle->pitch_angle = _ALPHA * (comp_angle_prev[1] + gyro_angle[1]) + (1 - _ALPHA) * acce_angle[1];

    return ESP_OK;
}

esp_err_t imu_wake_up(void){
    // ACCEL工作在LP模式下
    esp_err_t ret;
    uint8_t mode;
    ret=imu_reg_read(dev_handle,IMU_PWR_MGMT0,&mode,1);
    if(ESP_OK != ret)
    {
        return ret;
    }
    mode = (mode & 0xF0) | 0x06; // 设置ACCEL工作在LP模式下，GYRO工作在standby模式下
    ret=imu_reg_write(dev_handle,IMU_PWR_MGMT0,&mode,1);
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

esp_err_t imu_intr_config(){
    // 我使用的硬件默认不连接中断引脚，暂缓更新
    return ESP_OK;
}