#ifndef __IMU_H__
#define __IMU_H__
#include <driver/gpio.h>
#include <driver/i2c.h>

#define IMU_I2C_ADDR              0x69                // IMU I2C address
#define I2C_MASTER_FREQ_HZ          1000000             // I2C master clock frequency
#define I2C_MASTER_SCL_IO           GPIO_NUM_19         
#define I2C_MASTER_SDA_IO           GPIO_NUM_18
#define I2C_MASTER_NUM              I2C_NUM_0         

typedef void *imu_handle_t;
extern i2c_master_bus_handle_t mst_bus_handle;

typedef enum{
    IMU_MCLK_RDY = 0x00,
    IMU_DEVICE_CONFIG = 0x01,
    IMU_SIGNAL_PATH_RESET =0x02,
    IMU_DRIVE_CONFIG1 = 0x03,// I3C驱动配置，一般不用
    IMU_DRIVE_CONFIG2 = 0x04,// I2C驱动配置，主要用这个
    IMU_DRIVE_CONFIG3 = 0x05,// SPI驱动配置
    IMU_INT_CONFIG = 0x06,
    IMU_TEMP_DATA1 = 0x09,
    IMU_TEMP_DATA0 = 0x0A,
    IMU_ACCEL_DATA_X1 = 0x0B,
    IMU_ACCEL_DATA_X0 = 0x0C,
    IMU_ACCEL_DATA_Y1 = 0x0D,
    IMU_ACCEL_DATA_Y0 = 0x0E,
    IMU_ACCEL_DATA_Z1 = 0x0F,
    IMU_ACCEL_DATA_Z0 = 0x10,
    IMU_GYRO_DATA_X1 = 0x11,
    IMU_GYRO_DATA_X0 = 0x12,
    IMU_GYRO_DATA_Y1 = 0x13,
    IMU_GYRO_DATA_Y0 = 0x14,
    IMU_GYRO_DATA_Z1 = 0x15,
    IMU_GYRO_DATA_Z0 = 0x16,
    IMU_TMST_FSYNCH = 0x17,
    IMU_TMST_FSYNCL = 0x18,
    IMU_APEX_DATA4 = 0x1D,
    IMU_APEX_DATA5 = 0x1E,
    IMU_PWR_MGMT0 = 0x1F,
    IMU_GYRO_CONFIG0 = 0x20,
    IMU_ACCEL_CONFIG0 = 0x21,
    IMU_TEMP_CONFIG0 = 0x22,
    IMU_GYRO_CONFIG1 = 0x23,
    IMU_ACCEL_CONFIG1 = 0x24,
    IMU_APEX_CONFIG0 = 0x25,
    IMU_APEX_CONFIG1 = 0x26,
    IMU_WOM_CONFIG = 0x27,
    IMU_FIFO_CONFIG1 = 0x28,
    IMU_FIFO_CONFIG2 = 0x29,
    IMU_FIFO_CONFIG3 = 0x2A,
    IMU_INT_SOURCE0 = 0x2B,
    IMU_INT_SOURCE1 = 0x2C,
    IMU_INT_SOURCE3 = 0x2D,
    IMU_INT_SOURCE4 = 0x2E,
    IMU_FIFO_LOST_PKT0 = 0x2F,
    IMU_FIFO_LOST_PKT1 = 0x30,
    IMU_APEX_DATA0 = 0x31,
    IMU_APEX_DATA1 = 0x32,
    IMU_APEX_DATA2 = 0x33,
    IMU_APEX_DATA3 = 0x34,
    IMU_INTF_CONFIG1 = 0x35,
    IMU_INT_STATUS_DRDY = 0x39,
    IMU_INT_STATUS = 0x3A,
    IMU_INT_STATUS2 = 0x3B,
    IMU_INT_STATUS3 = 0x3C,
    IMU_FIFO_COUNTH = 0x3D,
    IMU_FIFO_COUNTL = 0x3E,
    IMU_FIFO_DATA = 0x3F,
    IMU_WHO_AM_I = 0x75,
    IMU_BLK_SEL_W = 0x79,
    IMU_MADDR_W = 0x7A,
    IMU_M_W = 0x7B,
    IMU_BLK_SEL_R = 0x7C,
    IMU_MADDR_R = 0x7D,
    IMU_M_R = 0x7E,
}imu_reg_bank0_map_t;

typedef struct{
    gpio_num_t int_gpio;
    
}imu_intr_config_t;

typedef struct{
    int16_t acce_raw_x;
    int16_t acce_raw_y;
    int16_t acce_raw_z;
}imu_acce_raw_data_t;

typedef struct{
    int16_t gyro_raw_x;
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;
}
imu_gyro_raw_data_t;

typedef struct{
    int16_t acce_x;
    int16_t acce_y;
    int16_t acce_z;
}imu_acce_data_t;

typedef struct{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
}imu_gyro_data_t;

typedef enum{
    ACCEL_UI_FS_SEL_2G = 0x03,
    ACCEL_UI_FS_SEL_4G = 0x02,
    ACCEL_UI_FS_SEL_8G = 0x01,
    ACCEL_UI_FS_SEL_16G = 0x00,
}ACCEL_UI_FS_SEL;

typedef enum{
    GYRO_UI_FS_SEL_2000DPS =0x00,
    GYRO_UI_FS_SEL_1000DPS =0x01,
    GYRO_UI_FS_SEL_500DPS =0x02,
    GYRO_UI_FS_SEL_250DPS =0x03,
}GYRO_UI_FS_SEL;

typedef enum{
    GYRO_ODR_1600HZ = 0x05,
    GYRO_ODR_800HZ = 0x06,
    GYRO_ODR_400HZ = 0x07,
    GYRO_ODR_200HZ = 0x08,
    GYRO_ODR_100HZ = 0x09,
    GYRO_ODR_50HZ = 0x0A,
    GYRO_ODR_25HZ = 0x0B,
    GYRO_ODR_12_5HZ = 0x0C,
}GYRO_ODR;


typedef enum{
    // Reserved： 0x00-0x04
    ACCEL_ODR_1600HZ = 0x05,
    ACCEL_ODR_800HZ = 0x06,
    ACCEL_ODR_400HZ = 0x07,
    ACCEL_ODR_200HZ = 0x08,
    ACCEL_ODR_100HZ = 0x09,
    ACCEL_ODR_50HZ = 0x0A,
    ACCEL_ODR_25HZ = 0x0B,
    ACCEL_ODR_12_5HZ = 0x0C,
    ACCEL_ODR_6_25HZ = 0x0D,
    ACCEL_ODR_3_125HZ = 0x0E,
    ACCEL_ODR_1_5625HZ = 0x0F,
}ACCEL_ODR;

typedef enum{
    DLPF_BW_180HZ = 0x01,
    DLPF_BW_72HZ = 0x02,
    DLPF_BW_34HZ = 0x03,
    DLPF_BW_16HZ = 0x04,
    DLPF_BW_8HZ = 0x05,
    DLPF_BW_4HZ = 0x06,//MAX
    //DLPF_BW_4HZ_1 = 0x07,
}TEMP_FILT_BW;

typedef enum{
    GYRO_UI_FILT_BW_180HZ = 0x01,
    GYRO_UI_FILT_BW_121HZ = 0x02,
    GYRO_UI_FILT_BW_73HZ = 0x03,
    GYRO_UI_FILT_BW_53HZ = 0x04,
    GYRO_UI_FILT_BW_34HZ = 0x05,
    GYRO_UI_FILT_BW_25HZ = 0x06,
    GYRO_UI_FILT_BW_16HZ = 0x07,
}
GYRO_UI_FILT_BW;


typedef enum{
    ACCEL_UI_AVG_2X = 0x00,
    ACCEL_UI_AVG_4X = 0x01,
    ACCEL_UI_AVG_8X = 0x02,
    ACCEL_UI_AVG_16X = 0x03,
    ACCEL_UI_AVG_32X = 0x04,
    ACCEL_UI_AVG_64X = 0x05,//MAX
}ACCEL_UI_AVG;

typedef enum{
    ACCEL_UI_FILT_BW_180HZ = 0x01,
    ACCEL_UI_FILT_BW_121HZ = 0x02,
    ACCEL_UI_FILT_BW_73HZ = 0x03,
    ACCEL_UI_FILT_BW_53HZ = 0x04,
    ACCEL_UI_FILT_BW_34HZ = 0x05,
    ACCEL_UI_FILT_BW_25HZ = 0x06,
    ACCEL_UI_FILT_BW_16HZ = 0x07}
    ACCEL_UI_FILT_BW;


typedef struct{
    float pitch_angle;
    float roll_angle;
    float yaw_angle;
}imu_comp_angle_t;



void imu_i2c_init(void);
void imu_i2c_deinit(void);
static esp_err_t imu_reg_write(imu_handle_t imu_handle,uint8_t reg_addr,uint8_t const *data,const uint8_t len);
static esp_err_t imu_reg_read(imu_handle_t imu_handle,uint8_t reg_addr,uint8_t const *data,const uint8_t len);
esp_err_t imu_get_acce_sensitivity(float *const acce_sensitivity);
esp_err_t imu_get_gyro_sensitivity(float *const gyro_sensitivity);
esp_err_t imu_get_raw_acce_data(imu_acce_raw_data_t *raw_acce_val);
esp_err_t imu_get_raw_gyro_data(imu_gyro_raw_data_t *raw_gyro_val);
esp_err_t imu_get_acce_data(imu_acce_data_t *acce_val);
esp_err_t imu_get_gyro_data(imu_gyro_data_t *gyro_val);
esp_err_t imu_get_temp_data(float *const temp);
esp_err_t imu_comp_filter(imu_acce_data_t *const acce_val,imu_gyro_data_t *const gyro_val,imu_comp_angle_t *const comp_angle);
esp_err_t imu_wake_up(void);
esp_err_t imu_sleep(void);
esp_err_t imu_who_am_i(void);
esp_err_t imu_acce_set_sampling_rate(double rate_hz);
esp_err_t imu_gyro_set_sampling_rate(double rate_hz);
esp_err_t imu_intr_config(void);


#endif