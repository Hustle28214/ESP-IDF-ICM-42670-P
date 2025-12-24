#include <stdio.h>
#include <math.h>
#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "IMU_EXAMPLE";

// 示例1: 基础初始化和读取数据
void imu_example_basic(void)
{
    ESP_LOGI(TAG, "IMU Basic Example Starting...");
    
    // 1. 初始化I2C和IMU
    esp_err_t ret = imu_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "IMU initialized successfully");
    
    // 2. 打印寄存器信息（调试用）
    imu_debug_registers();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 3. 读取传感器数据
    for (int i = 0; i < 10; i++) {
        imu_acce_data_t accel;
        imu_gyro_data_t gyro;
        float temp;
        
        // 批量读取（推荐方式）
        ret = imu_get_all_data(&accel, &gyro, &temp);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Sample %d:", i + 1);
            ESP_LOGI(TAG, "  Accel: X=%.2f g, Y=%.2f g, Z=%.2f g", 
                    accel.acce_x, accel.acce_y, accel.acce_z);
            ESP_LOGI(TAG, "  Gyro:  X=%.2f dps, Y=%.2f dps, Z=%.2f dps", 
                    gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
            ESP_LOGI(TAG, "  Temp:  %.2f °C", temp);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms采样间隔
    }
    
    // 4. 清理资源
    imu_i2c_deinit();
    ESP_LOGI(TAG, "IMU Basic Example Finished");
}

// 示例2: 实时角度计算（互补滤波）
void imu_example_angle_calculation(void)
{
    ESP_LOGI(TAG, "IMU Angle Calculation Example Starting...");
    
    esp_err_t ret = imu_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed!");
        return;
    }
    
    ESP_LOGI(TAG, "Place IMU on flat surface for calibration...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // 等待2秒让用户放置设备
    
    imu_acce_data_t accel;
    imu_gyro_data_t gyro;
    float temp;
    imu_comp_angle_t angle;
    
    ESP_LOGI(TAG, "Starting angle calculation (Press Ctrl+C to stop)");
    
    for (int i = 0; i < 50; i++) {
        ret = imu_get_all_data(&accel, &gyro, &temp);
        if (ret == ESP_OK) {
            // 使用互补滤波计算角度
            imu_comp_filter(&accel, &gyro, &angle);
            
            // 从加速度计直接计算角度（用于比较）
            float acc_pitch = atan2f(accel.acce_y, accel.acce_z) * 57.2957795f;
            float acc_roll = atan2f(-accel.acce_x, 
                                  sqrtf(accel.acce_y * accel.acce_y + 
                                        accel.acce_z * accel.acce_z)) * 57.2957795f;
            
            ESP_LOGI(TAG, "Angles:");
            ESP_LOGI(TAG, "  Complementary Filter - Pitch: %.2f°, Roll: %.2f°", 
                    angle.pitch_angle, angle.roll_angle);
            ESP_LOGI(TAG, "  Accelerometer Only  - Pitch: %.2f°, Roll: %.2f°", 
                    acc_pitch, acc_roll);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms采样间隔
    }
    
    imu_i2c_deinit();
    ESP_LOGI(TAG, "Angle Calculation Example Finished");
}

// 示例3: 高级功能演示
void imu_example_advanced(void)
{
    ESP_LOGI(TAG, "IMU Advanced Example Starting...");
    
    esp_err_t ret = imu_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed!");
        return;
    }
    
    // 1. 修改采样率
    ESP_LOGI(TAG, "Setting sampling rates...");
    imu_acce_set_sampling_rate(200.0);  // 200Hz加速度计
    imu_gyro_set_sampling_rate(100.0);  // 100Hz陀螺仪
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 2. 读取灵敏度信息
    float accel_sens, gyro_sens;
    imu_get_acce_sensitivity(&accel_sens);
    imu_get_gyro_sensitivity(&gyro_sens);
    ESP_LOGI(TAG, "Sensor Sensitivities:");
    ESP_LOGI(TAG, "  Accelerometer: %.2f LSB/g", accel_sens);
    ESP_LOGI(TAG, "  Gyroscope:     %.2f LSB/dps", gyro_sens);
    
    // 3. 读取原始数据
    ESP_LOGI(TAG, "Reading raw data...");
    imu_acce_raw_data_t raw_accel;
    imu_gyro_raw_data_t raw_gyro;
    
    ret = imu_get_raw_acce_data(&raw_accel);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Raw Accel: X=%d, Y=%d, Z=%d", 
                raw_accel.acce_raw_x, raw_accel.acce_raw_y, raw_accel.acce_raw_z);
    }
    
    ret = imu_get_raw_gyro_data(&raw_gyro);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Raw Gyro:  X=%d, Y=%d, Z=%d", 
                raw_gyro.gyro_raw_x, raw_gyro.gyro_raw_y, raw_gyro.gyro_raw_z);
    }
    
    // 4. 演示休眠和唤醒功能
    ESP_LOGI(TAG, "Putting IMU to sleep...");
    imu_sleep();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Waking up IMU...");
    imu_wake_up();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 5. 验证设备ID
    uint8_t who_am_i;
    ret = imu_reg_read(NULL, IMU_WHO_AM_I, &who_am_i, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Device ID: 0x%02X", who_am_i);
    }
    
    imu_i2c_deinit();
    ESP_LOGI(TAG, "Advanced Example Finished");
}

// 示例4: 实时监控任务
static void imu_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "IMU Monitor Task Started");
    
    esp_err_t ret = imu_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed!");
        vTaskDelete(NULL);
        return;
    }
    
    // 设置更高的采样率
    imu_acce_set_sampling_rate(100.0);
    imu_gyro_set_sampling_rate(100.0);
    
    imu_acce_data_t accel;
    imu_gyro_data_t gyro;
    float temp;
    imu_comp_angle_t angle;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // 精确的100Hz采样周期
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        
        ret = imu_get_all_data(&accel, &gyro, &temp);
        if (ret == ESP_OK) {
            // 计算角度
            imu_comp_filter(&accel, &gyro, &angle);
            
            // 计算加速度计矢量大小（检测运动）
            float accel_magnitude = sqrtf(accel.acce_x * accel.acce_x + 
                                        accel.acce_y * accel.acce_y + 
                                        accel.acce_z * accel.acce_z);
            
            // 计算角速度大小
            float gyro_magnitude = sqrtf(gyro.gyro_x * gyro.gyro_x + 
                                       gyro.gyro_y * gyro.gyro_y + 
                                       gyro.gyro_z * gyro.gyro_z);
            
            // 检测运动阈值
            if (fabsf(accel_magnitude - 1.0f) > 0.1f || gyro_magnitude > 5.0f) {
                ESP_LOGW(TAG, "Motion detected!");
            }
            
            // 根据需要记录或传输数据
            ESP_LOGI(TAG, "Pitch: %6.2f°, Roll: %6.2f°, Acc: %.2fg, Gyro: %.2fdps", 
                    angle.pitch_angle, angle.roll_angle, accel_magnitude, gyro_magnitude);
        }
    }
    
    imu_i2c_deinit();
    vTaskDelete(NULL);
}

// 示例5: 集成到主应用
void app_main(void)
{
    ESP_LOGI(TAG, "========== IMU Demo Application ==========");
    
    // 运行示例1: 基础功能
    ESP_LOGI(TAG, "\n=== Example 1: Basic IMU Operation ===");
    imu_example_basic();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 运行示例2: 角度计算
    ESP_LOGI(TAG, "\n=== Example 2: Angle Calculation ===");
    imu_example_angle_calculation();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 运行示例3: 高级功能
    ESP_LOGI(TAG, "\n=== Example 3: Advanced Features ===");
    imu_example_advanced();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 启动示例4: 实时监控（使用独立任务）
    ESP_LOGI(TAG, "\n=== Example 4: Real-time Monitoring ===");
    ESP_LOGI(TAG, "Starting IMU monitoring task...");
    
    // 创建监控任务
    xTaskCreate(imu_monitor_task,     // 任务函数
                "imu_monitor",         // 任务名称
                4096,                  // 栈大小
                NULL,                  // 参数
                5,                     // 优先级
                NULL);                 // 任务句柄
    
    // 主任务可以继续执行其他工作
    int count = 0;
    while (1) {
        ESP_LOGI(TAG, "Main task running... Count: %d", ++count);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// 简化版本：快速启动IMU
void imu_quick_start(void)
{
    // 最简单的IMU使用方式
    if (imu_i2c_init() == ESP_OK) {
        imu_acce_data_t accel;
        imu_gyro_data_t gyro;
        float temp;
        
        while (1) {
            if (imu_get_all_data(&accel, &gyro, &temp) == ESP_OK) {
                printf("A:%.2f,%.2f,%.2f G:%.2f,%.2f,%.2f T:%.1f\n",
                       accel.acce_x, accel.acce_y, accel.acce_z,
                       gyro.gyro_x, gyro.gyro_y, gyro.gyro_z,
                       temp);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}