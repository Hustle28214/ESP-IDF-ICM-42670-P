#include "imu.h"

void imu_init(void){
    imu_wake_up();
    //imu_intr_config();
    imu_acce_set_sampling_rate(200);
    imu_gyro_set_sampling_rate(200);
    imu_who_am_i();
    
}