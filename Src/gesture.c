#include "MPU6050/I2C.h"
#include "MPU6050/mpu6050.h"
void init_gesture()
{
    i2c_init(); // init hardware I2C
    DMP_Init();
}
