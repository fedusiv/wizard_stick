#include "MPU6050/I2C.h"
#include "stick_conf.h"


static I2C_HandleTypeDef i2cHandle;
static void Error_Handler(void);

void i2c_init( void )
{

    GPIO_InitTypeDef  GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    I2Cx_SCL_GPIO_CLK_ENABLE();
    I2Cx_SDA_GPIO_CLK_ENABLE();
    /* Enable I2Cx clock */
    I2Cx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* I2C TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

    /* I2C RX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = I2Cx_SDA_PIN;
    HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);

    i2cHandle.Instance             = I2Cx;
    i2cHandle.Init.ClockSpeed      = I2C_SPEEDCLOCK;
    i2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE;
    i2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
    i2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_10BIT;
    i2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2cHandle.Init.OwnAddress2     = 0xFF;
    i2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if(HAL_I2C_Init(&i2cHandle) != HAL_OK)
    {
      Error_Handler();
    }

}

HAL_StatusTypeDef i2c_write(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t length, uint8_t const *data) {
  return HAL_I2C_Mem_Write(&i2cHandle, slave_addr << 1, reg_addr,
  I2C_MEMADD_SIZE_8BIT, data, length, 10);
}

HAL_StatusTypeDef i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length,
    uint8_t *data) {
  return HAL_I2C_Mem_Read(&i2cHandle, slave_addr << 1, reg_addr,
  I2C_MEMADD_SIZE_8BIT, data, length, 10);

}

HAL_StatusTypeDef IICwriteBit(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t bitNum, uint8_t data) {
  uint8_t tmp;
  i2c_read(slave_addr, reg_addr, 1, &tmp);
  tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
  return i2c_write(slave_addr, reg_addr, 1, &tmp);
}


HAL_StatusTypeDef IICwriteBits(uint8_t slave_addr, uint8_t reg_addr,
    uint8_t bitStart, uint8_t length, uint8_t data) {

  uint8_t tmp, dataShift;
  HAL_StatusTypeDef status = i2c_read(slave_addr, reg_addr, 1, &tmp);
  if (status == HAL_OK) {
    uint8_t mask = (((1 << length) - 1) << (bitStart - length + 1));
    dataShift = data << (bitStart - length + 1);
    tmp &= mask;
    tmp |= dataShift;
    return i2c_write(slave_addr, reg_addr, 1, &tmp);
  } else {
    return status;
  }
}


void Error_Handler()
{

}


