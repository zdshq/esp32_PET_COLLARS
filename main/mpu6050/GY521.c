
#include "GY521.h"


int mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, mpu6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a mpu6050 sensor register
 */
int mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, mpu6050_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 1,
        .scl_io_num = 2,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE
    };
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
//**************************************
//初始化MPU6050
//**************************************
int GY521_Init(void){
    kalman2_init(state,1,{1,1});
    i2c_master_init();
    vTaskDelay(300);
    uint8_t H;
    mpu6050_register_read(0x6a, &H,1);
    ESP_LOGE("@@@:","%d",H);
    // mpu6050_register_write_byte(PWR_MGMT_1, 0x00);	//解除休眠状态
    // mpu6050_register_write_byte(SMPLRT_DIV, 0x07);
    // mpu6050_register_write_byte(CONFIG, 0x06);
    // mpu6050_register_write_byte(GYRO_CONFIG, 0x18);
    // mpu6050_register_write_byte(ACCEL_CONFIG, 0x01);

    int a = mpu6050_register_write_byte(MPU_60X0_PWR_MGMT_1_REG_ADDR, MPU_60X0_RESET_REG_VALU);
    if (a)
        return a;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_PWR_MGMT_1_REG_ADDR, MPU_60X0_PWR_MGMT_1_REG_VALU);
    if (a)
        return a;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_USER_CTRL_REG_ADDR, MPU_60X0_USER_CTRL_REG_VALU);
    if (a)
        return a;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_SMPLRT_DIV_REG_ADDR, MPU_60X0_SMPLRT_DIV_REG_VALU);
    if (a)
        return a;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_CONFIG_REG_ADDR, MPU_60X0_CONFIG_REG_VALU);
    if (a)
        return a;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_GYRO_CONFIG_REG_ADDR, MPU_60X0_GYRO_CONFIG_REG_VALU);
    if (a)
        return a;
    vTaskDelay(1);
    
    a = mpu6050_register_write_byte(MPU_60X0_ACCEL_CONFIG_REG_ADDR, MPU_60X0_ACCEL_CONFIG_REG_VALU);
    if (a)
        return a;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_FIFO_EN_REG_ADDR, MPU_60X0_FIFO_EN_REG_VALU);
    if (a)
        return a;
    return a;
}



//**************************************
//合成数据
//**************************************
short GetData(uint8_t REG_Address)
{
    uint8_t H = 0, L = 0;
    int err;
    err = mpu6050_register_read(REG_Address, &H,1);
    if (err)
        return err;
    err = mpu6050_register_read(REG_Address, &L,1);
    if (err)
        return err;
    // ESP_LOGE("!!!","H:%d,l:%d",H,L);
    return (H<<8)+L; //合成数据
}