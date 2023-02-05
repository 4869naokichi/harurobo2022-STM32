/*
 * MPU9250.h
 *
 *  Created on: May 9, 2021
 *      Author: naokichi
 *
 *  HALを使用してMPU9250をSPI通信で動かすライブラリです。
 *  LLのほうが速いと思われます。
 *  DMPは未実装です。
 *  TODO オフセット調整←途中
 *  TODO 地磁気センサのキャリブレーション←無理みが深い
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "main.h"

//Registers
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define I2C_SLV0_DO         0x63
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C

typedef enum {
    ACCEL_RANGE_2G  = 0x00,
    ACCEL_RANGE_4G  = 0x08,
    ACCEL_RANGE_8G  = 0x10,
    ACCEL_RANGE_16G = 0x18
} accel_range_t;

typedef enum {
    GYRO_RANGE_250dps  = 0x00,
    GYRO_RANGE_500dps  = 0x08,
    GYRO_RANGE_1000dps = 0x10,
    GYRO_RANGE_2000dps = 0x18
} gyro_range_t;

#define ACCEL_RESOLUTION 32767.5f;
#define GYRO_RESOLUTION 32767.5f;
#define G 9.8f

class MPU9250 {
public:
    MPU9250();
    void init(SPI_HandleTypeDef, GPIO_TypeDef*, uint16_t, accel_range_t, gyro_range_t);
    void calibration();
    void update9DOF();
    float ax, ay, az;
    float gx, gy, gz;
    float hx, hy, hz;
private:
    SPI_HandleTypeDef m_hspi;
    GPIO_TypeDef* m_CS_GPIO_PORT;
    uint16_t m_CS_PIN;
    accel_range_t m_accel_range;
    gyro_range_t m_gyro_range;
    float m_accel_scale;
    float m_gyro_scale;
    float m_magx_scale, m_magy_scale, m_magz_scale;
    float m_ax_offset, m_ay_offset, m_az_offset;
    float m_gx_offset, m_gy_offset, m_gz_offset;
    float m_hx_offset, m_hy_offset, m_hz_offset;
    uint8_t readByte(uint8_t);
    void writeByte(uint8_t, uint8_t);
};

#endif /* MPU9250_H_ */
