/*
 * MPU9250.cpp
 *
 *  Created on: May 9, 2021
 *      Author: naokichi
 */

#include <MPU9250.h>

/**
  * @brief コンストラクタ
  */
MPU9250::MPU9250()
{
    m_ax_offset = 0; m_ay_offset = 0; m_az_offset = 0;
    m_gx_offset = 0; m_gy_offset = 0; m_gz_offset = 0;
    m_hx_offset = 0; m_hy_offset = 0; m_hz_offset = 0;
    ax = 0; ay = 0; az = 0;
    gx = 0; gy = 0; gz = 0;
    hx = 0; hy = 0; hz = 0;

    //地磁気のスケールはinit()内で設定されます。
    m_magx_scale = 1;
    m_magy_scale = 1;
    m_magz_scale = 1;
}

/**
  * @brief 1バイト読む
  * @param reg レジスタ
  */
uint8_t MPU9250::readByte(uint8_t reg)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    tx_data[0] = reg | 0x80;
    tx_data[1] = 0x00;  // dummy

    HAL_GPIO_WritePin(m_CS_GPIO_PORT, m_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&m_hspi, tx_data, rx_data, 2, 1);
    HAL_GPIO_WritePin(m_CS_GPIO_PORT, m_CS_PIN, GPIO_PIN_SET);

    return rx_data[1];
}

/**
  * @brief 1バイト書き込む
  * @param reg  レジスタ
  * @param data 書き込むデータ
  */
void MPU9250::writeByte(uint8_t reg, uint8_t data)
{
  uint8_t rx_data[2];
  uint8_t tx_data[2];

  tx_data[0] = reg & 0x7F;
  tx_data[1] = data;  // write data

  HAL_GPIO_WritePin(m_CS_GPIO_PORT, m_CS_PIN, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&m_hspi, tx_data, rx_data, 2, 1);
  HAL_GPIO_WritePin(m_CS_GPIO_PORT, m_CS_PIN, GPIO_PIN_SET);
}

/**
  * @brief 初期化を行う
  * @param hspi         SPIのハンドラ
  * @param CS_GPIO_PORT チップセレクトのポート
  * @param CS_PIN       チップセレクトのピン番号
  * @param accel_range  加速度のレンジ
  * @param gyro_range   ジャイロのレンジ
  * TODO コメントの追加
  */
void MPU9250::init(SPI_HandleTypeDef hspi, GPIO_TypeDef* CS_GPIO_PORT, uint16_t CS_PIN, accel_range_t accel_range, gyro_range_t gyro_range)
{
    m_hspi = hspi;
    m_CS_GPIO_PORT = CS_GPIO_PORT;
    m_CS_PIN = CS_PIN;
    m_accel_range = accel_range;
    m_gyro_range = gyro_range;

    switch(m_accel_range){
        case ACCEL_RANGE_2G:
            m_accel_scale = 2.0f / ACCEL_RESOLUTION;
            break;
        case ACCEL_RANGE_4G:
            m_accel_scale = 4.0f / ACCEL_RESOLUTION;
            break;
        case ACCEL_RANGE_8G:
            m_accel_scale = 8.0f / ACCEL_RESOLUTION;
            break;
        case ACCEL_RANGE_16G:
            m_accel_scale = 16.0f / ACCEL_RESOLUTION;
            break;
    }
    switch(m_gyro_range){
        case GYRO_RANGE_250dps:
            m_gyro_scale = 250.0f / GYRO_RESOLUTION;
            break;
        case GYRO_RANGE_500dps:
            m_gyro_scale = 500.0f / GYRO_RESOLUTION;
            break;
        case GYRO_RANGE_1000dps:
            m_gyro_scale = 1000.0f / GYRO_RESOLUTION;
            break;
        case GYRO_RANGE_2000dps:
            m_gyro_scale = 2000.0f / GYRO_RESOLUTION;
            break;
    }

    writeByte(PWR_MGMT_1, 0x00);
    writeByte(PWR_MGMT_2, 0x00);
    writeByte(CONFIG, 0x00);
    writeByte(ACCEL_CONFIG, m_accel_range);
    writeByte(GYRO_CONFIG, m_gyro_range);

    writeByte(USER_CTRL, 0x30);
    writeByte(I2C_MST_CTRL, 0x0D);

    writeByte(I2C_SLV0_ADDR, 0x0C);
    writeByte(I2C_SLV0_REG, 0x0B);
    writeByte(I2C_SLV0_DO, 0x01);
    writeByte(I2C_SLV0_CTRL, 0x81);
    HAL_Delay(100);

    writeByte(I2C_SLV0_ADDR, 0x0C);
    writeByte(I2C_SLV0_REG, 0x0A);
    writeByte(I2C_SLV0_DO, 0x16);
    writeByte(I2C_SLV0_CTRL, 0x81);
    HAL_Delay(100);

    writeByte(I2C_SLV0_ADDR, 0x0C | 0x80);
    writeByte(I2C_SLV0_REG, 0x10);
    writeByte(I2C_SLV0_CTRL, 0x83);
    HAL_Delay(500);
    m_magx_scale = (((readByte(EXT_SENS_DATA_00) - 128) * 0.5) / 128) + 1;
    m_magy_scale = (((readByte(EXT_SENS_DATA_01) - 128) * 0.5) / 128) + 1;
    m_magz_scale = (((readByte(EXT_SENS_DATA_02) - 128) * 0.5) / 128) + 1;

    writeByte(I2C_SLV0_ADDR, 0x0C | 0x80);
    writeByte(I2C_SLV0_REG, 0x03);
    writeByte(I2C_SLV0_CTRL, 0x87);
    HAL_Delay(100);
}

/**
  * @brief キャリブレーションを行う
  * @note 平均値をとっているだけ
  */
void MPU9250::calibration()
{
    int32_t ax = 0; int32_t ay = 0; int32_t az = 0;
    int32_t gx = 0; int32_t gy = 0; int32_t gz = 0;
    int32_t hx = 0; int32_t hy = 0; int32_t hz = 0;

    for(int i=0; i<1000; i++){
        ax += (int16_t)((int16_t)(readByte(ACCEL_XOUT_H) << 8) | readByte(ACCEL_XOUT_L));
        ay += (int16_t)((int16_t)(readByte(ACCEL_YOUT_H) << 8) | readByte(ACCEL_YOUT_L));
        az += (int16_t)((int16_t)(readByte(ACCEL_ZOUT_H) << 8) | readByte(ACCEL_ZOUT_L));
        gx += (int16_t)((int16_t)(readByte(GYRO_XOUT_H) << 8) | readByte(GYRO_XOUT_L));
        gy += (int16_t)((int16_t)(readByte(GYRO_YOUT_H) << 8) | readByte(GYRO_YOUT_L));
        gz += (int16_t)((int16_t)(readByte(GYRO_ZOUT_H) << 8) | readByte(GYRO_ZOUT_L));
        hx += (int16_t)(readByte(EXT_SENS_DATA_00) | (int16_t)(readByte(EXT_SENS_DATA_01) << 8));
        hy += (int16_t)(readByte(EXT_SENS_DATA_02) | (int16_t)(readByte(EXT_SENS_DATA_03) << 8));
        hz += (int16_t)(readByte(EXT_SENS_DATA_04) | (int16_t)(readByte(EXT_SENS_DATA_05) << 8));
        HAL_Delay(5);
    }

    m_ax_offset = ax / 1000;
    m_ay_offset = ay / 1000;
    m_az_offset = az / 1000;
    m_gx_offset = gx / 1000;
    m_gy_offset = gy / 1000;
    m_gz_offset = gz / 1000;
    m_hx_offset = hx / 1000;
    m_hy_offset = hy / 1000;
    m_hz_offset = hz / 1000;
}

/**
  * @brief 9軸のデータを更新する
  * @return 加速度 [G]
  * @return ジャイロ [dps]
  * @return 地磁気 [μT]
  */
void MPU9250::update9DOF()
{
    int16_t raw;

    raw = (int16_t)((int16_t)(readByte(ACCEL_XOUT_H) << 8) | readByte(ACCEL_XOUT_L));
    ax = (float)((raw - m_ax_offset) * m_accel_scale);

    raw = (int16_t)((int16_t)(readByte(ACCEL_YOUT_H) << 8) | readByte(ACCEL_YOUT_L));
    ay = (float)((raw - m_ay_offset) * m_accel_scale);

    raw = (int16_t)((int16_t)(readByte(ACCEL_ZOUT_H) << 8) | readByte(ACCEL_ZOUT_L));
    az = (float)((raw - m_az_offset) * m_accel_scale) + 1;

    raw = (int16_t)((int16_t)(readByte(GYRO_XOUT_H) << 8) | readByte(GYRO_XOUT_L));
    gx = (float)((raw - m_gx_offset) * m_gyro_scale);

    raw = (int16_t)((int16_t)(readByte(GYRO_YOUT_H) << 8) | readByte(GYRO_YOUT_L));
    gy = (float)((raw - m_gy_offset) * m_gyro_scale);

    raw = (int16_t)((int16_t)(readByte(GYRO_ZOUT_H) << 8) | readByte(GYRO_ZOUT_L));
    gz = (float)((raw - m_gz_offset) * m_gyro_scale);

    raw = (int16_t)(readByte(EXT_SENS_DATA_00) | (int16_t)(readByte(EXT_SENS_DATA_01) << 8));
    hx = (float)((raw - m_hx_offset) * m_magx_scale);

    raw = (int16_t)(readByte(EXT_SENS_DATA_02) | (int16_t)(readByte(EXT_SENS_DATA_03) << 8));
    hy = (float)((raw - m_hy_offset) * m_magy_scale);

    raw = (int16_t)(readByte(EXT_SENS_DATA_04) | (int16_t)(readByte(EXT_SENS_DATA_05) << 8));
    hz = (float)((raw - m_hz_offset) * m_magz_scale);
}
