#ifndef ICM20948_ACCEL_CAL_H
#define ICM20948_ACCEL_CAL_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint8_t who_am_i;
} ICM20948_ACCEL_t;

typedef struct
{
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;

    float x_corrected_raw;
    float y_corrected_raw;
    float z_corrected_raw;

    float x_g;
    float y_g;
    float z_g;

    float x_ms2;
    float y_ms2;
    float z_ms2;
} ICM20948_ACCEL_Data_t;

typedef struct
{
    /* Six-position averages */
    float x_pos_x;
    float x_pos_y;
    float x_pos_z;

    float x_neg_x;
    float x_neg_y;
    float x_neg_z;

    float y_pos_x;
    float y_pos_y;
    float y_pos_z;

    float y_neg_x;
    float y_neg_y;
    float y_neg_z;

    float z_pos_x;
    float z_pos_y;
    float z_pos_z;

    float z_neg_x;
    float z_neg_y;
    float z_neg_z;

    /* Final calibration parameters */
    float offset_x_raw;
    float offset_y_raw;
    float offset_z_raw;

    float measured_lsb_per_g_x;
    float measured_lsb_per_g_y;
    float measured_lsb_per_g_z;

    float scale_x;
    float scale_y;
    float scale_z;

    uint8_t valid;
} ICM20948_ACCEL_Calibration_t;

HAL_StatusTypeDef ICM20948_ACCEL_Init(
        ICM20948_ACCEL_t *dev,
        SPI_HandleTypeDef *hspi,
        GPIO_TypeDef *cs_port,
        uint16_t cs_pin);

HAL_StatusTypeDef ICM20948_ACCEL_ReadRaw(
        ICM20948_ACCEL_t *dev,
        int16_t *ax,
        int16_t *ay,
        int16_t *az);

HAL_StatusTypeDef ICM20948_ACCEL_AverageRaw(
        ICM20948_ACCEL_t *dev,
        uint16_t sample_count,
        uint32_t sample_delay_ms,
        float *avg_x,
        float *avg_y,
        float *avg_z);

HAL_StatusTypeDef ICM20948_ACCEL_ComputeCalibration(
        ICM20948_ACCEL_Calibration_t *cal);

HAL_StatusTypeDef ICM20948_ACCEL_ReadCalibrated(
        ICM20948_ACCEL_t *dev,
        const ICM20948_ACCEL_Calibration_t *cal,
        ICM20948_ACCEL_Data_t *data);

#endif
