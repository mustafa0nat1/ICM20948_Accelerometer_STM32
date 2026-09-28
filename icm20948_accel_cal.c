#include "icm20948_accel_cal.h"

/* ---------------- USER BANK 0 ---------------- */

#define ICM20948_REG_WHO_AM_I             0x00
#define ICM20948_REG_USER_CTRL            0x03
#define ICM20948_REG_PWR_MGMT_1           0x06
#define ICM20948_REG_PWR_MGMT_2           0x07
#define ICM20948_REG_ACCEL_XOUT_H          0x2D
#define ICM20948_REG_BANK_SEL              0x7F

#define ICM20948_WHO_AM_I_VALUE            0xEA

/* ---------------- USER BANK 2 ---------------- */

#define ICM20948_B2_ACCEL_SMPLRT_DIV_1     0x10
#define ICM20948_B2_ACCEL_SMPLRT_DIV_2     0x11
#define ICM20948_B2_ACCEL_CONFIG           0x14

#define ICM20948_BANK_0                    0
#define ICM20948_BANK_2                    2

/*
 * ACCEL_CONFIG = 0x11:
 *
 * ACCEL_DLPFCFG = 2
 * ACCEL_FS_SEL  = 0 -> +/-2 g
 * ACCEL_FCHOICE = 1 -> DLPF enabled
 *
 * Nominal sensitivity at +/-2 g = 16384 LSB/g.
 *
 * Six-position calibration measures the actual LSB/g of
 * each axis, then uses:
 *
 * corrected_raw = (raw - offset) * scale
 *
 * scale = 16384 / measured_lsb_per_g
 */
#define ICM20948_ACCEL_NOMINAL_LSB_PER_G   16384.0f

#define STANDARD_GRAVITY_MS2               9.80665f

/* ------------------------------------------------------- */

static void CS_Low(ICM20948_ACCEL_t *dev)
{
    HAL_GPIO_WritePin(
            dev->cs_port,
            dev->cs_pin,
            GPIO_PIN_RESET);
}

static void CS_High(ICM20948_ACCEL_t *dev)
{
    HAL_GPIO_WritePin(
            dev->cs_port,
            dev->cs_pin,
            GPIO_PIN_SET);
}

static HAL_StatusTypeDef WriteRegister(
        ICM20948_ACCEL_t *dev,
        uint8_t address,
        uint8_t value)
{
    uint8_t tx[2];

    tx[0] = address & 0x7F;
    tx[1] = value;

    CS_Low(dev);

    HAL_StatusTypeDef status =
            HAL_SPI_Transmit(
                    dev->hspi,
                    tx,
                    2,
                    1000);

    CS_High(dev);

    return status;
}

static HAL_StatusTypeDef ReadRegister(
        ICM20948_ACCEL_t *dev,
        uint8_t address,
        uint8_t *value)
{
    uint8_t tx[2] = {0};
    uint8_t rx[2] = {0};

    tx[0] = address | 0x80;

    CS_Low(dev);

    HAL_StatusTypeDef status =
            HAL_SPI_TransmitReceive(
                    dev->hspi,
                    tx,
                    rx,
                    2,
                    1000);

    CS_High(dev);

    if (status == HAL_OK)
    {
        *value = rx[1];
    }

    return status;
}

static HAL_StatusTypeDef ReadRegisters(
        ICM20948_ACCEL_t *dev,
        uint8_t start_address,
        uint8_t *data,
        uint8_t length)
{
    if ((data == NULL) || (length == 0) || (length > 16))
    {
        return HAL_ERROR;
    }

    uint8_t tx[17] = {0};
    uint8_t rx[17] = {0};

    tx[0] = start_address | 0x80;

    CS_Low(dev);

    HAL_StatusTypeDef status =
            HAL_SPI_TransmitReceive(
                    dev->hspi,
                    tx,
                    rx,
                    length + 1,
                    1000);

    CS_High(dev);

    if (status != HAL_OK)
    {
        return status;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = rx[i + 1];
    }

    return HAL_OK;
}

static HAL_StatusTypeDef SelectBank(
        ICM20948_ACCEL_t *dev,
        uint8_t bank)
{
    return WriteRegister(
            dev,
            ICM20948_REG_BANK_SEL,
            (uint8_t)((bank & 0x03) << 4));
}

/* ------------------------------------------------------- */

HAL_StatusTypeDef ICM20948_ACCEL_Init(
        ICM20948_ACCEL_t *dev,
        SPI_HandleTypeDef *hspi,
        GPIO_TypeDef *cs_port,
        uint16_t cs_pin)
{
    HAL_StatusTypeDef status;

    if ((dev == NULL) ||
        (hspi == NULL) ||
        (cs_port == NULL))
    {
        return HAL_ERROR;
    }

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->who_am_i = 0;

    CS_High(dev);

    HAL_Delay(200);

    status = SelectBank(dev, ICM20948_BANK_0);
    if (status != HAL_OK) return status;

    status = ReadRegister(
            dev,
            ICM20948_REG_WHO_AM_I,
            &dev->who_am_i);

    if (status != HAL_OK)
    {
        return status;
    }

    if (dev->who_am_i != ICM20948_WHO_AM_I_VALUE)
    {
        return HAL_ERROR;
    }

    /*
     * Host communication is SPI.
     * Disable ICM primary I2C slave interface.
     */
    status = WriteRegister(
            dev,
            ICM20948_REG_USER_CTRL,
            0x10);

    if (status != HAL_OK) return status;

    /*
     * Wake device, CLKSEL = 1.
     */
    status = WriteRegister(
            dev,
            ICM20948_REG_PWR_MGMT_1,
            0x01);

    if (status != HAL_OK) return status;

    HAL_Delay(50);

    /*
     * Accelerometer-only test:
     *
     * DISABLE_ACCEL = 000
     * DISABLE_GYRO  = 111
     *
     * PWR_MGMT_2 = 0x07
     */
    status = WriteRegister(
            dev,
            ICM20948_REG_PWR_MGMT_2,
            0x07);

    if (status != HAL_OK) return status;

    /*
     * Datasheet notes a short delay after disabling gyro
     * before some SPI accesses. 1 ms is safely above it.
     */
    HAL_Delay(1);

    status = SelectBank(dev, ICM20948_BANK_2);
    if (status != HAL_OK) return status;

    /*
     * Sample divider = 10.
     * Same setting we used in the earlier working IMU test.
     */
    status = WriteRegister(
            dev,
            ICM20948_B2_ACCEL_SMPLRT_DIV_1,
            0x00);

    if (status != HAL_OK) return status;

    status = WriteRegister(
            dev,
            ICM20948_B2_ACCEL_SMPLRT_DIV_2,
            10);

    if (status != HAL_OK) return status;

    /*
     * +/-2g, DLPF enabled.
     */
    status = WriteRegister(
            dev,
            ICM20948_B2_ACCEL_CONFIG,
            0x11);

    if (status != HAL_OK) return status;

    HAL_Delay(20);

    return SelectBank(dev, ICM20948_BANK_0);
}

/* ------------------------------------------------------- */

HAL_StatusTypeDef ICM20948_ACCEL_ReadRaw(
        ICM20948_ACCEL_t *dev,
        int16_t *ax,
        int16_t *ay,
        int16_t *az)
{
    uint8_t data[6];

    if ((dev == NULL) ||
        (ax == NULL) ||
        (ay == NULL) ||
        (az == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status =
            SelectBank(dev, ICM20948_BANK_0);

    if (status != HAL_OK)
    {
        return status;
    }

    status = ReadRegisters(
            dev,
            ICM20948_REG_ACCEL_XOUT_H,
            data,
            6);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Accelerometer registers:
     * high byte first, low byte second.
     */
    *ax = (int16_t)(
            ((uint16_t)data[0] << 8) |
            data[1]);

    *ay = (int16_t)(
            ((uint16_t)data[2] << 8) |
            data[3]);

    *az = (int16_t)(
            ((uint16_t)data[4] << 8) |
            data[5]);

    return HAL_OK;
}

/* ------------------------------------------------------- */

HAL_StatusTypeDef ICM20948_ACCEL_AverageRaw(
        ICM20948_ACCEL_t *dev,
        uint16_t sample_count,
        uint32_t sample_delay_ms,
        float *avg_x,
        float *avg_y,
        float *avg_z)
{
    if ((dev == NULL) ||
        (sample_count == 0) ||
        (avg_x == NULL) ||
        (avg_y == NULL) ||
        (avg_z == NULL))
    {
        return HAL_ERROR;
    }

    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t sum_z = 0;

    uint16_t valid_samples = 0;

    while (valid_samples < sample_count)
    {
        int16_t ax;
        int16_t ay;
        int16_t az;

        if (ICM20948_ACCEL_ReadRaw(
                dev,
                &ax,
                &ay,
                &az) == HAL_OK)
        {
            sum_x += ax;
            sum_y += ay;
            sum_z += az;

            valid_samples++;
        }

        HAL_Delay(sample_delay_ms);
    }

    *avg_x = (float)sum_x / (float)valid_samples;
    *avg_y = (float)sum_y / (float)valid_samples;
    *avg_z = (float)sum_z / (float)valid_samples;

    return HAL_OK;
}

/* ------------------------------------------------------- */

HAL_StatusTypeDef ICM20948_ACCEL_ComputeCalibration(
        ICM20948_ACCEL_Calibration_t *cal)
{
    if (cal == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * Target-axis values from the six positions.
     */
    float x_pos = cal->x_pos_x;
    float x_neg = cal->x_neg_x;

    float y_pos = cal->y_pos_y;
    float y_neg = cal->y_neg_y;

    float z_pos = cal->z_pos_z;
    float z_neg = cal->z_neg_z;

    /*
     * Sanity check:
     * positive orientation must read above negative orientation.
     */
    if ((x_pos <= x_neg) ||
        (y_pos <= y_neg) ||
        (z_pos <= z_neg))
    {
        cal->valid = 0;
        return HAL_ERROR;
    }

    cal->offset_x_raw =
            (x_pos + x_neg) * 0.5f;

    cal->offset_y_raw =
            (y_pos + y_neg) * 0.5f;

    cal->offset_z_raw =
            (z_pos + z_neg) * 0.5f;

    cal->measured_lsb_per_g_x =
            (x_pos - x_neg) * 0.5f;

    cal->measured_lsb_per_g_y =
            (y_pos - y_neg) * 0.5f;

    cal->measured_lsb_per_g_z =
            (z_pos - z_neg) * 0.5f;

    /*
     * If the board was not placed close to +/-1g positions,
     * these ranges become implausibly small.
     *
     * We keep the check loose so it does not reject a
     * slightly imperfect manual placement.
     */
    if ((cal->measured_lsb_per_g_x < 8000.0f) ||
        (cal->measured_lsb_per_g_y < 8000.0f) ||
        (cal->measured_lsb_per_g_z < 8000.0f))
    {
        cal->valid = 0;
        return HAL_ERROR;
    }

    cal->scale_x =
            ICM20948_ACCEL_NOMINAL_LSB_PER_G /
            cal->measured_lsb_per_g_x;

    cal->scale_y =
            ICM20948_ACCEL_NOMINAL_LSB_PER_G /
            cal->measured_lsb_per_g_y;

    cal->scale_z =
            ICM20948_ACCEL_NOMINAL_LSB_PER_G /
            cal->measured_lsb_per_g_z;

    cal->valid = 1;

    return HAL_OK;
}

/* ------------------------------------------------------- */

HAL_StatusTypeDef ICM20948_ACCEL_ReadCalibrated(
        ICM20948_ACCEL_t *dev,
        const ICM20948_ACCEL_Calibration_t *cal,
        ICM20948_ACCEL_Data_t *data)
{
    if ((dev == NULL) ||
        (cal == NULL) ||
        (data == NULL) ||
        (!cal->valid))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status =
            ICM20948_ACCEL_ReadRaw(
                    dev,
                    &data->x_raw,
                    &data->y_raw,
                    &data->z_raw);

    if (status != HAL_OK)
    {
        return status;
    }

    data->x_corrected_raw =
            ((float)data->x_raw -
             cal->offset_x_raw) *
            cal->scale_x;

    data->y_corrected_raw =
            ((float)data->y_raw -
             cal->offset_y_raw) *
            cal->scale_y;

    data->z_corrected_raw =
            ((float)data->z_raw -
             cal->offset_z_raw) *
            cal->scale_z;

    data->x_g =
            data->x_corrected_raw /
            ICM20948_ACCEL_NOMINAL_LSB_PER_G;

    data->y_g =
            data->y_corrected_raw /
            ICM20948_ACCEL_NOMINAL_LSB_PER_G;

    data->z_g =
            data->z_corrected_raw /
            ICM20948_ACCEL_NOMINAL_LSB_PER_G;

    data->x_ms2 =
            data->x_g *
            STANDARD_GRAVITY_MS2;

    data->y_ms2 =
            data->y_g *
            STANDARD_GRAVITY_MS2;

    data->z_ms2 =
            data->z_g *
            STANDARD_GRAVITY_MS2;

    return HAL_OK;
}
