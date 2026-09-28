/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ICM-20948 Accelerometer 6-Position Calibration - Termite
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "icm20948_accel_cal.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

ICM20948_ACCEL_t accel;
ICM20948_ACCEL_Calibration_t accelCal;
ICM20948_ACCEL_Data_t accelData;

char uartBuffer[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

static void UART_Send(const char *text);

static void FormatFloat3(
        char *buffer,
        size_t buffer_size,
        float value);

static HAL_StatusTypeDef CapturePose(
        const char *pose_name,
        const char *instruction,
        float *avg_x,
        float *avg_y,
        float *avg_z);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void UART_Send(const char *text)
{
    HAL_UART_Transmit(
            &huart2,
            (uint8_t *)text,
            strlen(text),
            HAL_MAX_DELAY);
}

static void FormatFloat3(
        char *buffer,
        size_t buffer_size,
        float value)
{
    int32_t scaled;

    if (value >= 0.0f)
    {
        scaled = (int32_t)(value * 1000.0f + 0.5f);
    }
    else
    {
        scaled = (int32_t)(value * 1000.0f - 0.5f);
    }

    int32_t whole =
            scaled / 1000;

    int32_t fraction =
            scaled % 1000;

    if (fraction < 0)
    {
        fraction = -fraction;
    }

    if ((scaled < 0) && (whole == 0))
    {
        snprintf(
                buffer,
                buffer_size,
                "-0.%03ld",
                (long)fraction);
    }
    else
    {
        snprintf(
                buffer,
                buffer_size,
                "%ld.%03ld",
                (long)whole,
                (long)fraction);
    }
}

static HAL_StatusTypeDef CapturePose(
        const char *pose_name,
        const char *instruction,
        float *avg_x,
        float *avg_y,
        float *avg_z)
{
    UART_Send("\r\n");
    UART_Send("--------------------------------------------\r\n");

    snprintf(
            uartBuffer,
            sizeof(uartBuffer),
            "%s\r\n",
            pose_name);

    UART_Send(uartBuffer);
    UART_Send(instruction);
    UART_Send("\r\n");
    UART_Send("You have 5 seconds to place the sensor.\r\n");

    for (int i = 5; i >= 1; i--)
    {
        snprintf(
                uartBuffer,
                sizeof(uartBuffer),
                "Starting measurement in %d...\r\n",
                i);

        UART_Send(uartBuffer);
        HAL_Delay(1000);
    }

    UART_Send("HOLD STILL - SAMPLING FOR 3 SECONDS...\r\n");

    HAL_StatusTypeDef status =
            ICM20948_ACCEL_AverageRaw(
                    &accel,
                    300,
                    10,
                    avg_x,
                    avg_y,
                    avg_z);

    if (status != HAL_OK)
    {
        UART_Send("POSE READ ERROR\r\n");
        return status;
    }

    char x[24], y[24], z[24];

    FormatFloat3(
            x,
            sizeof(x),
            *avg_x);

    FormatFloat3(
            y,
            sizeof(y),
            *avg_y);

    FormatFloat3(
            z,
            sizeof(z),
            *avg_z);

    snprintf(
            uartBuffer,
            sizeof(uartBuffer),
            "POSE SAVED | AVG RAW X:%s Y:%s Z:%s\r\n",
            x,
            y,
            z);

    UART_Send(uartBuffer);

    return HAL_OK;
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();

    /* USER CODE BEGIN 2 */

    UART_Send("\r\n");
    UART_Send("============================================\r\n");
    UART_Send("ICM-20948 ACCELEROMETER CALIBRATION\r\n");
    UART_Send("6-POSITION METHOD\r\n");
    UART_Send("============================================\r\n");

    if (ICM20948_ACCEL_Init(
            &accel,
            &hspi1,
            GPIOA,
            GPIO_PIN_4) != HAL_OK)
    {
        snprintf(
                uartBuffer,
                sizeof(uartBuffer),
                "ACCEL INIT ERROR | WHO_AM_I:0x%02X\r\n",
                accel.who_am_i);

        UART_Send(uartBuffer);

        while (1)
        {
            HAL_Delay(1000);
        }
    }

    snprintf(
            uartBuffer,
            sizeof(uartBuffer),
            "ICM-20948 FOUND : 0x%02X\r\n\r\n",
            accel.who_am_i);

    UART_Send(uartBuffer);

    UART_Send("IMPORTANT:\r\n");
    UART_Send("For each step, place the requested sensor axis\r\n");
    UART_Send("vertically toward the ceiling, then keep it still.\r\n");
    UART_Send("The code will automatically capture all 6 positions.\r\n");

    memset(
            &accelCal,
            0,
            sizeof(accelCal));

    /*
     * 1) +X UP
     */
    if (CapturePose(
            "POSITION 1 / 6 : +X UP",
            "Point the sensor +X axis toward the CEILING.",
            &accelCal.x_pos_x,
            &accelCal.x_pos_y,
            &accelCal.x_pos_z) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * 2) -X UP
     */
    if (CapturePose(
            "POSITION 2 / 6 : -X UP",
            "Point the sensor +X axis toward the FLOOR.",
            &accelCal.x_neg_x,
            &accelCal.x_neg_y,
            &accelCal.x_neg_z) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * 3) +Y UP
     */
    if (CapturePose(
            "POSITION 3 / 6 : +Y UP",
            "Point the sensor +Y axis toward the CEILING.",
            &accelCal.y_pos_x,
            &accelCal.y_pos_y,
            &accelCal.y_pos_z) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * 4) -Y UP
     */
    if (CapturePose(
            "POSITION 4 / 6 : -Y UP",
            "Point the sensor +Y axis toward the FLOOR.",
            &accelCal.y_neg_x,
            &accelCal.y_neg_y,
            &accelCal.y_neg_z) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * 5) +Z UP
     */
    if (CapturePose(
            "POSITION 5 / 6 : +Z UP",
            "Point the sensor +Z axis toward the CEILING.",
            &accelCal.z_pos_x,
            &accelCal.z_pos_y,
            &accelCal.z_pos_z) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * 6) -Z UP
     */
    if (CapturePose(
            "POSITION 6 / 6 : -Z UP",
            "Point the sensor +Z axis toward the FLOOR.",
            &accelCal.z_neg_x,
            &accelCal.z_neg_y,
            &accelCal.z_neg_z) != HAL_OK)
    {
        Error_Handler();
    }

    UART_Send("\r\n");
    UART_Send("============================================\r\n");
    UART_Send("ALL 6 POSITIONS CAPTURED\r\n");
    UART_Send("CALCULATING OFFSET + SCALE...\r\n");
    UART_Send("============================================\r\n");

    if (ICM20948_ACCEL_ComputeCalibration(
            &accelCal) != HAL_OK)
    {
        UART_Send("ACCEL CALIBRATION FAILED\r\n");
        UART_Send("One or more axes did not see a valid +1g / -1g range.\r\n");
        UART_Send("Check axis directions and repeat the test.\r\n");

        while (1)
        {
            HAL_Delay(1000);
        }
    }

    char ox[24], oy[24], oz[24];
    char lx[24], ly[24], lz[24];
    char sx[24], sy[24], sz[24];

    FormatFloat3(
            ox,
            sizeof(ox),
            accelCal.offset_x_raw);

    FormatFloat3(
            oy,
            sizeof(oy),
            accelCal.offset_y_raw);

    FormatFloat3(
            oz,
            sizeof(oz),
            accelCal.offset_z_raw);

    FormatFloat3(
            lx,
            sizeof(lx),
            accelCal.measured_lsb_per_g_x);

    FormatFloat3(
            ly,
            sizeof(ly),
            accelCal.measured_lsb_per_g_y);

    FormatFloat3(
            lz,
            sizeof(lz),
            accelCal.measured_lsb_per_g_z);

    FormatFloat3(
            sx,
            sizeof(sx),
            accelCal.scale_x);

    FormatFloat3(
            sy,
            sizeof(sy),
            accelCal.scale_y);

    FormatFloat3(
            sz,
            sizeof(sz),
            accelCal.scale_z);

    UART_Send("\r\nACCEL CALIBRATION SUCCESS\r\n\r\n");

    snprintf(
            uartBuffer,
            sizeof(uartBuffer),
            "OFFSET RAW [LSB]\r\n"
            "X:%s Y:%s Z:%s\r\n\r\n"
            "MEASURED LSB/g\r\n"
            "X:%s Y:%s Z:%s\r\n\r\n"
            "SCALE CORRECTION\r\n"
            "X:%s Y:%s Z:%s\r\n\r\n",
            ox,
            oy,
            oz,
            lx,
            ly,
            lz,
            sx,
            sy,
            sz);

    UART_Send(uartBuffer);

    UART_Send("NORMAL MODE STARTED\r\n");
    UART_Send("Output: RAW | calibrated g | calibrated m/s^2\r\n\r\n");

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */

    while (1)
    {
        if (ICM20948_ACCEL_ReadCalibrated(
                &accel,
                &accelCal,
                &accelData) == HAL_OK)
        {
            char xg[24], yg[24], zg[24];
            char xms[24], yms[24], zms[24];

            FormatFloat3(
                    xg,
                    sizeof(xg),
                    accelData.x_g);

            FormatFloat3(
                    yg,
                    sizeof(yg),
                    accelData.y_g);

            FormatFloat3(
                    zg,
                    sizeof(zg),
                    accelData.z_g);

            FormatFloat3(
                    xms,
                    sizeof(xms),
                    accelData.x_ms2);

            FormatFloat3(
                    yms,
                    sizeof(yms),
                    accelData.y_ms2);

            FormatFloat3(
                    zms,
                    sizeof(zms),
                    accelData.z_ms2);

            snprintf(
                    uartBuffer,
                    sizeof(uartBuffer),

                    "RAW X:%6d Y:%6d Z:%6d | "
                    "g X:%s Y:%s Z:%s | "
                    "m/s2 X:%s Y:%s Z:%s\r\n",

                    accelData.x_raw,
                    accelData.y_raw,
                    accelData.z_raw,

                    xg,
                    yg,
                    zg,

                    xms,
                    yms,
                    zms);

            UART_Send(uartBuffer);
        }
        else
        {
            UART_Send("ACCEL READ ERROR\r\n");
        }

        HAL_Delay(100);
    }

    /* USER CODE END WHILE */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType =
            RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
            RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
            RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
            RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
            RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLMUL =
            RCC_PLL_MUL4;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
            RCC_CLOCKTYPE_HCLK |
            RCC_CLOCKTYPE_SYSCLK |
            RCC_CLOCKTYPE_PCLK1 |
            RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
            RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
            RCC_SYSCLK_DIV2;

    RCC_ClkInitStruct.APB1CLKDivider =
            RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
            RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief SPI1 Initialization Function
  * @retval None
  */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;

    hspi1.Init.Mode =
            SPI_MODE_MASTER;

    hspi1.Init.Direction =
            SPI_DIRECTION_2LINES;

    hspi1.Init.DataSize =
            SPI_DATASIZE_8BIT;

    hspi1.Init.CLKPolarity =
            SPI_POLARITY_LOW;

    hspi1.Init.CLKPhase =
            SPI_PHASE_1EDGE;

    hspi1.Init.NSS =
            SPI_NSS_SOFT;

    hspi1.Init.BaudRatePrescaler =
            SPI_BAUDRATEPRESCALER_8;

    hspi1.Init.FirstBit =
            SPI_FIRSTBIT_MSB;

    hspi1.Init.TIMode =
            SPI_TIMODE_DISABLE;

    hspi1.Init.CRCCalculation =
            SPI_CRCCALCULATION_DISABLE;

    hspi1.Init.CRCPolynomial = 7;

    hspi1.Init.CRCLength =
            SPI_CRC_LENGTH_DATASIZE;

    hspi1.Init.NSSPMode =
            SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief USART2 Initialization Function
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

    huart2.Init.BaudRate =
            115200;

    huart2.Init.WordLength =
            UART_WORDLENGTH_8B;

    huart2.Init.StopBits =
            UART_STOPBITS_1;

    huart2.Init.Parity =
            UART_PARITY_NONE;

    huart2.Init.Mode =
            UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl =
            UART_HWCONTROL_NONE;

    huart2.Init.OverSampling =
            UART_OVERSAMPLING_16;

    huart2.Init.OneBitSampling =
            UART_ONE_BIT_SAMPLE_DISABLE;

    huart2.AdvancedInit.AdvFeatureInit =
            UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(
            GPIOA,
            GPIO_PIN_4,
            GPIO_PIN_SET);

    GPIO_InitStruct.Pin =
            GPIO_PIN_4;

    GPIO_InitStruct.Mode =
            GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
            GPIO_NOPULL;

    GPIO_InitStruct.Speed =
            GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(
            GPIOA,
            &GPIO_InitStruct);
}


void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT
void assert_failed(
        uint8_t *file,
        uint32_t line)
{
}
#endif
