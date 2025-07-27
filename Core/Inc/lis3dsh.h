/**
 * @file lis3dsh.h
 * @brief Header file for LIS3DSH accelerometer driver using STM32 HAL.
 *
 * Provides definitions, constants, data structures, and function declarations
 * for interacting with the LIS3DSH 3-axis accelerometer sensor over SPI.
 *
 * @date April 25, 2025
 * @author Adesh Sudhir
 */

#ifndef INC_LIS3DSH_H_
#define INC_LIS3DSH_H_

#include "stm32f4xx_hal.h"

/** @defgroup LIS3DSH_Register_Addresses Register Addresses
 * @{
 */
#define LIS3DSH_WHO_AM_I_ADDR    0x0F    /**< WHO_AM_I register address */
#define LIS3DSH_CTRL_REG4        0x20    /**< Control Register 4 address */
#define LIS3DSH_CTRL_REG5        0x24    /**< Control Register 5 address */
#define LIS3DSH_X_AXIS_L_REG     0x28    /**< X-axis output low register address */
/** @} */

/** @defgroup LIS3DSH_Constants Constants
 * @{
 */
#define LIS3DSH_WHO_AM_I_VAL     0x3F        /**< Expected WHO_AM_I value */
#define LIS3DSH_SENSITIVITY_2G   (0.000006f) /**< Sensitivity factor for ±2g full scale */
#define LIS3DSH_MAX_RAW_2G       (33333)     /**< Maximum raw output for ±2g range */

// Can be modified
#define ALPHA                    0.1f        /**< Smoothing factor for low-pass filter */

/**
 * @brief Struct to hold raw and normalized axis data from LIS3DSH.
 */
typedef struct
{
    int16_t x_raw;     /**< Raw X-axis data */
    int16_t y_raw;     /**< Raw Y-axis data */
    int16_t z_raw;     /**< Raw Z-axis data */

    float x_norm;      /**< Normalized X-axis data */
    float y_norm;      /**< Normalized Y-axis data */
    float z_norm;      /**< Normalized Z-axis data */

} LIS3DSH_Axes_t;

/**
 * @brief Enumeration for LIS3DSH full-scale selection.
 */
typedef enum
{
    LIS3DSH_SCALE_2G  = 0x00, /**< ±2g full scale */
    LIS3DSH_SCALE_4G  = 0x08, /**< ±4g full scale */
    LIS3DSH_SCALE_6G  = 0x10, /**< ±6g full scale */
    LIS3DSH_SCALE_8G  = 0x18, /**< ±8g full scale */
    LIS3DSH_SCALE_16G = 0x20  /**< ±16g full scale */
} LIS3DSH_Scale_t;

/**
 * @brief Initializes the LIS3DSH accelerometer.
 *
 * @param hspi Pointer to the SPI handle used for communication.
 * @return HAL status.
 */
HAL_StatusTypeDef LIS3DSH_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief Reads raw and normalized acceleration data from the sensor.
 *
 * @param axes Pointer to structure to hold X, Y, and Z axis values.
 * @return HAL status.
 */
HAL_StatusTypeDef LIS3DSH_ReadXYZ(LIS3DSH_Axes_t *axes);

/**
 * @brief Sets the full-scale range of the accelerometer.
 *
 * @param scale Desired scale from LIS3DSH_Scale_t enum.
 * @return HAL status.
 */
HAL_StatusTypeDef LIS3DSH_SetScale(LIS3DSH_Scale_t scale);

/**
 * @brief Reads the device ID from the WHO_AM_I register.
 *
 * @return The WHO_AM_I register value.
 */
uint8_t LIS3DSH_ReadID(void);

/**
 * @brief Calibrates the sensor (e.g., to remove offset).
 */
void LIS3DSH_Calibrate(void);

#endif /* INC_LIS3DSH_H_ */
