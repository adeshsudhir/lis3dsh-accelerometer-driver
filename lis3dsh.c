/*
 * LIS3DSH.c
 *
 *  Created on: Apr 25, 2025
 *      Author: Adesh Sudhir
 */

#include "lis3dsh.h"

/// @brief Pointer to SPI handle used for communication with LIS3DSH
static SPI_HandleTypeDef *lis3dsh_hspi;

/// @brief Pull CS line low (select LIS3DSH)
#define LIS3DSH_CS_low()	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
/// @brief Pull CS line high (deselect LIS3DSH)
#define LIS3DSH_CS_high()	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)

/// @brief Offset calibration values3

static int16_t offset_x = 0;
static int16_t offset_y = 0;
static int16_t offset_z = 0;

/// @brief Filtered axis values using IIR filter
static float x_filtered = 0;
static float y_filtered = 0;
static float z_filtered = 0;

/// @brief Current sensitivity (LSB per g)
static float lis3dsh_sensitivity = 0.00006f;

/// @brief Full scale value in g
static float full_scale_g = 2.0f;

/**
 * @brief Write to a register on the LIS3DSH via SPI
 * @param reg_addr Register address
 * @param data Byte to write
 */
static void LIS3DSH_WriteReg(uint8_t reg_addr, uint8_t data)
{
	uint8_t tx[2] = {(reg_addr & 0x3F), data};

	LIS3DSH_CS_low();
	HAL_SPI_Transmit(lis3dsh_hspi, tx, 2, HAL_MAX_DELAY);
	LIS3DSH_CS_high();
}

/**
 * @brief Read a register from the LIS3DSH via SPI
 * @param reg_addr Register address
 * @return Value read from the register
 */
static uint8_t LIS3DSH_ReadReg(uint8_t reg_addr)
{
	uint8_t tx[2] = {(0x80 | reg_addr), 0x00};
	uint8_t rx[2];

	LIS3DSH_CS_low();
	HAL_SPI_TransmitReceive(lis3dsh_hspi, tx, rx, 2, HAL_MAX_DELAY);
	LIS3DSH_CS_high();
	return rx[1];
}

/**
 * @brief Initialize the LIS3DSH sensor
 * @param hspi Pointer to the SPI handle
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef LIS3DSH_Init(SPI_HandleTypeDef *hspi)
{
	lis3dsh_hspi = hspi;

	if (LIS3DSH_ReadID() != LIS3DSH_WHO_AM_I_VAL) {
		return HAL_ERROR;
	}

	// Enable X, Y, Z axes and set output data rate to 100 Hz (0x67)
	LIS3DSH_WriteReg(LIS3DSH_CTRL_REG4, 0x67);

	// Set full scale to default ±2g
	LIS3DSH_WriteReg(LIS3DSH_CTRL_REG5, 0x00);

	return HAL_OK;
}

/**
 * @brief Read the WHO_AM_I register to identify the device
 * @return WHO_AM_I register value
 */
uint8_t LIS3DSH_ReadID(void)
{
	return LIS3DSH_ReadReg(LIS3DSH_WHO_AM_I_ADDR);
}

/**
 * @brief Read raw and normalized X, Y, Z acceleration values
 * @param axes Pointer to structure to store the data
 * @return HAL_OK if successful
 */
HAL_StatusTypeDef LIS3DSH_ReadXYZ(LIS3DSH_Axes_t *axes)
{
	int16_t x, y, z;

	uint8_t tx[1] = {0};
	uint8_t rx[6];

	tx[0] = 0x80 | LIS3DSH_X_AXIS_L_REG;

	LIS3DSH_CS_low();
	HAL_SPI_Transmit(lis3dsh_hspi, tx, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(lis3dsh_hspi, rx, 6, HAL_MAX_DELAY);
	LIS3DSH_CS_high();

	x = (int16_t)(rx[1] << 8 | rx[0]);
	y = (int16_t)(rx[3] << 8 | rx[2]);
	z = (int16_t)(rx[5] << 8 | rx[4]);

	x -= offset_x;
	y -= offset_y;
	z -= offset_z;

	axes->x_raw = x;
	axes->y_raw = y;
	axes->z_raw = z;

	float max_raw = full_scale_g / lis3dsh_sensitivity;

	axes->x_norm = ((float)x * 100.0f) / max_raw;
	axes->y_norm = ((float)y * 100.0f) / max_raw;
	axes->z_norm = ((float)z * 100.0f) / max_raw;

	// Apply IIR low-pass filter
	x_filtered = ALPHA * axes->x_norm + (1.0f - ALPHA) * x_filtered;
	y_filtered = ALPHA * axes->y_norm + (1.0f - ALPHA) * y_filtered;
	z_filtered = ALPHA * axes->z_norm + (1.0f - ALPHA) * z_filtered;

	axes->x_norm = x_filtered;
	axes->y_norm = y_filtered;
	axes->z_norm = z_filtered;

	return HAL_OK;
}

/**
 * @brief Set the accelerometer full-scale range
 * @param scale Desired scale (e.g., ±2g, ±4g, etc.)
 * @return HAL_OK if valid scale set, HAL_ERROR otherwise
 */
HAL_StatusTypeDef LIS3DSH_SetScale(LIS3DSH_Scale_t scale)
{
	uint8_t reg = LIS3DSH_ReadReg(LIS3DSH_CTRL_REG5);
	reg &= ~(0x38);  // Clear FSCALE bits
	reg |= scale;
	LIS3DSH_WriteReg(LIS3DSH_CTRL_REG5, reg);

	switch (scale) {
		case LIS3DSH_SCALE_2G:
			lis3dsh_sensitivity = 0.00006f;
			full_scale_g = 2.0f;
			break;
		case LIS3DSH_SCALE_4G:
			lis3dsh_sensitivity = 0.00012f;
			full_scale_g = 4.0f;
			break;
		case LIS3DSH_SCALE_6G:
			lis3dsh_sensitivity = 0.00018f;
			full_scale_g = 6.0f;
			break;
		case LIS3DSH_SCALE_8G:
			lis3dsh_sensitivity = 0.00024f;
			full_scale_g = 8.0f;
			break;
		case LIS3DSH_SCALE_16G:
			lis3dsh_sensitivity = 0.00073f;
			full_scale_g = 16.0f;
			break;
		default:
			return HAL_ERROR;
	}

	return HAL_OK;
}

/**
 * @brief Calibrate the sensor by reading and storing offsets
 *        for X, Y, and Z axes
 */
void LIS3DSH_Calibrate(void)
{
	int16_t x, y, z;

	uint8_t tx[1] = {0};
	uint8_t rx[6];

	tx[0] = 0x80 | LIS3DSH_X_AXIS_L_REG;

	LIS3DSH_CS_low();
	HAL_SPI_Transmit(lis3dsh_hspi, tx, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(lis3dsh_hspi, rx, 6, HAL_MAX_DELAY);
	LIS3DSH_CS_high();

	x = ((uint16_t) rx[1] << 8 | rx[0]);
	y = ((uint16_t) rx[3] << 8 | rx[2]);
	z = ((uint16_t) rx[5] << 8 | rx[4]);

	offset_x = x;
	offset_y = y;
	offset_z = z;
}
