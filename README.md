# LIS3DSH Accelerometer Driver for STM32 HAL

A simple and clean SPI-based driver for the LIS3DSH 3-axis accelerometer, designed for use with STM32 HAL libraries.

## Features

* Initialize the LIS3DSH sensor
* Read the `WHO_AM_I` ID
* Set the full-scale measurement range (±2g, ±4g, etc.)
* Read raw and normalized accelerometer data for X, Y, and Z axes
* Includes a simple IIR low-pass filter for smoothing data

## How to Use

1.  Add `lis3dsh.h` and `lis3dsh.c` to your project.
2.  Include `lis3dsh.h` in your `main.c`.
3.  Initialize your SPI peripheral (e.g., `hspi1`) using STM32CubeMX or manually.
4.  Initialize the driver:

```c
#include "lis3dsh.h"

SPI_HandleTypeDef hspi1; // Your SPI handle
LIS3DSH_Axes_t my_axes;

if (LIS3DSH_Init(&hspi1) == HAL_OK) {
    // Sensor is ready
} else {
    // Check connections
}
```

5.  Read data in your main loop:

```c
while (1) {
    LIS3DSH_ReadXYZ(&my_axes);
    // Use my_axes.x_norm, my_axes.y_norm, etc.
    HAL_Delay(100);
}
```

## Planned Improvements

This section is perfect for listing the changes you plan to make. It shows that you're aware of the code's current state and have a vision for it.

* [ ] **Improve Calibration Logic:** The current calibration is a single-shot "tare" function. It will be updated to average multiple samples and properly account for the 1g gravity offset on the Z-axis.
* [ ] **Refactor SPI Read:** The multi-byte read in `LIS3DSH_ReadXYZ` will be updated to use a single, more robust `HAL_SPI_TransmitReceive()` call.
* [ ] **Change Normalization Units:** The normalized data will be converted to standard gravity units (g's) instead of a percentage of the full-scale range for more intuitive use.

## License

Consider adding an open-source license, like the MIT License. You can add a `LICENSE` file easily on GitHub.
