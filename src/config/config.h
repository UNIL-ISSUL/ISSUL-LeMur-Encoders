/**
 * @file config.h
 * @brief Centralized configuration for the LeMur project.
 *
 * This file contains all the hardware and application-level constants,
 * making it easy to manage and modify the device's behavior without
 * altering the core logic.
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>

//==============================================================================
// Bluetooth Configuration
//==============================================================================
/** @brief The name of the device as it will appear in Bluetooth scans. */
const char* BLUETOOTH_DEVICE_NAME = "LeMur_streaming";


//==============================================================================
// I2C and GPIO Pin Definitions
//==============================================================================
#define I2C_SDA_PIN 25 ///< GPIO pin for I2C data line.
#define I2C_SCL_PIN 21 ///< GPIO pin for I2C clock line.
#define MODBUS_RX_PIN 33 ///< GPIO pin for Modbus (RS485) receive.
#define MODBUS_TX_PIN 23 ///< GPIO pin for Modbus (RS485) transmit.


//==============================================================================
// I2C Device Addresses
//==============================================================================
#define I2C_MULTIPLEXER_ADDR 0x70 ///< I2C address of the TCA9548 multiplexer.
#define ADC_ADDR 0x48             ///< I2C address of the ADC.
#define ENCODER_ADDR 0x59         ///< I2C address of the EXT-Encoder unit.
#define DAC_ADDR 0x58             ///< I2C address of the GP8413 DAC.


//==============================================================================
// Communication Parameters
//==============================================================================
#define I2C_FREQ 400000UL      ///< I2C bus frequency in Hz.
#define MODBUS_BAUDRATE 19200UL ///< Baud rate for Modbus communication.


//==============================================================================
// Application Constants
//==============================================================================
/** @brief The period at which the main loop's sensor reading is triggered, in microseconds. */
#define UPDATE_PERIOD_US 5000
/** @brief The total number of encoder channels being used. */
#define MAX_ENCODER_CHANNELS 2


//==============================================================================
// Filter Configuration
//==============================================================================
/** @brief The order of the IIR Butterworth low-pass filter. */
const int FILTER_ORDER = 4;
/**
 * @brief Numerator (b) coefficients for a 4th-order Butterworth filter.
 * Generated for a sampling frequency (fs) of 200Hz and a cutoff (fc) of 9Hz.
 */
const float B_COEFFS[] = {
    0.0004165992, 0.0016663968, 0.0024995952, 0.0016663968, 0.0004165992,
};
/**
 * @brief Denominator (a) coefficients for a 4th-order Butterworth filter.
 * Generated for a sampling frequency (fs) of 200Hz and a cutoff (fc) of 9Hz.
 */
const float A_COEFFS[] = {
    1.0000000000, -3.1806385489, 3.8611943490, -2.1121553551, 0.4382651423,
};


//==============================================================================
// Encoder Configuration
//==============================================================================
/** @brief The number of pulses per revolution for each encoder. */
const uint32_t ENCODER_PULSE[MAX_ENCODER_CHANNELS] = {10000 * 2, 10000 * 2};
/** @brief The perimeter of the roller/pulley connected to each encoder, in mm. */
const float ENCODER_PERIMETER[MAX_ENCODER_CHANNELS] = {100.0 * M_PI, 126.7 * M_PI};


//==============================================================================
// Modbus Configuration
//==============================================================================
#define NUM_COILS 2 ///< Number of Modbus coils to be used.
#define NUM_HOLDING_REGISTERS 1 ///< Number of Modbus holding registers.


//==============================================================================
// Median Filter Window Sizes
//==============================================================================
#define BELT_SPEED_MEDIAN_SIZE 7   ///< Window size for the belt speed median filter.
#define STEPS_SPEED_MEDIAN_SIZE 7  ///< Window size for the steps speed median filter.
#define INCLINAISON_MEDIAN_SIZE 10 ///< Window size for the inclination median filter.

#endif // CONFIG_H