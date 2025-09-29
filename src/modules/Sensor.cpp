/**
 * @file Sensor.cpp
 * @brief Implementation of the Sensor class.
 */
#include "Sensor.h"

/**
 * @brief Construct a new Sensor:: Sensor object.
 *
 * @param hardwareManager A reference to the hardware manager instance.
 */
Sensor::Sensor(HardwareManager& hardwareManager) :
    _hardwareManager(hardwareManager),
    _beltSpeedMedian(BELT_SPEED_MEDIAN_SIZE),
    _stepsSpeedMedian(STEPS_SPEED_MEDIAN_SIZE),
    _inclinationMedian(INCLINAISON_MEDIAN_SIZE),
    _lastBeltEncoderCount(0),
    _lastStepsEncoderCount(0),
    _lastTimeUs(0),
    _beltSpeed(0),
    _stepsSpeed(0),
    _inclination(0)
{
    // All members are initialized in the initializer list.
}

/**
 * @brief Initializes the sensor module.
 *
 * This sets up the IIR filters with their coefficients from the config file
 * and records the initial time for speed calculations.
 */
void Sensor::begin() {
    _beltSpeedFilter.init(FILTER_ORDER, B_COEFFS, A_COEFFS);
    _stepsSpeedFilter.init(FILTER_ORDER, B_COEFFS, A_COEFFS);
    _lastTimeUs = micros();
}

/**
 * @brief Updates all sensor values.
 *
 * This function reads the latest data from the encoders and ADC,
 * computes the raw speed, applies median and IIR filters, and updates
 * the internal state with the new filtered values.
 */
void Sensor::update() {
    uint32_t beltEncoderCount = _hardwareManager.readEncoder(0);
    uint32_t stepsEncoderCount = _hardwareManager.readEncoder(1);
    uint32_t timeUs = micros();

    // Compute raw speeds in pulses per second
    float rawBeltSpeed = computeSpeed(beltEncoderCount - _lastBeltEncoderCount, timeUs - _lastTimeUs);
    float rawStepsSpeed = computeSpeed(stepsEncoderCount - _lastStepsEncoderCount, timeUs - _lastTimeUs);

    // Store current values for the next delta calculation
    _lastBeltEncoderCount = beltEncoderCount;
    _lastStepsEncoderCount = stepsEncoderCount;
    _lastTimeUs = timeUs;

    // Add raw speeds to the median filter to remove outliers
    _beltSpeedMedian.add(rawBeltSpeed);
    _stepsSpeedMedian.add(rawStepsSpeed);

    // Get the median-filtered speed and convert it from pulses/sec to mm/s
    float medianBeltSpeed = _beltSpeedMedian.getMedian() * ENCODER_PERIMETER[0] / ENCODER_PULSE[0];
    float medianStepsSpeed = _stepsSpeedMedian.getMedian() * ENCODER_PERIMETER[1] / ENCODER_PULSE[1];

    // Apply the IIR low-pass filter to smooth the signal
    _beltSpeed = _beltSpeedFilter.filter(medianBeltSpeed);
    _stepsSpeed = _stepsSpeedFilter.filter(medianStepsSpeed);

    // Read and filter the inclination value
    _inclinationMedian.add(_hardwareManager.readADC());
    float rawInclination = _inclinationMedian.getMedian(); // Value is in mV
    _inclination = rawInclination * 90.0 / 10000.0; // Convert mV to degrees (assuming 10V or 10000mV = 90 degrees)
}

/**
 * @brief Gets the current filtered belt speed.
 * @return The belt speed in mm/s.
 */
float Sensor::getBeltSpeed() const {
    return _beltSpeed;
}

/**
 * @brief Gets the current filtered steps speed.
 * @return The steps speed in mm/s.
 */
float Sensor::getStepsSpeed() const {
    return _stepsSpeed;
}

/**
 * @brief Gets the current filtered inclination.
 * @return The inclination in degrees.
 */
float Sensor::getInclination() const {
    return _inclination;
}

/**
 * @brief Computes speed from encoder counts and time delta.
 * @param delta_count The change in encoder pulses.
 * @param delta_time The time elapsed in microseconds.
 * @return The speed in pulses per second.
 */
float Sensor::computeSpeed(int32_t delta_count, uint32_t delta_time) {
    if (delta_time == 0) {
        return 0.0; // Avoid division by zero
    }
    // Calculate speed in pulses per second
    float speed = 1e6 * fabs(delta_count) / delta_time;
    return speed;
}