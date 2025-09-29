/**
 * @file Sensor.h
 * @brief Manages sensor reading, filtering, and calculations.
 *
 * This class handles reading from the encoders and ADC via the HardwareManager,
 * applying filtering (median and IIR), and computing the final speed and
 * inclination values.
 */
#ifndef SENSOR_H
#define SENSOR_H

#include <RunningMedian.h>
#include "../filter.h"
#include "HardwareManager.h"
#include "../config/config.h"

class Sensor {
public:
    /**
     * @brief Construct a new Sensor object.
     * @param hardwareManager A reference to the HardwareManager instance.
     */
    Sensor(HardwareManager& hardwareManager);

    /**
     * @brief Initializes the filters and sets the initial time.
     */
    void begin();

    /**
     * @brief Reads sensors, computes, and filters the speed and inclination.
     * This method should be called periodically.
     */
    void update();

    /**
     * @brief Gets the filtered belt speed.
     * @return The current belt speed in mm/s.
     */
    float getBeltSpeed() const;

    /**
     * @brief Gets the filtered steps speed.
     * @return The current steps speed in mm/s.
     */
    float getStepsSpeed() const;

    /**
     * @brief Gets the filtered inclination.
     * @return The current inclination in degrees.
     */
    float getInclination() const;

private:
    /**
     * @brief Computes speed in pulses per second from encoder deltas.
     * @param delta_count The change in encoder counts.
     * @param delta_time The change in time in microseconds.
     * @return The computed speed in pulses/sec.
     */
    float computeSpeed(int32_t delta_count, uint32_t delta_time);

    HardwareManager& _hardwareManager; ///< Reference to the hardware manager.

    IIRFilter _beltSpeedFilter;      ///< IIR filter for the belt speed.
    IIRFilter _stepsSpeedFilter;     ///< IIR filter for the steps speed.

    RunningMedian _beltSpeedMedian;    ///< Median filter for the belt speed.
    RunningMedian _stepsSpeedMedian;   ///< Median filter for the steps speed.
    RunningMedian _inclinationMedian;  ///< Median filter for the inclination.

    uint32_t _lastBeltEncoderCount;  ///< Last read count of the belt encoder.
    uint32_t _lastStepsEncoderCount; ///< Last read count of the steps encoder.
    uint32_t _lastTimeUs;            ///< The timestamp of the last update.

    float _beltSpeed;    ///< The final filtered belt speed in mm/s.
    float _stepsSpeed;   ///< The final filtered steps speed in mm/s.
    float _inclination;  ///< The final filtered inclination in degrees.
};

#endif // SENSOR_H