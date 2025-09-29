/**
 * @file main.cpp
 * @brief Main application file for the LeMur project.
 *
 * This file contains the primary setup and loop functions. It initializes and
 * coordinates the HardwareManager, Sensor, and Communication modules to
 * read sensor data, process it, and send it over Bluetooth and Modbus.
 */
#include <M5Atom.h>
#include "config/config.h"
#include "models/packet.h"
#include "modules/HardwareManager.h"
#include "modules/Sensor.h"
#include "modules/Communication.h"

//==============================================================================
// Global Objects
//==============================================================================
HardwareManager hardwareManager; ///< Manages all hardware interactions.
Sensor sensor(hardwareManager); ///< Processes sensor data to calculate speed and inclination.
Communication communication; ///< Handles Bluetooth and Modbus communications.
Packet packet; ///< The data packet to be sent over Bluetooth.

//==============================================================================
// State Variables
//==============================================================================
/** @brief A volatile flag set by a timer interrupt to signal a sensor read. */
volatile bool flag_read_encoder = false;
/** @brief A flag to toggle the debug output on the serial port. */
bool debug_mode = true;

//==============================================================================
// Interrupt Service Routine
//==============================================================================
/**
 * @brief Timer ISR that sets the flag to read encoders periodically.
 */
void IRAM_ATTR onTimer() {
    flag_read_encoder = true;
}

//==============================================================================
// Helper Functions
//==============================================================================
/**
 * @brief Scales the speed in mm/s to a 15-bit value for the DAC.
 * @param speed_mm_s The speed in mm/s.
 * @return The scaled 15-bit value for the DAC.
 */
uint16_t scale_speed_to_dac(float speed_mm_s) {
    const uint16_t MAX_VOLTAGE = 10000; // 10V in mV
    const float MAX_SPEED_MM_S = 40 * 1e6 / 3600; // 40 km/h in mm/s
    float speed_mv = MAX_VOLTAGE * speed_mm_s / MAX_SPEED_MM_S;
    return (uint16_t)(32767 * speed_mv / MAX_VOLTAGE);
}

//==============================================================================
// Arduino Setup Function
//==============================================================================
void setup() {
    M5.begin(false, true, true); // No Serial, yes I2C, yes Display
    Serial.begin(19200);
    Serial.println("M5Atom Initialized");

    // Initialize all hardware. If it fails, halt execution.
    if (!hardwareManager.begin()) {
        Serial.println("Hardware initialization failed. Halting.");
        M5.dis.drawpix(0, CRGB::Red);
        while (1);
    }

    // Initialize the application modules
    sensor.begin();
    communication.begin();

    // Initialize the hardware timer for periodic sensor reads
    hw_timer_t* timer = timerBegin(3, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, UPDATE_PERIOD_US, true);
    timerAlarmEnable(timer);

    M5.dis.drawpix(0, CRGB::Green);
    Serial.println("Setup Complete");
}

//==============================================================================
// Arduino Loop Function
//==============================================================================
void loop() {
    M5.update(); // Update M5Atom button state

    // If the timer interrupt has fired, process sensor data
    if (flag_read_encoder) {
        sensor.update();

        // Populate the data packet with the latest values
        packet.belt_encoder_speed = (int16_t)round(sensor.getBeltSpeed());
        packet.steps_encoder_speed = (int16_t)round(sensor.getStepsSpeed());
        packet.inclinaison = (int16_t)round(sensor.getInclination() * 100);

        // Send the packet over Bluetooth
        communication.sendPacket(packet);

        // Determine which speed to use for the DAC output based on Modbus coil state
        float dac_speed = communication.isStepsMode() ? sensor.getStepsSpeed() : sensor.getBeltSpeed();
        hardwareManager.setDACOutput(scale_speed_to_dac(dac_speed));

        // Update Modbus registers with the current state
        communication.setEncoderFeedback(dac_speed > 0);
        communication.setFeedbackSpeed((uint16_t)round(dac_speed));

        // Print debug information if enabled
        if (debug_mode) {
            uint32_t now = millis();
            Serial.printf(">belt_speed:%d:%d\n", now, packet.belt_encoder_speed);
            Serial.printf(">steps_speed:%d:%d\n", now, packet.steps_encoder_speed);
        }

        flag_read_encoder = false; // Reset the flag
    } else {
        // If not reading sensors, poll for Modbus updates
        communication.updateModbus();
    }

    // Toggle debug mode and reset encoders when the button is pressed
    if (M5.Btn.wasPressed()) {
        debug_mode = !debug_mode;
        if (debug_mode) {
            M5.dis.drawpix(0, CRGB::Orange);
            Serial.println("Debug mode enabled");
            hardwareManager.resetEncoders();
        } else {
            M5.dis.drawpix(0, CRGB::Green);
        }
    }
}