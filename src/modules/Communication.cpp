/**
 * @file Communication.cpp
 * @brief Implementation of the Communication class.
 */
#include "Communication.h"
#include <Arduino.h> // For Serial2

/**
 * @brief Construct a new Communication:: Communication object.
 *
 * Initializes the Modbus slave and sets the default values for coils and registers.
 */
Communication::Communication() :
    _slave(Serial2, 3) // Assuming Serial2 and slave ID 3
{
    // Initialize coil and register arrays to a known state (all false/zero)
    for(int i = 0; i < NUM_COILS; i++) _coils[i] = false;
    for(int i = 0; i < NUM_HOLDING_REGISTERS; i++) _registers[i] = 0;
}

/**
 * @brief Initializes the communication services.
 *
 * This method starts the Bluetooth serial communication with the configured device name
 * and sets up the Modbus RTU slave on Serial2 with the specified baud rate and pin configuration.
 */
void Communication::begin() {
    // Initialize Bluetooth Serial as a slave device
    _serialBT.begin(BLUETOOTH_DEVICE_NAME, false);
    Serial.printf("The device \"%s\" started in slave mode \n", BLUETOOTH_DEVICE_NAME);

    // Initialize Modbus RTU slave communication
    Serial2.begin(MODBUS_BAUDRATE, SERIAL_8E1, MODBUS_RX_PIN, MODBUS_TX_PIN);
    _slave.begin(MODBUS_BAUDRATE);
    _slave.setCoils(_coils, NUM_COILS);
    _slave.setHoldingRegisters(_registers, NUM_HOLDING_REGISTERS);
}

/**
 * @brief Sends a data packet via Bluetooth.
 * @param packet The data packet to be sent.
 */
void Communication::sendPacket(const Packet& packet) {
    _serialBT.write(packet.bytes, sizeof(packet.bytes));
}

/**
 * @brief Polls for and handles Modbus requests.
 *
 * This function should be called repeatedly in the main loop to allow the
 * Modbus slave to respond to master requests.
 */
void Communication::updateModbus() {
    _slave.update();
    if (_slave.hasException()) {
        // Silently clear exceptions. For debugging, you could print _slave.getExceptionMessage()
        _slave.clearException();
    }
}

/**
 * @brief Checks the state of the first Modbus coil to determine the operating mode.
 * @return true if the coil is set (steps mode), false otherwise (belt mode).
 */
bool Communication::isStepsMode() const {
    return _coils[0];
}

/**
 * @brief Sets the state of the second Modbus coil, used for encoder feedback status.
 * @param enabled The desired state of the feedback coil.
 */
void Communication::setEncoderFeedback(bool enabled) {
    _coils[1] = enabled;
}

/**
 * @brief Updates the first holding register with the current feedback speed.
 * @param speed The speed value to write to the register.
 */
void Communication::setFeedbackSpeed(uint16_t speed) {
    _registers[0] = speed;
}