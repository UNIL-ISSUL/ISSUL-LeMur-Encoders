/**
 * @file Communication.h
 * @brief Manages Bluetooth and Modbus communication.
 *
 * This class abstracts the functionalities for sending data packets over
 * Bluetooth and handling Modbus RTU slave operations.
 */
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <BluetoothSerial.h>
#include <ModbusRTUSlave.h>
#include "../models/packet.h"
#include "../config/config.h"

class Communication {
public:
    /**
     * @brief Construct a new Communication object.
     */
    Communication();

    /**
     * @brief Initializes Bluetooth and Modbus services.
     */
    void begin();

    /**
     * @brief Sends a data packet over Bluetooth.
     * @param packet The packet to be sent.
     */
    void sendPacket(const Packet& packet);

    /**
     * @brief Updates the Modbus slave to process incoming requests.
     */
    void updateModbus();

    /**
     * @brief Checks if the system is in 'steps' mode based on a Modbus coil.
     * @return True if in steps mode, false otherwise (belt mode).
     */
    bool isStepsMode() const;

    /**
     * @brief Sets the state of the encoder feedback Modbus coil.
     * @param enabled The state to set the coil to.
     */
    void setEncoderFeedback(bool enabled);

    /**
     * @brief Sets the speed value in the Modbus holding register.
     * @param speed The speed value to be set.
     */
    void setFeedbackSpeed(uint16_t speed);

private:
    BluetoothSerial _serialBT;  ///< The Bluetooth Serial object.
    ModbusRTUSlave _slave;      ///< The Modbus RTU slave object.
    bool _coils[NUM_COILS];     ///< Array to hold the state of Modbus coils.
    uint16_t _registers[NUM_HOLDING_REGISTERS]; ///< Array for Modbus holding registers.
};

#endif // COMMUNICATION_H