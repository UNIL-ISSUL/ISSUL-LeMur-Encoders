/**
 * @file packet.h
 * @brief Defines the data structure for Bluetooth communication.
 */
#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include <Arduino.h> // For byte type

/**
 * @union Packet
 * @brief A union to structure the data packet for Bluetooth transmission.
 *
 * This union allows the data to be accessed either as a raw byte array (`bytes`)
 * for serial transmission, or as a structured set of fields for easy data manipulation.
 */
union Packet {
  /** @brief The raw byte array for transmission. Size: 7 bytes. */
  byte bytes[7];
  struct {
    /** @brief The speed of the belt encoder, in mm/s. */
    int16_t belt_encoder_speed;
    /** @brief The speed of the steps encoder, in mm/s. */
    int16_t steps_encoder_speed;
    /** @brief The inclination, in degrees x100. */
    int16_t inclinaison;
    /** @brief A stop byte to mark the end of the packet. Defaults to line feed (0x0A). */
    byte stopbyte = 0x0A;
  };
};

#endif // PACKET_H