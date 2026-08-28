#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <TCA9548.h>
#include <UNIT_EXT_ENCODER.h>
#include <ModbusRTUSlave.h>
#include <M5Atom.h>
#include "Lift.h"
#include "config.h"

// Helper: Output 4-20mA current via DFR0972 DAC on multiplexer
void set_DFR0972_mA(TCA9548 &multiplexer, uint8_t channel, float current_mA, uint16_t dac_4mA, uint16_t dac_20mA);

// Complete hardware initialization and structured diagnostic sequence
bool system_hardware_init(
    TCA9548 &multiplexer,
    UNIT_EXT_ENCODER encoders[],
    size_t num_encoders,
    Lift &lift,
    ModbusRTUSlave &slave,
    bool coils[],
    uint16_t registers[],
    hw_timer_t *&timer_ref,
    void (*timerISR)()
);
