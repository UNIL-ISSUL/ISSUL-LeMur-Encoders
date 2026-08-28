#include "UNIT_EXT_ENCODER.h"

UNIT_EXT_ENCODER::UNIT_EXT_ENCODER() {
    _addr = UNIT_EXT_ENCODER_ADDR;
    _wire = &Wire;
    _sda = 21;
    _scl = 22;
    _speed = 100000L;
}

void UNIT_EXT_ENCODER::writeBytes(uint8_t addr, uint8_t reg, const uint8_t *buffer, uint8_t length) {
    _wire->beginTransmission(addr);
    _wire->write(reg);
    for (uint8_t i = 0; i < length; i++) {
        _wire->write(buffer[i]);
    }
    _wire->endTransmission();
}

void UNIT_EXT_ENCODER::readBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length) {
    _wire->beginTransmission(addr);
    _wire->write(reg);
    _wire->endTransmission(true);
    uint8_t read_bytes = _wire->requestFrom((int)addr, (int)length);
    for (uint8_t i = 0; i < read_bytes && i < length; i++) {
        buffer[i] = _wire->read();
    }
}

bool UNIT_EXT_ENCODER::begin(TwoWire *wire, uint8_t addr, uint8_t sda, uint8_t scl, uint32_t speed) {
    _wire = wire ? wire : &Wire;
    _addr = addr;
    _sda = sda;
    _scl = scl;
    _speed = speed;
    _wire->begin((int)_sda, (int)_scl, _speed);
    delay(10);
    _wire->beginTransmission(_addr);
    return (_wire->endTransmission() == 0);
}

bool UNIT_EXT_ENCODER::init(TwoWire *wire, uint8_t addr, uint8_t sda, uint8_t scl, uint32_t speed) {
    _wire = wire ? wire : &Wire;
    _addr = addr;
    _sda = sda;
    _scl = scl;
    _speed = speed;
    return true; // Fixed missing return statement
}

uint32_t UNIT_EXT_ENCODER::getEncoderValue(void) {
    uint8_t data[4] = {0};
    readBytes(_addr, UNIT_EXT_ENCODER_ENCODER_REG, data, 4);
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

uint32_t UNIT_EXT_ENCODER::getZeroPulseValue(void) {
    uint8_t data[4] = {0};
    readBytes(_addr, UNIT_EXT_ENCODER_ZERO_PULSE_VALUE_REG, data, 4);
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

void UNIT_EXT_ENCODER::setZeroPulseValue(uint32_t value) {
    uint8_t data[4];
    data[0] = (value & 0xff);
    data[1] = ((value >> 8) & 0xff);
    data[2] = ((value >> 16) & 0xff);
    data[3] = ((value >> 24) & 0xff);
    writeBytes(_addr, UNIT_EXT_ENCODER_ZERO_PULSE_VALUE_REG, data, 4);
}

uint32_t UNIT_EXT_ENCODER::getMeterValue(void) {
    uint8_t data[4] = {0};
    readBytes(_addr, UNIT_EXT_ENCODER_METER_REG, data, 4);
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

void UNIT_EXT_ENCODER::getMeterString(char *str) {
    if (!str) return;
    uint8_t read_buf[9] = {0};
    readBytes(_addr, UNIT_EXT_ENCODER_METER_STRING_REG, read_buf, 9);
    memcpy(str, read_buf, sizeof(read_buf));
}

void UNIT_EXT_ENCODER::resetEncoder(void) {
    uint8_t data[1] = {1};
    writeBytes(_addr, UNIT_EXT_ENCODER_RESET_REG, data, 1);
}

void UNIT_EXT_ENCODER::setPerimeter(uint32_t perimeter) {
    uint8_t data[4];
    data[0] = (perimeter & 0xff);
    data[1] = ((perimeter >> 8) & 0xff);
    data[2] = ((perimeter >> 16) & 0xff);
    data[3] = ((perimeter >> 24) & 0xff);
    writeBytes(_addr, UNIT_EXT_ENCODER_PERIMETER_REG, data, 4);
}

void UNIT_EXT_ENCODER::setZeroMode(uint8_t mode) {
    uint8_t data[1] = {mode};
    writeBytes(_addr, UNIT_EXT_ENCODER_ZERO_MODE_REG, data, 1);
}

uint32_t UNIT_EXT_ENCODER::getPerimeter(void) {
    uint8_t data[4] = {0};
    readBytes(_addr, UNIT_EXT_ENCODER_PERIMETER_REG, data, 4);
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

void UNIT_EXT_ENCODER::setPulse(uint32_t pulse) {
    uint8_t data[4];
    data[0] = (pulse & 0xff);
    data[1] = ((pulse >> 8) & 0xff);
    data[2] = ((pulse >> 16) & 0xff);
    data[3] = ((pulse >> 24) & 0xff);
    writeBytes(_addr, UNIT_EXT_ENCODER_PULSE_REG, data, 4);
}

uint32_t UNIT_EXT_ENCODER::getPulse(void) {
    uint8_t data[4] = {0};
    readBytes(_addr, UNIT_EXT_ENCODER_PULSE_REG, data, 4);
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

uint8_t UNIT_EXT_ENCODER::setI2CAddress(uint8_t addr) {
    _wire->beginTransmission(_addr);
    _wire->write(I2C_ADDRESS_REG);
    _wire->write(addr);
    _wire->endTransmission();
    _addr = addr;
    return _addr;
}

uint8_t UNIT_EXT_ENCODER::getI2CAddress(void) {
    _wire->beginTransmission(_addr);
    _wire->write(I2C_ADDRESS_REG);
    _wire->endTransmission();

    uint8_t reg_value = 0;
    if (_wire->requestFrom((int)_addr, 1)) {
        reg_value = _wire->read();
    }
    return reg_value;
}

uint8_t UNIT_EXT_ENCODER::getFirmwareVersion(void) {
    _wire->beginTransmission(_addr);
    _wire->write(FIRMWARE_VERSION_REG);
    _wire->endTransmission();

    uint8_t reg_value = 0;
    if (_wire->requestFrom((int)_addr, 1)) {
        reg_value = _wire->read();
    }
    return reg_value;
}
