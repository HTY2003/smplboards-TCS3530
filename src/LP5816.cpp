#include "LP5816.h"

LP5816::LP5816(TwoWire& wire) 
        : _wire(wire) { }

bool LP5816::begin(const LP5816_config& config) {
    reset();
    delay(5);

    enable();
    if (!checkEnabled())
        return false;

    setConfig(config);
    return true;
}

void LP5816::setConfig(const LP5816_config& config) {
    uint8_t buffer[4];

    buffer[0] = config.max_current;

    buffer[1] = (config.channel_en[3] << 3)
            | (config.channel_en[2] << 2)
            | (config.channel_en[1] << 1)
            | config.channel_en[0];

    buffer[2] = (config.fade_time << 4)
            | (config.fade_en[3] << 3)
            | (config.fade_en[2] << 2)
            | (config.fade_en[1] << 1)
            | config.fade_en[0];

    buffer[3] = (config.exp_dim_en[3] << 7)
            | (config.exp_dim_en[2] << 6)
            | (config.exp_dim_en[1] << 5)
            | (config.exp_dim_en[0] << 4);

    i2c_write(I2C_ADDR, reg::REG_DEV_CONFIG0, buffer, 4);
    i2c_write8(I2C_ADDR, reg::REG_UPDATE_CMD, 0x55);
}

void LP5816::setOutput(const LP5816_output& output) {
    i2c_write(I2C_ADDR, reg::REG_OUT0_DC, output.current, NUM_LEDS * 2);
}

// --- Helper functions start here ---

void LP5816::reset() {
    i2c_write8(I2C_ADDR, reg::REG_RESET_CMD, 0xCC);
}

void LP5816::enable() {
    i2c_write8(I2C_ADDR, reg::REG_CHIP_EN, 0x1);
}

bool LP5816::checkEnabled() {
    return i2c_read8(I2C_ADDR, reg::REG_CHIP_EN) == 0x1;
}

void LP5816::i2c_write8(uint8_t addr, uint8_t reg, uint8_t payload) {
    _wire.beginTransmission(addr);
    _wire.write(reg);
    _wire.write(payload);
    _wire.endTransmission();
}

uint8_t LP5816::i2c_read8(uint8_t addr, uint8_t reg) {
    _wire.beginTransmission(addr);
    _wire.write(reg);
    _wire.endTransmission();
    _wire.requestFrom(addr, 1);
    return _wire.read();
}

void LP5816::i2c_write(uint8_t addr, uint8_t reg, const uint8_t buffer[], uint8_t length) {
    _wire.beginTransmission(addr);
    _wire.write(reg);
    for (int i = 0; i < length; i++) {
        _wire.write(buffer[i]);
    }
    _wire.endTransmission();
}

void LP5816::i2c_read(uint8_t addr, uint8_t reg, uint8_t buffer[], uint8_t length) {
    _wire.beginTransmission(addr);
    _wire.write(reg);
    _wire.endTransmission();
    _wire.requestFrom(addr, length);
    for (int i = 0; i < length; i++) {
        buffer[i] = _wire.read();
    }
}