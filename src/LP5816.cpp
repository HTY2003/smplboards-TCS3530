#include "LP5816.h"

LP5816::LP5816(TwoWire& wire) 
        : _wire(wire) { }

bool LP5816::begin(LP5816_max_current max_current, LP5816_channel_en channel_en, LP5816_pwm_config pwm_config) {
    reset();
    delay(5);

    setEnable();
    if (!getEnable())
        return false;

    setMaxCurrent(max_current);
    setChannelEnable(channel_en);
    setPWMConfig(pwm_config);
    setAllDutyCycle(0xFF);
    return true;
}

void LP5816::reset() {
    i2c_write8(I2C_ADDR, reg::REG_RESET_CMD, 0xCC);
}

void LP5816::setMaxCurrent(LP5816_max_current max_current) {
    i2c_write8(I2C_ADDR, reg::REG_DEV_CONFIG0, (uint8_t) max_current);
    updateDevConfig();
}

void LP5816::setChannelEnable(LP5816_channel_en channel_en) {
    uint8_t payload = (channel_en.ch3_en << 3)
            | (channel_en.ch2_en << 2)
            | (channel_en.ch1_en << 1)
            | channel_en.ch0_en;
    i2c_write8(I2C_ADDR, reg::REG_DEV_CONFIG1, payload);
    updateDevConfig();
}

void LP5816::setPWMConfig(LP5816_pwm_config pwm_config) {
    uint8_t payload1 = (pwm_config.fade_time << 4)
            | (pwm_config.ch3_fade_en << 3)
            | (pwm_config.ch2_fade_en << 2)
            | (pwm_config.ch1_fade_en << 1)
            | pwm_config.ch0_fade_en;

    uint8_t payload2 = (pwm_config.ch3_exp_dim_en << 7)
            | (pwm_config.ch2_exp_dim_en << 6)
            | (pwm_config.ch1_exp_dim_en << 5)
            | (pwm_config.ch0_exp_dim_en << 4);
    
    i2c_write8(I2C_ADDR, reg::REG_DEV_CONFIG2, payload1);
    i2c_write8(I2C_ADDR, reg::REG_DEV_CONFIG3, payload2);
    updateDevConfig();
}

void LP5816::setOneCurrentPct(LP5816_channel channel, uint8_t current_pct) {
    switch (channel) {
        case LP5816_CH_0:
            i2c_write8(I2C_ADDR, reg::REG_OUT0_DC, current_pct);
            break;

        case LP5816_CH_1:
            i2c_write8(I2C_ADDR, reg::REG_OUT1_DC, current_pct);
            break;

        case LP5816_CH_2:
            i2c_write8(I2C_ADDR, reg::REG_OUT2_DC, current_pct);
            break;

        case LP5816_CH_3:
            i2c_write8(I2C_ADDR, reg::REG_OUT3_DC, current_pct);
            break;
    }
}

void LP5816::setAllCurrentPct(uint8_t current_pct) {
    uint8_t buffer[NUM_LEDS];
    for (int i = 0; i < NUM_LEDS; i++) {
        buffer[i] = current_pct;
    }
    i2c_write(I2C_ADDR, reg::REG_OUT0_DC, buffer, NUM_LEDS);
}

void LP5816::setOneDutyCycle(LP5816_channel channel, uint8_t duty_cycle) {
    switch (channel) {
        case LP5816_CH_0:
            i2c_write8(I2C_ADDR, reg::REG_OUT0_MANUAL_PWM, duty_cycle);
            break;

        case LP5816_CH_1:
            i2c_write8(I2C_ADDR, reg::REG_OUT1_MANUAL_PWM, duty_cycle);
            break;

        case LP5816_CH_2:
            i2c_write8(I2C_ADDR, reg::REG_OUT2_MANUAL_PWM, duty_cycle);
            break;

        case LP5816_CH_3:
            i2c_write8(I2C_ADDR, reg::REG_OUT3_MANUAL_PWM, duty_cycle);
            break;
    }
}

void LP5816::setAllDutyCycle(uint8_t duty_cycle) {
    uint8_t buffer[NUM_LEDS];
    for (int i = 0; i < NUM_LEDS; i++) {
        buffer[i] = duty_cycle;
    }
    i2c_write(I2C_ADDR, reg::REG_OUT0_MANUAL_PWM, buffer, NUM_LEDS);
}

void LP5816::setOutput(LP5816_output output) {
    i2c_write(I2C_ADDR, reg::REG_OUT0_DC, &output.ch0_current_pct, NUM_LEDS * 2);
}

void LP5816::updateDevConfig() {
    i2c_write8(I2C_ADDR, reg::REG_UPDATE_CMD, 0x55);
}

void LP5816::setEnable() {
    i2c_write8(I2C_ADDR, reg::REG_CHIP_EN, 0x1);
}

bool LP5816::getEnable() {
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

void LP5816::i2c_write(uint8_t addr, uint8_t reg, uint8_t buffer[], uint8_t length) {
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