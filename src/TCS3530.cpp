#include "TCS3530.h"

TCS3530::TCS3530(TwoWire& wire, int int_pin, int vsync_pin) 
        : _wire(wire), _int(int_pin), _vsync(vsync_pin) { }

bool TCS3530::begin() {
    reset();
    delay(5);
    if (getID() != 0x68)
        return false;
    enable();
    setMeasurementTime(179, 4);
    setGain(4);
    setWaitTime(0, TCS3530_NORMAL);
    return true;
}

int TCS3530::getID() {
    return i2c_read8(I2C_ADDR, reg::REG_ID);
}

void TCS3530::reset() {
    i2c_write8(I2C_ADDR, reg::REG_CONTROL_SCL, 0x01);
}

void TCS3530::enable() {
    i2c_write8(I2C_ADDR, reg::REG_OSCEN, 0x01);
    i2c_write8(I2C_ADDR, reg::REG_ENABLE, 0x01);
    i2c_write8(I2C_ADDR, reg::REG_CFG7, 0x80);
    i2c_write8(I2C_ADDR, reg::REG_ENABLE, 0x03);
}

void TCS3530::setWaitTime(uint8_t wait_time, TCS3530_trigger_timing trigger_timing) {
    i2c_write8(I2C_ADDR, reg::REG_WTIME, wait_time);
    i2c_write8(I2C_ADDR, reg::REG_TRIGGER_MODE, trigger_timing);
}

void TCS3530::setMeasurementTime(uint16_t sample_time_step, uint16_t sample_count) {
    uint16_t sample_time_step_cleaned = constrain(sample_time_step, 0, 0x7FF);
    uint8_t payload1 = (sample_time_step_cleaned & 0x7) << 5;
    uint8_t payload2 = sample_time_step_cleaned >> 3;
    i2c_write8(I2C_ADDR, reg::REG_SAMPLE_TIME0, payload1);
    i2c_write8(I2C_ADDR, reg::REG_SAMPLE_TIME1, payload2);

    uint16_t sample_count_cleaned = constrain(sample_count, 0, 0x7FF);
    payload1 = sample_count_cleaned & 0x0FF;
    payload2 = sample_count_cleaned >> 8;
    i2c_write8(I2C_ADDR, reg::REG_ALS_NR_SAMPLES0, payload1);
    i2c_write8(I2C_ADDR, reg::REG_ALS_NR_SAMPLES1, payload2);
}

void TCS3530::setGain(uint8_t gain) {
    uint16_t gain_cleaned = constrain(gain, 0, 0xF);
    uint8_t payload = gain_cleaned << 4;
    i2c_write8(I2C_ADDR, reg::REG_CFG8, payload);
}

void TCS3530::getData(TCS3530_data *data) {
    uint8_t buffer[16];

    i2c_read(I2C_ADDR, reg::REG_ALS_DATA_FIRST, buffer, 16);
    data->x = (buffer[1] << 8) | buffer[0];
    data->y = (buffer[3] << 8) | buffer[2];
    data->z = (buffer[5] << 8) | buffer[4];
    data->ir = (buffer[7] << 8) | buffer[6];
    data->hgl = (buffer[9] << 8) | buffer[8];
    data->hgh = (buffer[11] << 8) | buffer[10];
    data->clear = (buffer[13] << 8) | buffer[12];
    data->flicker = (buffer[15] << 8) | buffer[14];
}

void TCS3530::convertDataToLab(TCS3530_data *data, TCS3530_lab_result *result) {
    float x = (float) data->x / 65536.0;
    float y = (float) data->y / 65536.0;
    float z = (float) data->z / 65536.0;

    float fx = convertToLabHelper(x);
    float fy = convertToLabHelper(y);
    float fz = convertToLabHelper(z);

    result->l = 116 * fx - 16;
    result->a = 500 * (fx - fy);
    result->b = 200 * (fy - fz);
}

float TCS3530::convertToLabHelper(float in) {
    if (in > _delta_cubed)
        return pow(in, 1.0 / 3.0);
    return _const + in / (3.0 * _delta_squared);
}

void TCS3530::i2c_write8(uint8_t addr, uint8_t reg, uint8_t payload) {
    _wire.beginTransmission(addr);
    _wire.write(reg);
    _wire.write(payload);
    _wire.endTransmission();
}

uint8_t TCS3530::i2c_read8(uint8_t addr, uint8_t reg) {
    _wire.beginTransmission(addr);
    _wire.write(reg);
    _wire.endTransmission();
    _wire.requestFrom(addr, 1);
    return _wire.read();
}

void TCS3530::i2c_write(uint8_t addr, uint8_t reg, uint8_t buffer[], uint8_t length) {
    _wire.beginTransmission(addr);
    _wire.write(reg);
    for (int i = 0; i < length; i++) {
        _wire.write(buffer[i]);
    }
    _wire.endTransmission();
}

void TCS3530::i2c_read(uint8_t addr, uint8_t reg, uint8_t buffer[], uint8_t length) {
    _wire.beginTransmission(addr);
    _wire.write(reg);
    _wire.endTransmission();
    _wire.requestFrom(addr, length);
    for (int i = 0; i < length; i++) {
        buffer[i] = _wire.read();
    }
}