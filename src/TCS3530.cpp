#include "TCS3530.h"

TCS3530::TCS3530(TwoWire& wire) 
        : _wire(wire) { }

bool TCS3530::begin(const TCS3530_config& config) {
    reset();
    delay(5);
    if (getID() != 0x68)
        return false;
    enablePON();
    setConfig(config);
    clearFIFO();
    return true;
}

void TCS3530::setConfig(const TCS3530_config& config) {
    disableALS();

    i2c_write8(I2C_ADDR, reg::REG_TRIGGER_MODE, config.trigger_mode);
    i2c_write8(I2C_ADDR, reg::REG_WTIME, config.wait_time);

    uint16_t sample_time_cleaned = constrain(config.sample_time, 179, 2047);
    uint16_t sample_count_cleaned = constrain(config.sample_count, 7, 2047);
    uint8_t buffer[4];
    
    buffer[0] = (sample_time_cleaned & 0x7) << 5;
    buffer[1] = sample_time_cleaned >> 3;
    i2c_write(I2C_ADDR, reg::REG_SAMPLE_TIME0, buffer, 2);

    buffer[0] = sample_count_cleaned & 0xFF;
    buffer[1] = sample_count_cleaned >> 8;
    i2c_write(I2C_ADDR, reg::REG_ALS_NR_SAMPLES0, buffer, 2);

    for (int i = 0; i < 4; i++) {
        buffer[i] = (constrain(config.gain[2*i+1], 0, 15) << 4) | (constrain(config.gain[2*i], 0, 15));
    }
    i2c_write(I2C_ADDR, reg::REG_MEAS_SEQR_STEP0_MOD_GAINX_0, buffer, 4);

    uint8_t payload = 0;
    // if (config.fifo_int_en) {
    if (true) {
        setFIFOThreshold(15);
        payload |= 0x04;
    }
    // if (config.als_int_en) {
    //     setALSThreshold(config.als_low_thr, config.als_high_thr);
    //     payload |= 0x08;
    // }
    // if (config.mod_int_en) {
    //     payload |= 0x80;
    // }
    i2c_write8(I2C_ADDR, reg::REG_INTENAB, payload);

    enableALS();
}

void TCS3530::getStatus(TCS3530_status& status) {

    uint8_t buffer[6];
    i2c_read(I2C_ADDR, reg::REG_STATUS, buffer, 6);
    // status.fifo_int = (buffer[0] >> 2) & 0x1;
    // status.als_int = (buffer[0] >> 3) & 0x1;
    // status.mod_int = (buffer[0] >> 7) & 0x1;
    // status.mod_digital_sat_any = (buffer[2] >> 4) & 0x1;
    status.mod_analog_sat_any = (buffer[2] >> 0) & 0x1;
    // status.als_int_low = (buffer[2] >> 4) & 0x1;
    // status.als_int_high = (buffer[2] >> 5) & 0x1;
    for (int i = 0; i < 8; i++) {
        status.mod_analog_sat[i] = (buffer[5] >> i) & 0x1;
    }
    i2c_read(I2C_ADDR, reg::REG_FIFO_STATUS1, buffer, 1);
    status.fifo_overflow = buffer[0] >> 7;
}

// void TCS3530::clearInterrupts() {
//     i2c_write8(I2C_ADDR, reg::REG_STATUS, 0b10001100);
// }

bool TCS3530::updateRawData(TCS3530_raw_data& raw) {
    uint8_t buffer[16];

    i2c_read(I2C_ADDR, reg::REG_FIFO_STATUS0, buffer, 2);
    uint16_t fifo_lvl = (buffer[0] << 3) | (buffer[1] & 0b111);

    if (fifo_lvl >= 16) {
        i2c_read(I2C_ADDR, reg::REG_FIFO_DATA, buffer, 16);
        raw.data[TCS3530_X] = (buffer[1] << 8) | buffer[0];
        raw.data[TCS3530_Y] = (buffer[3] << 8) | buffer[2];
        raw.data[TCS3530_Z] = (buffer[5] << 8) | buffer[4];
        raw.data[TCS3530_IR] = (buffer[7] << 8) | buffer[6];
        raw.data[TCS3530_HGL] = (buffer[9] << 8) | buffer[8];
        raw.data[TCS3530_HGH] = (buffer[11] << 8) | buffer[10];
        raw.data[TCS3530_CLEAR] = (buffer[13] << 8) | buffer[12];
        raw.data[TCS3530_FLICKER] = (buffer[15] << 8) | buffer[14];
        clearFIFO();
        return true;
    }

    return false;
}

void TCS3530::convertRawToNorm(const TCS3530_raw_data& raw, TCS3530_norm_data& result) {
    for (int i = 0; i < 8; i++) {
        result.data[i] = (float) (raw.data[i] - result.low[i]) / (float) (result.high[i] - result.low[i]);
        result.data[i] = constrain(result.data[i], 0, 1);
    }
}

void TCS3530::convertNormToLab(const TCS3530_norm_data& norm, TCS3530_lab_data& result) {
    float fx = convertToLabHelper(norm.data[TCS3530_X]);
    float fy = convertToLabHelper(norm.data[TCS3530_Y]);
    float fz = convertToLabHelper(norm.data[TCS3530_Z]);

    result.data[TCS3530_L] = 116.0 * fx - 16.0;
    result.data[TCS3530_A] = 500.0 * (fx - fy);
    result.data[TCS3530_B] = 200.0 * (fy - fz);
}

// --- Helper functions start here ---

void TCS3530::reset() {
    i2c_write8(I2C_ADDR, reg::REG_CONTROL_SCL, 0x01);
}

void TCS3530::enablePON() {
    i2c_write8(I2C_ADDR, reg::REG_OSCEN, 0x01);
    i2c_write8(I2C_ADDR, reg::REG_ENABLE, 0x01);
    i2c_write8(I2C_ADDR, reg::REG_CFG7, 0x80);              // Enable coherence buffering
    i2c_write8(I2C_ADDR, reg::REG_VSYNC_GPIO_INT, 0x06);    // Set VSYNC/GPIO pin to input
}

void TCS3530::enableALS() {
    i2c_write8(I2C_ADDR, reg::REG_ENABLE, 0x03);
}

void TCS3530::disableALS() {
    i2c_write8(I2C_ADDR, reg::REG_ENABLE, 0x01);
}

int TCS3530::getID() {
    return i2c_read8(I2C_ADDR, reg::REG_ID);
}

void TCS3530::setFIFOThreshold(uint16_t fifo_thr) {
    uint16_t fifo_thr_cleaned = constrain(fifo_thr, 0, 2047);
    uint8_t payload1 = fifo_thr_cleaned >> 3;
    uint8_t payload2 = fifo_thr_cleaned & 0b111;
    
    i2c_write8(I2C_ADDR, reg::REG_FIFO_THR, payload1);
    i2c_write8(I2C_ADDR, reg::REG_CFG2, payload2);
}

// void TCS3530::setALSThreshold(uint16_t als_low_thr, uint16_t als_high_thr) {
//     uint8_t buffer[6];
//     buffer[0] = als_low_thr & 0xFF;
//     buffer[1] = als_low_thr >> 8;
//     buffer[2] = 0;
//     buffer[3] = als_low_thr & 0xFF;
//     buffer[4] = als_low_thr >> 8;
//     buffer[5] = 0;

//     i2c_write(I2C_ADDR, reg::REG_AILT0, buffer, 6);
// }

void TCS3530::clearFIFO() {
    i2c_write8(I2C_ADDR, reg::REG_CONTROL, 0x02);
}

float TCS3530::convertToLabHelper(float in) {
    if (in > _delta_cubed) {
        return pow(in, 1.0 / 3.0);
    }
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

void TCS3530::i2c_write(uint8_t addr, uint8_t reg, const uint8_t buffer[], uint8_t length) {
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