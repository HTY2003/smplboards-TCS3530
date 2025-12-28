#ifndef TCS3530_H
#define TCS3530_H

#include <Wire.h>

typedef enum {
    TCS3530_OFF,
    TCS3530_NORMAL,
    TCS3530_LONG,
    TCS3530_FAST,
    TCS3530_FAST_LONG,
    TCS3530_VSYNC,
}  TCS3530_trigger_timing;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint16_t ir;
    uint16_t hgl;
    uint16_t hgh;
    uint16_t clear;
    uint16_t flicker;
} TCS3530_data;

typedef struct {
    float l;
    float a;
    float b;
} TCS3530_lab_result;

class TCS3530
{
    private:
        static const uint8_t I2C_ADDR = 0x39;

        /**
         * TCS3530 Register Map
         */
        typedef enum {
            REG_CONTROL_SCL = 0x24,
            REG_MOD_OFFSET0_L = 0x40,
            REG_MOD_OFFSET0_H = 0x41,
            REG_MOD_OFFSET1_L = 0x42,
            REG_MOD_OFFSET1_H = 0x43,
            REG_MOD_OFFSET2_L = 0x44,
            REG_MOD_OFFSET2_H = 0x45,
            REG_MOD_OFFSET3_L = 0x46,
            REG_MOD_OFFSET3_H = 0x47,
            REG_MOD_OFFSET4_L = 0x48,
            REG_MOD_OFFSET4_H = 0x49,
            REG_MOD_OFFSET5_L = 0x4A,
            REG_MOD_OFFSET5_H = 0x4B,
            REG_MOD_OFFSET6_L = 0x4C,
            REG_MOD_OFFSET6_H = 0x4D,
            REG_MOD_OFFSET7_L = 0x4E,
            REG_MOD_OFFSET7_H = 0x4F,
            REG_OSCEN = 0x7F,
            REG_ENABLE = 0x80,
            REG_MEAS_MODE0 = 0x81,
            REG_MEAS_MODE1 = 0x82,
            REG_SAMPLE_TIME0 = 0x83,
            REG_SAMPLE_TIME1 = 0x84,
            REG_SAMPLE_TIME_ALTERNATIVE0 = 0x85,
            REG_SAMPLE_TIME_ALTERNATIVE1 = 0x86,
            REG_ALS_NR_SAMPLES0 = 0x87,
            REG_ALS_NR_SAMPLES1 = 0x88,
            REG_ALS_NR_SAMPLES_ALTERNATIVE0 = 0x89,
            REG_ALS_NR_SAMPLES_ALTERNATIVE1 = 0x8A,
            REG_FD_NR_SAMPLES0 = 0x8B,
            REG_FD_NR_SAMPLES1 = 0x8C,
            REG_FD_NR_SAMPLES_ALTERNATIVE0 = 0x8D,
            REG_FD_NR_SAMPLES_ALTERNATIVE1 = 0x8E,
            REG_WTIME = 0x8F,
            REG_AUX_ID = 0x90,
            REG_REV_ID = 0x91,
            REG_ID = 0x92,
            REG_AILT0 = 0x93,
            REG_AILT1 = 0x94,
            REG_AILT2 = 0x95,
            REG_AIHT0 = 0x96,
            REG_AIHT1 = 0x97,
            REG_AIHT2 = 0x98,
            REG_AGC_NR_SAMPLES_L = 0x99,
            REG_AGC_NR_SAMPLES_H = 0x9A,
            REG_STATUS = 0x9B,
            REG_STATUS2 = 0x9C,
            REG_STATUS3 = 0x9D,
            REG_STATUS4 = 0x9E,
            REG_STATUS5 = 0x9F,
            REG_STATUS6 = 0xA0,
            REG_CFG0 = 0xA1,
            REG_CFG1 = 0xA2,
            REG_CFG2 = 0xA3,
            REG_CFG3 = 0xA4,
            REG_CFG4 = 0xA5,
            REG_CFG5 = 0xA6,
            REG_CFG6 = 0xA7,
            REG_CFG7 = 0xA8,
            REG_CFG8 = 0xA9,
            REG_CFG9 = 0xAA,
            REG_MOD_CHANNEL_CTRL = 0xAB,
            REG_TRIGGER_MODE = 0xAD,
            REG_OSC_TUNE = 0xAE,
            REG_VSYNC_GPIO_INT = 0xB0,
            REG_INTENAB = 0xBA,
            REG_SIEN = 0xBB,
            REG_CONTROL = 0xBC,
            REG_ALS_DATA_STATUS = 0xBD,
            REG_ALS_DATA_FIRST = 0xBE,
            REG_ALS_DATA = 0xBF,
            REG_MEAS_SEQR_STEP0_MOD_GAINX_0 = 0xC0,
            REG_MEAS_SEQR_STEP0_MOD_GAINX_1 = 0xC1,
            REG_MEAS_SEQR_STEP0_MOD_GAINX_2 = 0xC2,
            REG_MEAS_SEQR_STEP0_MOD_GAINX_3 = 0xC3,
            REG_MEAS_SEQR_STEP1_MOD_GAINX_0 = 0xC4,
            REG_MEAS_SEQR_STEP1_MOD_GAINX_1 = 0xC5,
            REG_MEAS_SEQR_STEP1_MOD_GAINX_2 = 0xC6,
            REG_MEAS_SEQR_STEP1_MOD_GAINX_3 = 0xC7,
            REG_MEAS_SEQR_STEP2_MOD_GAINX_0 = 0xC8,
            REG_MEAS_SEQR_STEP2_MOD_GAINX_1 = 0xC9,
            REG_MEAS_SEQR_STEP2_MOD_GAINX_2 = 0xCA,
            REG_MEAS_SEQR_STEP2_MOD_GAINX_3 = 0xCB,
            REG_MEAS_SEQR_STEP3_MOD_GAINX_0 = 0xCC,
            REG_MEAS_SEQR_STEP3_MOD_GAINX_1 = 0xCD,
            REG_MEAS_SEQR_STEP3_MOD_GAINX_2 = 0xCE,
            REG_MEAS_SEQR_STEP3_MOD_GAINX_3 = 0xCF,
            REG_MEAS_SEQR_STEP0_FD = 0xD0,
            REG_MEAS_SEQR_STEP1_FD = 0xD1,
            REG_MEAS_SEQR_STEP2_FD = 0xD2,
            REG_MEAS_SEQR_STEP3_FD = 0xD3,
            REG_MEAS_SEQR_STEP0_RESIDUAL = 0xD4,
            REG_MEAS_SEQR_STEP1_RESIDUAL = 0xD5,
            REG_MEAS_SEQR_STEP2_RESIDUAL = 0xD6,
            REG_MEAS_SEQR_STEP3_RESIDUAL = 0xD7,
            REG_MEAS_SEQR_STEP0_ALS = 0xD8,
            REG_MEAS_SEQR_STEP1_ALS = 0xD9,
            REG_MEAS_SEQR_STEP2_ALS = 0xDA,
            REG_MEAS_SEQR_STEP3_ALS = 0xDB,
            REG_MEAS_SEQR_APERS_AND_VSYNC_WAIT = 0xDC,
            REG_MEAS_SEQR_AGC = 0xDD,
            REG_MEAS_SEQR_SMUX_AND_SAMPLE_TIME = 0xDE,
            REG_MEAS_SEQR_WAIT_AND_TS_ENABLE = 0xDF,
            REG_MOD_CALIB_CFG0 = 0xE0,
            REG_MOD_CALIB_CFG2 = 0xE2,
            REG_MOD_CALIB_CFG3 = 0xE3,
            REG_MOD_COMP_CFG2 = 0xE7,
            REG_MOD_RESIDUAL_CFG0 = 0xE8,
            REG_MOD_RESIDUAL_CFG1 = 0xE9,
            REG_MOD_RESIDUAL_CFG2 = 0xEA,
            REG_VSYNC_DELAY_CFG0 = 0xEB,
            REG_VSYNC_DELAY_CFG1 = 0xEC,
            REG_VSYNC_PERIOD0 = 0xED,
            REG_VSYNC_PERIOD1 = 0xEE,
            REG_VSYNC_PERIOD_TARGET0 = 0xEF,
            REG_VSYNC_PERIOD_TARGET1 = 0xF0,
            REG_VSYNC_CONTROL = 0xF1,
            REG_VSYNC_CFG = 0xF2,
            REG_FIFO_THR = 0xF3,
            REG_MOD_FIFO_DATA_CFG0 = 0xF4,
            REG_MOD_FIFO_DATA_CFG1 = 0xF5,
            REG_MOD_FIFO_DATA_CFG2 = 0xF6,
            REG_MOD_FIFO_DATA_CFG3 = 0xF7,
            REG_MOD_FIFO_DATA_CFG4 = 0xF8,
            REG_MOD_FIFO_DATA_CFG5 = 0xF9,
            REG_MOD_FIFO_DATA_CFG6 = 0xFA,
            REG_MOD_FIFO_DATA_CFG7 = 0xFB,
            REG_FIFO_STATUS0_H = 0xFC,
            REG_FIFO_STATUS0_L = 0xFD,
            REG_FIFO_STATUS1 = 0xFD,
            REG_FIFO_DATA_PROTOCOL = 0xFE,
            REG_FIFO_DATA = 0xFF
        } reg;

    public:
        TCS3530(TwoWire& wire, int int_pin = -1, int vsync_pin = -1);
        bool        begin();
        void        reset();
        void        enable();
        void        setWaitTime(uint8_t wtime, TCS3530_trigger_timing trigger_timing);
        void        setMeasurementTime(uint16_t sample_time_step, uint16_t sample_count);
        void        setGain(uint8_t gain);

        void        getData(TCS3530_data *data);
        void        convertDataToLab(TCS3530_data *data, TCS3530_lab_result *result);
        int         getID();
        
    private:
        float convertToLabHelper(float in);

        void i2c_write8(uint8_t addr, uint8_t reg, uint8_t payload);
        uint8_t i2c_read8(uint8_t addr, uint8_t reg);
        void i2c_write(uint8_t addr, uint8_t reg, uint8_t buffer[], uint8_t length);
        void i2c_read(uint8_t addr, uint8_t reg, uint8_t buffer[], uint8_t length);

        TwoWire& _wire;
        int _int, _vsync;
        float _delta_cubed = 216.0 / 24389.0;
        float _delta_squared = 36.0 / 841.0;
        float _const = 4.0 / 29.0;
};

#endif