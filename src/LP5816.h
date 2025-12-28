#ifndef LP5816_H
#define LP5816_H

#include <Wire.h>

// LP5816 max current
typedef enum
{
    LP5816_MA_25_5,
    LP5816_MA_51
} LP5816_max_current;

// LP5816 fade time
typedef enum
{
    LP5816_S_0,
    LP5816_S_0_05,
    LP5816_S_0_10,
    LP5816_S_0_15,
    LP5816_S_0_20,
    LP5816_S_0_25,
    LP5816_S_0_30,
    LP5816_S_0_35,
    LP5816_S_0_40,
    LP5816_S_0_45,
    LP5816_S_0_50,
    LP5816_S_1,
    LP5816_S_2,
    LP5816_S_4,
    LP5816_S_6,
    LP5816_S_8,
} LP5816_fade_time;

// LP5816 channel
typedef enum
{
    LP5816_CH_0,
    LP5816_CH_1,
    LP5816_CH_2,
    LP5816_CH_3
} LP5816_channel;

// LP5816 channel enable
typedef struct
{
    bool ch0_en;
    bool ch1_en;
    bool ch2_en;
    bool ch3_en;   
} LP5816_channel_en;

// LP5816 PWM config
typedef struct
{
    LP5816_fade_time fade_time;

    bool ch0_fade_en;
    bool ch1_fade_en;
    bool ch2_fade_en;
    bool ch3_fade_en;

    bool ch0_exp_dim_en;
    bool ch1_exp_dim_en;
    bool ch2_exp_dim_en;
    bool ch3_exp_dim_en;
} LP5816_pwm_config;

// LP5816 output
typedef struct
{
    uint8_t ch0_current;
    uint8_t ch1_current;
    uint8_t ch2_current;
    uint8_t ch3_current;

    uint8_t ch0_duty_cycle;
    uint8_t ch1_duty_cycle;
    uint8_t ch2_duty_cycle;
    uint8_t ch3_duty_cycle;
} LP5816_output;

class LP5816
{
    static const uint8_t I2C_ADDR = 0x2C;
    static const uint8_t NUM_LEDS = 4;

    private:
        /**
         * LP5816 Register Map
         */
        typedef enum {
            REG_CHIP_EN = 0x00,
            REG_DEV_CONFIG0 = 0x01,
            REG_DEV_CONFIG1 = 0x02,
            REG_DEV_CONFIG2 = 0x03,
            REG_DEV_CONFIG3 = 0x04,
            REG_SHUTDOWN_CMD = 0x0D,
            REG_RESET_CMD = 0x0E,
            REG_UPDATE_CMD = 0x0F,
            REG_FLAG_CLR = 0x13,
            REG_OUT0_DC = 0x14,
            REG_OUT1_DC = 0x15,
            REG_OUT2_DC = 0x16,
            REG_OUT3_DC = 0x17,
            REG_OUT0_MANUAL_PWM = 0x18,
            REG_OUT1_MANUAL_PWM = 0x19,
            REG_OUT2_MANUAL_PWM = 0x1A,
            REG_OUT3_MANUAL_PWM = 0x1B,
            REG_FLAG = 0x40
        } reg;

    public:
        LP5816(TwoWire& wire);
        bool        begin(LP5816_max_current max_current, LP5816_channel_en channel_en, LP5816_pwm_config pwm_config);
        void        setMaxCurrent(LP5816_max_current max_current);
        void        setChannelEnable(LP5816_channel_en channel_en);
        void        setPWMConfig(LP5816_pwm_config pwm_config);
        void        setOneCurrent(LP5816_channel channel, uint8_t current);
        void        setAllCurrent(uint8_t current);
        void        setOneDutyCycle(LP5816_channel channel, uint8_t duty_cycle);
        void        setAllDutyCycle(uint8_t duty_cycle);
        void        setOutput(LP5816_output output);
        
    private:
        void        reset();
        void        updateDevConfig();
        void        enable();
        bool        getEnabled();

        void        i2c_write8(uint8_t addr, uint8_t reg, uint8_t payload);
        uint8_t     i2c_read8(uint8_t addr, uint8_t reg);
        void        i2c_write(uint8_t addr, uint8_t reg, uint8_t buffer[], uint8_t length);
        void        i2c_read(uint8_t addr, uint8_t reg, uint8_t buffer[], uint8_t length);

        TwoWire& _wire;
};



#endif