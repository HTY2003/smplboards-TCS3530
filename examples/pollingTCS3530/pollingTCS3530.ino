#include <LP5816.h>
#include <TCS3530.h>

// See documentation for information on these configuration options
#define I2C_OBJECT                  Wire

#define LP5816_MAX_CURRENT          LP5816_MA_25_5
#define LP5816_LED_BRIGHTNESS       127

#define TCS3530_TRIGGER_MODE        TCS3530_OFF
#define TCS3530_WAIT_TIME           0
#define TCS3530_SAMPLE_TIME         179
#define TCS3530_SAMPLE_COUNT        3
#define TCS3530_GAIN                4

LP5816_config led_config;
LP5816_output led_output;
LP5816 driver(I2C_OBJECT);

TCS3530_config cs_config;
TCS3530_raw_data raw;
TCS3530_norm_data norm;
TCS3530_lab_data lab;
TCS3530 colour_sensor(I2C_OBJECT);

uint32_t last_print_time;

void setup()
{
  Serial.begin(115200);
  I2C_OBJECT.begin();

  delay(3000);

  led_config.max_current = LP5816_MAX_CURRENT;
  led_config.fade_time = LP5816_S_0;

  for (int i = 0; i < 4; i++) {
    led_config.channel_en[i] = true;
    led_config.fade_en[i] = false;
    led_config.exp_dim_en[i] = false;

    led_output.current[i] = LP5816_LED_BRIGHTNESS;
    led_output.duty_cycle[i] = 255;
  }

  cs_config.trigger_mode = TCS3530_TRIGGER_MODE;
  cs_config.wait_time = TCS3530_WAIT_TIME;
  cs_config.sample_time = TCS3530_SAMPLE_TIME;
  cs_config.sample_count = TCS3530_SAMPLE_COUNT;
  
  for (int i = 0; i < 8; i++) {
    cs_config.gain[i] = TCS3530_GAIN;
    norm.high[i] = 65535;
    norm.low[i] = 0;
  }

  driver.begin(led_config);
  driver.setOutput(led_output);
    
  while (!colour_sensor.begin(cs_config)) {
    Serial.println("Sensor cannot begin!");
    delay(1000);
  }
}

void loop()
{
  while (millis() - last_print_time < 1000) {
    // if updateRawData returns true, it means raw was updated with a new reading
    if (colour_sensor.updateRawData(raw)) {
      colour_sensor.convertRawToNorm(raw, norm);
      colour_sensor.convertNormToLab(norm, lab);
    }
  }

  last_print_time = millis();

  Serial.println("--- Raw Data ---");
  Serial.print("X: ");
  Serial.println(raw.data[TCS3530_X]);
  Serial.print("Y: ");
  Serial.println(raw.data[TCS3530_Y]);
  Serial.print("Z: ");
  Serial.println(raw.data[TCS3530_Z]);
  Serial.print("IR: ");
  Serial.println(raw.data[TCS3530_IR]);
  Serial.print("HgL: ");
  Serial.println(raw.data[TCS3530_HGL]);
  Serial.print("HgL: ");
  Serial.println(raw.data[TCS3530_HGH]);
  Serial.print("CLEAR: ");
  Serial.println(raw.data[TCS3530_CLEAR]);
  Serial.print("FLICKER: ");
  Serial.println(raw.data[TCS3530_FLICKER]);
  Serial.println();

  Serial.println("--- Normalized Data ---");
  Serial.print("X: ");
  Serial.println(norm.data[TCS3530_X]);
  Serial.print("Y: ");
  Serial.println(norm.data[TCS3530_Y]);
  Serial.print("Z: ");
  Serial.println(norm.data[TCS3530_Z]);
  Serial.print("IR: ");
  Serial.println(norm.data[TCS3530_IR]);
  Serial.print("HgL: ");
  Serial.println(norm.data[TCS3530_HGL]);
  Serial.print("HgL: ");
  Serial.println(norm.data[TCS3530_HGH]);
  Serial.print("CLEAR: ");
  Serial.println(norm.data[TCS3530_CLEAR]);
  Serial.print("FLICKER: ");
  Serial.println(norm.data[TCS3530_FLICKER]);
  Serial.println();

  Serial.println("--- LAB Data ---");
  Serial.print("L: ");
  Serial.println(lab.data[TCS3530_L]);
  Serial.print("A: ");
  Serial.println(lab.data[TCS3530_A]);
  Serial.print("B: ");
  Serial.println(lab.data[TCS3530_B]);
  Serial.println();
}