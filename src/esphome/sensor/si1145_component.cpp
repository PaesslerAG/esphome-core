// Based on:
//   - https://www.silabs.com/documents/public/data-sheets/Si1145-46-47.pdf
//   - https://github.com/adafruit/Adafruit_SI1145_Library

#include "esphome/defines.h"

#ifdef USE_SI1145

#include "esphome/sensor/si1145_component.h"
#include "esphome/log.h"
#include "esphome/helpers.h"

ESPHOME_NAMESPACE_BEGIN

namespace sensor {

static const char *TAG = "sensor.si1145";

// Registers
static const uint8_t SI1145_REG_PARTID = 0x00;

static const uint8_t SI1145_REG_MEASRATE0 = 0x08;
static const uint8_t SI1145_REG_MEASRATE1 = 0x09;

static const uint8_t SI1145_REG_IRQEN = 0x04;
static const uint8_t SI1145_REG_IRQEN_ALSEVERYSAMPLE = 0x01;


static const uint8_t SI1145_REG_IRQMODE1 = 0x05;
static const uint8_t SI1145_REG_IRQMODE2 = 0x06;

static const uint8_t SI1145_REG_INTCFG = 0x03;
static const uint8_t SI1145_REG_INTCFG_INTOE = 0x01;
static const uint8_t SI1145_REG_IRQSTAT = 0x21;

static const uint8_t SI1145_REG_PSLED21 = 0x0F;

static const uint8_t SI1145_REG_COMMAND = 0x18;
static const uint8_t SI1145_REG_HWKEY = 0x07;

static const uint8_t SI1145_REG_UCOEFF0 = 0x13;
static const uint8_t SI1145_REG_UCOEFF1 = 0x14;
static const uint8_t SI1145_REG_UCOEFF2 = 0x15;
static const uint8_t SI1145_REG_UCOEFF3 = 0x16;

static const uint8_t SI1145_REG_PARAMWR = 0x17;
static const uint8_t SI1145_REG_PARAMRD = 0x2E;

static const uint8_t SI1145_REG_ALS_VIS_DATA0 = 0x22;
static const uint8_t SI1145_REG_ALS_IR_DATA0 = 0x24;
static const uint8_t SI1145_REG_ALS_UVINDEX0 = 0x2C;
static const uint8_t SI1145_REG_PS1_DATA0 = 0x26;

// Commands
static const uint8_t SI1145_CMD_RESET = 0x01;
static const uint8_t SI1145_CMD_PARAM_SET = 0xA0;

// Parameters
static const uint8_t SI1145_PARAM_PSADCGAIN = 0x0B;

static const uint8_t SI1145_PARAM_CHLIST = 0x01;
static const uint8_t SI1145_PARAM_CHLIST_ENUV = 0x80;
static const uint8_t SI1145_PARAM_CHLIST_ENALSIR = 0x20;
static const uint8_t SI1145_PARAM_CHLIST_ENALSVIS = 0x10;
static const uint8_t SI1145_PARAM_CHLIST_ENPS1 = 0x01;

static const uint8_t SI1145_PARAM_PS1ADCMUX = 0x07;
static const uint8_t SI1145_PARAM_ALSIRADCMUX = 0x0E;
static const uint8_t SI1145_PARAM_ADCMUX_SMALLIR = 0x00;
static const uint8_t SI1145_PARAM_ADCMUX_LARGEIR = 0x03;

static const uint8_t SI1145_PARAM_PSLED12SEL = 0x02;
static const uint8_t SI1145_PARAM_PSLED12SEL_PS1LED1 = 0x01;

static const uint8_t SI1145_PARAM_PSADCOUNTER = 0x0A;
static const uint8_t SI1145_PARAM_ALSIRADCOUNTER = 0x1D;
static const uint8_t SI1145_PARAM_ADCCOUNTER_511CLK = 0x70;

static const uint8_t SI1145_PARAM_PSADCMISC = 0x0C;
static const uint8_t SI1145_PARAM_PSADCMISC_RANGE = 0x20;
static const uint8_t SI1145_PARAM_PSADCMISC_PSMODE = 0x04;

static const uint8_t SI1145_PARAM_ALSIRADCGAIN = 0x1E;
static const uint8_t SI1145_PARAM_ALSIRADCMISC = 0x1F;
static const uint8_t SI1145_PARAM_ALSIRADCMISC_RANGE = 0x20;

static const uint8_t SI1145_PARAM_ALSVISADCGAIN = 0x11;
static const uint8_t SI1145_PARAM_ALSVISADCMISC = 0x12;
static const uint8_t SI1145_PARAM_ALSVISADCMISC_VISRANGE = 0x20;



SI1145Component::SI1145Component(I2CComponent *parent, const std::string &uv_name,
                                 const std::string &visible_name, const std::string &ir_name,
                                 uint8_t address, uint32_t update_interval)
  : PollingComponent(update_interval),
    I2CDevice(parent, address),
    uv_sensor_(new SI1145UVLightSensor(uv_name, this)),
    visible_light_sensor_(new SI1145VisibleLightSensor(visible_name, this)),
    ir_sensor_(new SI1145InfaredLightSensor(ir_name, this)) {}

void SSI1145Component::setup() {
  ESP_LOGCONFIG(TAG, "setting up SI1145...")
  uint8_t part_id = 0
  if (!this->read_byte(SI1145_REG_PARTID, &part_id)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }
  if (part_id != 0x45) {
    // TODO: Refine Error Message
    this->error_code_ = WRONG_CHIP_ID;
    this->mark_failed();
    return;
  }

  // Configure SI1145
  write_byte(SI1145_REG_MEASRATE0, 0);
  write_byte(SI1145_REG_MEASRATE1, 0);
  write_byte(SI1145_REG_IRQEN, 0);
  write_byte(SI1145_REG_IRQMODE1, 0);
  write_byte(SI1145_REG_IRQMODE2, 0);
  write_byte(SI1145_REG_INTCFG, 0);
  write_byte(SI1145_REG_IRQSTAT, 0xFF);

  write_bytel(SI1145_REG_COMMAND, SI1145_CMD_RESET);
  // wait for device to reset
  delay(10);
  write_byte(SI1145_REG_HWKEY, 0x17);
  
  delay(10);
  // enable UVindex measurement coefficients!
  write_byte(SI1145_REG_UCOEFF0, 0x29);
  write_byte(SI1145_REG_UCOEFF1, 0x89);
  write_byte(SI1145_REG_UCOEFF2, 0x02);
  write_byte(SI1145_REG_UCOEFF3, 0x00);

  write_param(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
  SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS |
  SI1145_PARAM_CHLIST_ENPS1);

  // enable interrupt on every sample
  write_byte(SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);  
  write_byte(SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);

  // program LED current
  write_byte(SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
  write_param(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
  // prox sensor #1 uses LED #1
  write_param(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
  // fastest clocks, clock div 1
  write_param(SI1145_PARAM_PSADCGAIN, 0);
  // take 511 clocks to measure
  write_param(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in prox mode, high range
  write_param(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE|
    SI1145_PARAM_PSADCMISC_PSMODE);

  write_param(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);  
  // fastest clocks, clock div 1
  write_param(SI1145_PARAM_ALSIRADCGAIN, 0);
  // take 511 clocks to measure
  write_param(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode
  write_param(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);

  // fastest clocks, clock div 1
  write_param(SI1145_PARAM_ALSVISADCGAIN, 0);
  // take 511 clocks to measure
  write_param(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode (not normal signal)
  write_param(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);

  // measurement rate for auto
  write_byte(SI1145_REG_MEASRATE0, 0xFF); // 255 * 31.25uS = 8ms
  
  // auto run
  write8_byte(SI1145_REG_COMMAND, SI1145_PSALS_AUTO);
  
}

void SI1145Component::dump_config() {
  ESP_LOGCONFIG(TAG, "SI1145:");
  LOG_I2C_DEVICE(this);
  switch (this->error_code_) {
    case COMMUNICATION_FAILED:
      ESP_LOGE(TAG, "Communication with SI1145 failed!");
      break;
    case WRONG_CHIP_ID:
      ESP_LOGE(TAG, "SI1145 has wrong part ID! Is it a SI1145?");
      break;
    case NONE:
    default:
      break;
}

uint8_t SI1145Component::write_param(uint8_t p, uint8_t v) {
  write_byte(SI1145_REG_PARAMWR, v);
  write_byte(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
  return read_byte(SI1145_REG_PARAMRD);
}

// returns the UV index * 100 (divide by 100 to get the index)
uint16_t SI1145Component::read_uv() {
 return read_byte_16(SI1145_REG_ALS_UVINDEX0); 
}

// returns visible+IR light levels
uint16_t SI1145Component::read_visible() {
 return read_byte_16(SI1145_REG_ALS_VIS_DATA0); 
}

// returns IR light levels
uint16_t SI1145Component::read_ir() {
 return read_byte_16(SI1145_REG_ALS_IR_DATA0); 
}

// returns "Proximity" - assumes an IR LED is attached to LED
uint16_t Adafruit_SI1145::read_prox() {
 return read_byte_16(SI1145_REG_PS1_DATA0); 
}

SI1145UVLightSensor *SI1145Component::get_uv_sensor() const { return this->uv_sensor_; }
SI1145VisibleLightSensor *SI1145Component::get_visible_light_sensor() const { return this->visible_light_sensor_; }
SI1145InfaredLightSensor *SI1145Component::get_ir_sensor() const { return this->ir_sensor_; }

} // namespace sensor

ESPHOME_NAMESPACE_END

//#endif  // USE_SI1145
