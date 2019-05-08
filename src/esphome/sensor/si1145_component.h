#ifndef ESPHOME_SENSOR_SI1145_COMPONENT_H
#define ESPHOME_SENSOR_SI1145_COMPONENT_H

#include "esphome/defines.h"

#ifdef USE_SI1145

#include "esphome/sensor/sensor.h"
#include "esphome/i2c_component.h"

ESPHOME_NAMESPACE_BEGIN

namespace sensor {

class SI1145UVLightSensor;
class SI1145VisibleLightSensor;
class SI1145InfaredLightSensor;

/// This class implements support for the SI1145 family of ultraviolet-light+visible-light+infared-light i2c sensors.
class SI1145Component : public PollingComponent, public I2CDevice {
 public:
  SI1145Component(I2CComponent *parent, const std::string &uv_name,
                  const std::string &visible_name, const std::string &ir_name,
                  uint8_t address = 0x60, uint32_t update_interval = 60000);
// ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  SI1145UVLightSensor *get_uv_sensor() const;
  SI1145VisibleLightSensor *get_visible_light_sensor() const;
  SI1145InfaredLightSensor *get_ir_sensor() const;

  void setup() override;
  void dump_config() override;
  void reset();
  uint16_t read_uv();
  uint16_t read_visible();
  uint16_t read_ir();
  uint16_t read_prox();

 protected:
  uint8_t write_param(uint8_t p, ui nt8_t v)

  SI1145UVLightSensor *uv_sensor_;
  SI1145VisibleLightSensor *visible_light_sensor_;
  SI1145InfaredLightSensor *ir_sensor_;
  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    WRONG_CHIP_ID,
  } error_code_{NONE};
};

}  // namespace sensor

ESPHOME_NAMESPACE_END

#endif  // USE_SI1145

#endif  // ESPHOME_SENSOR_SI1145_COMPONENT_H