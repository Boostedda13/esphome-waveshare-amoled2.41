#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esp_log.h"

namespace esphome {
namespace qmi8658 {

static const char *TAG = "qmi8658";
#define QMI8658_ADDR 0x6B
#define Qmi8658EnableReg_Ctrl7 0x08
#define Qmi8658EnableAcc 0x1
#define Qmi8658EnableGyr 0x2

#define Qmi8658InterruptReg_Ctrl1 2

#define Qmi8658AccSetupReg_Ctrl2 3
#define Qmi8658AccRange_8g 0x02 << 4
#define Qmi8658AccOdr_125Hz 0x06

#define Qmi8658GyrSetupReg_Ctrl3 4
#define Qmi8658GyrRange_256dps 4 << 4
#define Qmi8658GyrOdr_125Hz 0x06

class QMI8658Driver {
 public:
  void set_i2c_device(i2c::I2CDevice *dev) { i2c_dev_ = dev; }

  bool begin() {
    if (!i2c_dev_) return false;

    uint8_t whoami = 0;
    if (!read_bytes(0x00, &whoami, 1)) return false;
    if (whoami == 0x05) 
    {
      // Got the correct chip on the bus
      ESP_LOGCONFIG(TAG, "Found IMU on the i2c 0x%02X", whoami);
      // Init: Setup
      write_byte(Qmi8658InterruptReg_Ctrl1, 0x60); // Disable interrupt pins, they are on extended io and don't work on ESPHome anyway
      write_byte(Qmi8658AccSetupReg_Ctrl2, Qmi8658AccRange_8g | Qmi8658AccOdr_125Hz); //accel config 8g range @ 125hz
      write_byte(Qmi8658GyrSetupReg_Ctrl3, Qmi8658GyrRange_256dps | Qmi8658GyrOdr_125Hz);
      // Init: accel & gyro
      write_byte(Qmi8658EnableReg_Ctrl7, Qmi8658EnableAcc | Qmi8658EnableGyr);
      return true;
    }
    else
    {
      ESP_LOGCONFIG(TAG, "Unexpected WHO_AM_I: 0x%02X", whoami);
      return false;
    }
  }

  bool get_accel_gyro(float &ax, float &ay, float &az,
                      float &gx, float &gy, float &gz) {
    uint8_t buf[12];
    if (!read_bytes(0x35, buf, 12)) return false;

    int16_t raw_ax = (int16_t)(buf[0] | buf[1] << 8);
    int16_t raw_ay = (int16_t)(buf[2] | buf[3] << 8);
    int16_t raw_az = (int16_t)(buf[4] | buf[5] << 8);
    int16_t raw_gx = (int16_t)(buf[6] | (buf[7] << 8));
    int16_t raw_gy = (int16_t)(buf[8] | (buf[9] << 8));
    int16_t raw_gz = (int16_t)(buf[10] | (buf[11] << 8));

    // Turn into useable values
    ax = raw_ax / 4096.0f * 9.81f; //Hard coded for the 8g range
    ay = raw_ay / 4096.0f * 9.81f;
    az = raw_az / 4096.0f * 9.81f;
    gx = raw_gx / 128.0f; //Hard coded for the 256dps range
    gy = raw_gy / 128.0f;
    gz = raw_gz / 128.0f;

    return true;
  }

  bool get_temperature(float &temp) {
    uint8_t buf[2];
    if (!read_bytes(0x33, buf, 2)) return false;
    int16_t raw = (int16_t)(buf[0] | buf[1] << 8);
    temp = raw / 256.0f;
    return true;
  }

 private:
  i2c::I2CDevice *i2c_dev_{nullptr};

  bool read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    // Set register pointer
    if (i2c_dev_->write(&reg, 1) != i2c::ERROR_OK)
      return false;

    // Read requested bytes
    return i2c_dev_->read(data, len) == i2c::ERROR_OK;
  }

  bool write_byte(uint8_t reg, uint8_t value) {
    return i2c_dev_->write_byte(reg, value) == i2c::ERROR_OK;
  }
};

class QMI8658Sensor : public PollingComponent {
 public:
  sensor::Sensor *accel_x_{nullptr};
  sensor::Sensor *accel_y_{nullptr};
  sensor::Sensor *accel_z_{nullptr};
  sensor::Sensor *gyro_x_{nullptr};
  sensor::Sensor *gyro_y_{nullptr};
  sensor::Sensor *gyro_z_{nullptr};
  sensor::Sensor *temperature_{nullptr};

  i2c::I2CDevice *i2c_dev_{nullptr};
  QMI8658Driver imu_;

  void set_accel_x(sensor::Sensor *s) { accel_x_ = s; }
  void set_accel_y(sensor::Sensor *s) { accel_y_ = s; }
  void set_accel_z(sensor::Sensor *s) { accel_z_ = s; }
  void set_gyro_x(sensor::Sensor *s) { gyro_x_ = s; }
  void set_gyro_y(sensor::Sensor *s) { gyro_y_ = s; }
  void set_gyro_z(sensor::Sensor *s) { gyro_z_ = s; }
  void set_temperature(sensor::Sensor *s) { temperature_ = s; }

  void set_i2c_bus(i2c::I2CBus *bus) {
    static i2c::I2CDevice *dev = new i2c::I2CDevice();  // default constructor
    dev->set_i2c_address(QMI8658_ADDR);
    dev->set_i2c_bus(bus);
    i2c_dev_ = dev;
    imu_.set_i2c_device(i2c_dev_);
  }

  void setup() override {
    if (!i2c_dev_) {
      ESP_LOGE(TAG, "I2C device not set!");
      return;
    }
    if (!imu_.begin()) {
      ESP_LOGE(TAG, "Failed to init QMI8658");
    }
  }

  void update() override {
    float ax, ay, az, gx, gy, gz, temp;
    if (imu_.get_accel_gyro(ax, ay, az, gx, gy, gz)) {
      if (accel_x_) accel_x_->publish_state(ax);
      if (accel_y_) accel_y_->publish_state(ay);
      if (accel_z_) accel_z_->publish_state(az);
      if (gyro_x_) gyro_x_->publish_state(gx);
      if (gyro_y_) gyro_y_->publish_state(gy);
      if (gyro_z_) gyro_z_->publish_state(gz);
    }
    if (imu_.get_temperature(temp) && temperature_) {
      temperature_->publish_state(temp);
    }
  }
};

}  // namespace qmi8658
}  // namespace esphome
