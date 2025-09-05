import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import CONF_ID

qmi8658_ns = cg.esphome_ns.namespace("qmi8658")
QMI8658Sensor = qmi8658_ns.class_("QMI8658Sensor", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(QMI8658Sensor),
        cv.Required("i2c_id"): cv.use_id(i2c),  # just pass the bus
        cv.Optional("accel_x"): sensor.sensor_schema(unit_of_measurement="m/s²"),
        cv.Optional("accel_y"): sensor.sensor_schema(unit_of_measurement="m/s²"),
        cv.Optional("accel_z"): sensor.sensor_schema(unit_of_measurement="m/s²"),
        cv.Optional("gyro_x"): sensor.sensor_schema(unit_of_measurement="°/s"),
        cv.Optional("gyro_y"): sensor.sensor_schema(unit_of_measurement="°/s"),
        cv.Optional("gyro_z"): sensor.sensor_schema(unit_of_measurement="°/s"),
        cv.Optional("temperature"): sensor.sensor_schema(unit_of_measurement="°C"),
    }
).extend(cv.polling_component_schema("1s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Create the I2CDevice from the bus
    i2c_bus = await cg.get_variable(config["i2c_id"])
    cg.add(var.set_i2c_bus(i2c_bus))

    # Attach sensors
    for key in ["accel_x","accel_y","accel_z","gyro_x","gyro_y","gyro_z","temperature"]:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_{key}")(sens))
