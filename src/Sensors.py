#!/usr/bin/env python3
import time
import board
import busio
from adafruit_bno08x import (BNO08X_I2C, BNO_REPORT_ROTATION_VECTOR)
from adafruit_dps310 import advanced
import ms5837
from std_msgs.msg import String, Float32
from combined_sensors.msg import sensorValues
import rospy

class Sensors:

    pub = rospy.Publisher('sensors', sensorValues, queue_size = 10)
    rospy.init_node('sensor_values', anonymous = false)
    r = rospy.Rate(10)
    msg = sensorValues()

    def tempSensor():
        i2c = board.I2C()  
        dps310 = advanced.DPS310_Advanced(i2c)
        dps310.reset()
        
        dps310.temperature_oversample_count = advanced.SampleCount.COUNT_16
        dps310.temperature_rate = advanced.Rate.RATE_1_HZ
        dps310.mode = advanced.Mode.CONT_PRESTEMP
        dps310.wait_temperature_ready()

        return dps310.temperature()

    def pressureInsideSenor():
        i2c = board.I2C()  
        dps310 = advanced.DPS310_Advanced(i2c)
        dps310.reset()

        dps310.pressure_oversample_count = advanced.SampleCount.COUNT_2
        dps310.pressure_rate = advanced.Rate.RATE_1_HZ
        dps310.mode = advanced.Mode.CONT_PRESTEMP
        dps310.wait_pressure_ready()

        return dps310.pressure()

    def orientationSensor():
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        try: bno = BNO08X_I2C(i2c)
        except: print("Cannot connect to the bno sensor")
        else:
            bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

            bno_quad = []

            quat_i, quat_j, quat_k, quat_real = bno.quaternion()

            bno_quad.append(quat_i, quat_j, quat_k, quat_real)

            return bno_quad

    def pressureOutsideSenor():
        sensor = ms5837.MS5837()
        try: sensor.init()
        except: print("Cannot connect to the pressure sensor")
        else:
            sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
            return sensor.read()

while 1:
    tempSensor = Sensors.tempSensor()
    orientationSensor = Sensors.orientationSensor()
    pressureOutsideSenor = Sensors.pressureOutsideSenor()
    pressureInsideSenor = Sensors.pressureInsideSenor()

    print(f"Temp value: {tempSensor}, Orientation value: {orientationSensor}, Pressure Out value: {pressureOutsideSenor}, Pressure In value: {pressureInsideSenor}")

    msg.tempSensor = tempSensor
    msg.pressureInsideSenor = pressureInsideSenor
    msg.pressureOutsideSenor = pressureOutsideSenor
    msg.quat_i = bno_quad[0]
    msg.quat_j = bno_quad[1]
    msg.quat_k = bno_quad[2]
    msg.quat_real = bno_quad[3]

    pub.publish(msg)

    r.sleep()