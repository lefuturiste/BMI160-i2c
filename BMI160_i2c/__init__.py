# FROM https://github.com/serioeseGmbH/BMI160
# based on https://github.com/arduino/ArduinoCore-arc32/blob/master/libraries/CurieIMU/src/BMI160.cpp

from BMI160_i2c import registers
from BMI160_i2c import commands
from BMI160_i2c import definitions
from BMI160_i2c.sleep import sleep_us, sleep_ms

from struct import unpack
from smbus2 import SMBus, i2c_msg

class Driver:

  # Power on and prepare for general usage.
  # This will activate the device and take it out of sleep mode (which must be done
  # after start-up). This function also sets both the accelerometer and the gyroscope
  # to default range settings, namely +/- 2g and +/- 250 degrees/sec.
  def __init__(self, addr=0x68, bus=1):
    self.addr = addr
    
    # Initialize the i2c bus driver
    self.bus = SMBus(bus)
    
    # Issue a soft-reset to bring the device into a clean state
    self._reg_write(registers.CMD, commands.SOFT_RESET)
    sleep_ms(1)

    # Issue a dummy-read to force the device into I2C comms mode
    self._reg_read(0x7F)
    sleep_ms(1)

    # Power up the accelerometer
    self._reg_write(registers.CMD, commands.ACC_MODE_NORMAL)
    # Wait for power-up to complete
    while (1 != self._reg_read_bits(registers.PMU_STATUS, definitions.ACC_PMU_STATUS_BIT, definitions.ACC_PMU_STATUS_LEN)):
      pass
    sleep_ms(1)

    # Power up the gyroscope
    self._reg_write(registers.CMD, commands.GYR_MODE_NORMAL)
    sleep_ms(1)
    # Wait for power-up to complete
    while (1 != self._reg_read_bits(registers.PMU_STATUS, definitions.GYR_PMU_STATUS_BIT, definitions.GYR_PMU_STATUS_LEN)):
      sleep_ms(200)
      pass
    sleep_ms(1)

    self.setFullScaleGyroRange(definitions.GYRO_RANGE_250, 250.0)
    self.setFullScaleAccelRange(definitions.ACCEL_RANGE_2G, 2.0)

    # Only PIN1 interrupts currently supported - map all interrupts to PIN1
    self._reg_write(registers.INT_MAP_0, 0xFF)
    self._reg_write(registers.INT_MAP_1, 0xF0)
    self._reg_write(registers.INT_MAP_2, 0x00)

  def _reg_read_bits(self, reg, pos, len):
    b = self._reg_read(reg)
    mask = (1 << len) - 1
    b >>= pos
    b &= mask
    return b

  def _reg_write_bits(self, reg, data, pos, len):
    b = self._reg_read(reg)
    mask = ((1 << len) - 1) << pos
    data <<= pos  # shift data into correct position
    data &= mask  # zero all non-important bits in data
    b &= ~(mask)  # zero all important bits in existing byte
    b |= data     # combine data with existing byte
    self._reg_write(reg, b)

  def _is_bit_set(self, value, bit):
    return value & (1 << bit)

  # Test the sign bit and set remaining MSBs if sign bit is set */
  # def _sign_extend(val, orig):
  #     return (((val) & (1 << ((orig) - 1))) ? (val | (((1 << (1 + (sizeof(val) << 3) - (orig))) - 1) << (orig))) : val)

  # Get Device ID.
  # This register is used to verify the identity of the device (0b11010001, 0xD1).
  # @return Device ID (should be 0xD1)
  # @see BMI160_RA_CHIP_ID
  def get_device_id(self):
    return self._reg_read(registers.CHIP_ID)

  # Get gyroscope output data rate.
  # The gyr_odr parameter allows setting the output data rate of the gyroscope
  # as described in the table below.
  #
  # <pre>
  #  6 =   25Hz
  #  7 =   50Hz
  #  8 =  100Hz
  #  9 =  200Hz
  # 10 =  400Hz
  # 11 =  800Hz
  # 12 = 1600Hz
  # 13 = 3200Hz
  # </pre>
  #
  # @return Current sample rate
  # @see registers.GYRO_CONF
  # @see BMI160GyroRate
  def get_gyro_rate(self):
    return self._reg_read_bits(registers.GYRO_CONF, definitions.GYRO_RATE_SEL_BIT, definitions.GYRO_RATE_SEL_LEN)

  # Set gyroscope output data rate.
  # @param rate New output data rate
  # @see get_gyro_rate()
  # @see definitions.GYRO_RATE_25HZ
  # @see registers.GYRO_CONF
  def set_gyro_rate(self, rate):
    self._reg_write_bits(registers.GYRO_CONF, rate,
      definitions.GYRO_RATE_SEL_BIT, definitions.GYRO_RATE_SEL_LEN)

  # Get accelerometer output data rate.
  # The acc_odr parameter allows setting the output data rate of the accelerometer
  # as described in the table below.
  #
  # <pre>
  #  5 =  25/2Hz
  #  6 =    25Hz
  #  7 =    50Hz
  #  8 =   100Hz
  #  9 =   200Hz
  # 10 =   400Hz
  # 11 =   800Hz
  # 12 =  1600Hz
  # 13 =  3200Hz
  # </pre>
  #
  # @return Current sample rate
  # @see registers.ACCEL_CONF
  # @see BMI160AccelRate
  def get_accel_rate(self):
    return self._reg_read_bits(registers.ACCEL_CONF, definitions.ACCEL_RATE_SEL_BIT, definitions.ACCEL_RATE_SEL_LEN)

  # Set accelerometer output data rate.
  # @param rate New output data rate
  # @see get_accel_rate()
  # @see registers.ACCEL_CONF
  def set_accel_rate(self, rate):
    self._reg_write_bits(registers.ACCEL_CONF, rate,
      definitions.ACCEL_RATE_SEL_BIT, definitions.ACCEL_RATE_SEL_LEN)

  # Get gyroscope digital low-pass filter mode.
  # The gyro_bwp parameter sets the gyroscope digital low pass filter configuration.
  #
  # When the filter mode is set to Normal (@see definitions.DLPF_MODE_NORM), the filter
  # bandwidth for each respective gyroscope output data rates is shown in the table below:
  #
  # <pre>
  # ODR     | 3dB cut-off
  # --------+------------
  #    25Hz | 10.7Hz
  #    50Hz | 20.8Hz
  #   100Hz | 39.9Hz
  #   200Hz | 74.6Hz
  #   400Hz | 136.6Hz
  #   800Hz | 254.6Hz
  #  1600Hz | 523.9Hz
  #  3200Hz | 890Hz
  # </pre>
  #
  # When the filter mode is set to OSR2 (@see definitions.DLPF_MODE_OSR2), the filter
  # bandwidths above are approximately halved.
  #
  # When the filter mode is set to OSR4 (@see definitions.DLPF_MODE_OSR4), the filter
  # bandwidths above are approximately 4 times smaller.
  #
  # @return DLFP configuration
  # @see registers.GYRO_CONF
  # @see BMI160DLPFMode
  def get_gyro_dlpf_mode(self):
    return self._reg_read_bits(registers.GYRO_CONF, definitions.GYRO_DLPF_SEL_BIT, definitions.GYRO_DLPF_SEL_LEN)

  # Set gyroscope digital low-pass filter configuration.
  # @param mode New DLFP configuration setting
  # @see get_gyro_dlpf_mode()
  def set_gyro_dlpf_mod(self, emode):
    return self._reg_write_bits(registers.GYRO_CONF, mode, definitions.GYRO_DLPF_SEL_BIT, definitions.GYRO_DLPF_SEL_LEN)

  # Get accelerometer digital low-pass filter mode.
  # The acc_bwp parameter sets the accelerometer digital low pass filter configuration.
  #
  # When the filter mode is set to Normal (@see definitions.DLPF_MODE_NORM), the filter
  # bandwidth for each respective accelerometer output data rates is shown in the table below:
  #
  # <pre>
  # ODR     | 3dB cut-off
  # --------+--------------
  #  12.5Hz |  5.06Hz
  #    25Hz | 10.12Hz
  #    50Hz | 20.25Hz
  #   100Hz | 40.5Hz
  #   200Hz | 80Hz
  #   400Hz | 162Hz (155Hz for Z axis)
  #   800Hz | 324Hz (262Hz for Z axis)
  #  1600Hz | 684Hz (353Hz for Z axis)
  # </pre>
  #
  # When the filter mode is set to OSR2 (@see definitions.DLPF_MODE_OSR2), the filter
  # bandwidths above are approximately halved.
  #
  # When the filter mode is set to OSR4 (@see definitions.DLPF_MODE_OSR4), the filter
  # bandwidths above are approximately 4 times smaller.
  #
  # @return DLFP configuration
  # @see registers.GYRO_CONF
  # @see BMI160DLPFMode
  def getAccelDLPFMode(self):
    return self._reg_read_bits(registers.ACCEL_CONF, definitions.ACCEL_DLPF_SEL_BIT, definitions.ACCEL_DLPF_SEL_LEN)

  # Set accelerometer digital low-pass filter configuration.
  # @param mode New DLFP configuration setting
  # @see getAccelDLPFMode()
  def setAccelDLPFMode(self, mode):
    return self._reg_write_bits(registers.ACCEL_CONF, mode, definitions.ACCEL_DLPF_SEL_BIT, definitions.ACCEL_DLPF_SEL_LEN)

  # Get full-scale gyroscope range.
  # The gyr_range parameter allows setting the full-scale range of the gyro sensors,
  # as described in the table below.
  #
  # <pre>
  # 4 = +/-  125 degrees/sec
  # 3 = +/-  250 degrees/sec
  # 2 = +/-  500 degrees/sec
  # 1 = +/- 1000 degrees/sec
  # 0 = +/- 2000 degrees/sec
  # </pre>
  #
  # @return Current full-scale gyroscope range setting
  # @see registers.GYRO_RANGE
  # @see BMI160GyroRange
  def getFullScaleGyroRange(self):
    return self._reg_read_bits(registers.GYRO_RANGE, definitions.GYRO_RANGE_SEL_BIT, definitions.GYRO_RANGE_SEL_LEN)

  # Set full-scale gyroscope range.
  # @param range New full-scale gyroscope range value
  # @see getFullScaleGyroRange()
  def setFullScaleGyroRange(self, range, real):
    self._reg_write_bits(registers.GYRO_RANGE, range,
      definitions.GYRO_RANGE_SEL_BIT, definitions.GYRO_RANGE_SEL_LEN)
    self.gyro_range = real

  # Get full-scale accelerometer range.
  # The FS_SEL parameter allows setting the full-scale range of the accelerometer
  # sensors, as described in the table below.
  #
  # <pre>
  #  3 = +/- 2g
  #  5 = +/- 4g
  #  8 = +/- 8g
  # 12 = +/- 16g
  # </pre>
  #
  # @return Current full-scale accelerometer range setting
  # @see registers.ACCEL_RANGE
  # @see BMI160AccelRange
  def getFullScaleAccelRange(self):
    return self._reg_read_bits(registers.ACCEL_RANGE, definitions.ACCEL_RANGE_SEL_BIT, definitions.ACCEL_RANGE_SEL_LEN)

  # Set full-scale accelerometer range.
  # @param range New full-scale accelerometer range setting
  # @see getFullScaleAccelRange()
  # @see BMI160AccelRange
  def setFullScaleAccelRange(self, range, real):
    self._reg_write_bits(registers.ACCEL_RANGE, range,
      definitions.ACCEL_RANGE_SEL_BIT, definitions.ACCEL_RANGE_SEL_LEN)
    self.accel_range = real

  # Get accelerometer offset compensation enabled value.
  # @see getXAccelOffset()
  # @see registers.OFFSET_6
  def getAccelOffsetEnabled(self):
    return 0 != (self._reg_read_bits(registers.OFFSET_6, definitions.ACC_OFFSET_EN, 1))

  # Set accelerometer offset compensation enabled value.
  # @see getXAccelOffset()
  # @see registers.OFFSET_6
  def setAccelOffsetEnabled(self, enabled):
    self._reg_write_bits(registers.OFFSET_6,
      1 if enabled else 0, definitions.ACC_OFFSET_EN, 1)

  # Execute internal calibration to generate Accelerometer X-Axis offset value.
  # This populates the Accelerometer offset compensation value for the X-Axis only.
  # These can be retrieved using the getXAccelOffset() methods.
  # Note that this procedure may take up to 250ms to complete.
  #
  # IMPORTANT: The user MUST ensure NO movement and correct orientation of the
  # BMI160 device occurs while this auto-calibration process is active.
  # For example, to calibrate to a target of 0g on the X-axis, the BMI160 device
  # must be resting horizontally as shown in Section 5.2 of the BMI160 Data Sheet.
  #
  # To enable offset compensation, @see setAccelOffsetEnabled()
  #
  # @param target X-axis target value (0 = 0g, 1 = +1g, -1 = -1g)
  # @see setAccelOffsetEnabled()
  # @see getXAccelOffset()
  # @see registers.FOC_CONF
  # @see registers.CMD
  def autoCalibrateXAccelOffset(self, target):
    foc_conf = 0
    if (target == 1):
      foc_conf = (0x1 << definitions.FOC_ACC_X_BIT)
    elif (target == -1):
      foc_conf = (0x2 << definitions.FOC_ACC_X_BIT)
    elif (target == 0):
      foc_conf = (0x3 << definitions.FOC_ACC_X_BIT)
    else:
      return  # Invalid target value

    self._reg_write(registers.FOC_CONF, foc_conf)
    self._reg_write(registers.CMD, commands.START_FOC)
    while (not (self._reg_read_bits(registers.STATUS, definitions.STATUS_FOC_RDY, 1))):
      sleep_ms(1)

  # Execute internal calibration to generate Accelerometer Y-Axis offset value.
  # This populates the Accelerometer offset compensation value for the Y-Axis only.
  # These can be retrieved using the getYAccelOffset() methods.
  # Note that this procedure may take up to 250ms to complete.
  #
  # IMPORTANT: The user MUST ensure NO movement and correct orientation of the
  # BMI160 device occurs while this auto-calibration process is active.
  # For example, to calibrate to a target of 0g on the Y-axis, the BMI160 device
  # must be resting horizontally as shown in Section 5.2 of the BMI160 Data Sheet.
  #
  # To enable offset compensation, @see setAccelOffsetEnabled()
  #
  # @param target Y-axis target value (0 = 0g, 1 = +1g, -1 = -1g)
  # @see setAccelOffsetEnabled()
  # @see getYAccelOffset()
  # @see registers.FOC_CONF
  # @see registers.CMD
  def autoCalibrateYAccelOffset(self, target):
    foc_conf = 0
    if (target == 1):
      foc_conf = (0x1 << definitions.FOC_ACC_Y_BIT)
    elif (target == -1):
      foc_conf = (0x2 << definitions.FOC_ACC_Y_BIT)
    elif (target == 0):
      foc_conf = (0x3 << definitions.FOC_ACC_Y_BIT)
    else:
      return  # Invalid target value

    self._reg_write(registers.FOC_CONF, foc_conf)
    self._reg_write(registers.CMD, commands.START_FOC)
    while (not (self._reg_read_bits(registers.STATUS, definitions.STATUS_FOC_RDY, 1))):
      sleep_ms(1)

  # Execute internal calibration to generate Accelerometer Z-Axis offset value.
  # This populates the Accelerometer offset compensation value for the Z-Axis only.
  # These can be retrieved using the getZAccelOffset() methods.
  # Note that this procedure may take up to 250ms to complete.
  #
  # IMPORTANT: The user MUST ensure NO movement and correct orientation of the
  # BMI160 device occurs while this auto-calibration process is active.
  # For example, to calibrate to a target of +1g on the Z-axis, the BMI160 device
  # must be resting horizontally as shown in Section 5.2 of the BMI160 Data Sheet.
  #
  # To enable offset compensation, @see setAccelOffsetEnabled()
  #
  # @param target Z-axis target value (0 = 0g, 1 = +1g, -1 = -1g)
  # @see setAccelOffsetEnabled()
  # @see getZAccelOffset()
  # @see registers.FOC_CONF
  # @see registers.CMD
  def autoCalibrateZAccelOffset(self, target):
    foc_conf = 0
    if (target == 1):
      foc_conf = (0x1 << definitions.FOC_ACC_Z_BIT)
    elif (target == -1):
      foc_conf = (0x2 << definitions.FOC_ACC_Z_BIT)
    elif (target == 0):
      foc_conf = (0x3 << definitions.FOC_ACC_Z_BIT)
    else:
      return  # Invalid target value

    self._reg_write(registers.FOC_CONF, foc_conf)
    self._reg_write(registers.CMD, commands.START_FOC)
    while (not (self._reg_read_bits(registers.STATUS, definitions.STATUS_FOC_RDY, 1))):
      sleep_ms(1)

  # Get offset compensation value for accelerometer X-axis data.
  # The value is represented as an 8-bit two-complement number in
  # units of 3.9mg per LSB.
  # @see registers.OFFSET_0
  def getXAccelOffset(self):
    return self._reg_read(registers.OFFSET_0)

  # Set offset compensation value for accelerometer X-axis data.
  # This is used for applying manual calibration constants if required.
  # For auto-calibration, @see autoCalibrateXAccelOffset().
  # @see getXAccelOffset()
  # @see registers.OFFSET_0
  def setXAccelOffset(self, offset):
    self._reg_write(registers.OFFSET_0, offset)
    getAccelerationX()  # Read and discard the next data value

  # Get offset compensation value for accelerometer Y-axis data.
  # The value is represented as an 8-bit two-complement number in
  # units of 3.9mg per LSB.
  # @see registers.OFFSET_1
  def getYAccelOffset(self):
    return self._reg_read(registers.OFFSET_1)

  # Set offset compensation value for accelerometer Y-axis data.
  # This is used for applying manual calibration constants if required.
  # For auto-calibration, @see autoCalibrateYAccelOffset().
  # @see getYAccelOffset()
  # @see registers.OFFSET_1
  def setYAccelOffset(self, offset):
    self._reg_write(registers.OFFSET_1, offset)
    getAccelerationY()  # Read and discard the next data value

  # Get offset compensation value for accelerometer Z-axis data.
  # The value is represented as an 8-bit two-complement number in
  # units of 3.9mg per LSB.
  # @see registers.OFFSET_2
  def getZAccelOffset(self):
    return self._reg_read(registers.OFFSET_2)

  # Set offset compensation value for accelerometer Z-axis data.
  # This is used for applying manual calibration constants if required.
  # For auto-calibration, @see autoCalibrateZAccelOffset().
  # @see getZAccelOffset()
  # @see registers.OFFSET_2
  def setZAccelOffset(self, offset):
    self._reg_write(registers.OFFSET_2, offset)
    getAccelerationZ()  # Read and discard the next data value

  # Get gyroscope offset compensation enabled value.
  # @see getXGyroOffset()
  # @see registers.OFFSET_6
  def getGyroOffsetEnabled(self):
    return 0 != (self._reg_read_bits(registers.OFFSET_6, definitions.GYR_OFFSET_EN, 1))

  # Set gyroscope offset compensation enabled value.
  # @see getXGyroOffset()
  # @see registers.OFFSET_6
  def setGyroOffsetEnabled(self, enabled):
    self._reg_write_bits(
      registers.OFFSET_6, 0x1 if enabled else 0, definitions.GYR_OFFSET_EN, 1)

  # Execute internal calibration to generate Gyro offset values.
  # This populates the Gyro offset compensation values for all 3 axes.
  # These can be retrieved using the get[X/Y/Z]GyroOffset() methods.
  # Note that this procedure may take up to 250ms to complete.
  #
  # IMPORTANT: The user MUST ensure that NO rotation of the BMI160 device
  # occurs while this auto-calibration process is active.
  #
  # To enable offset compensation, @see setGyroOffsetEnabled()
  # @see setGyroOffsetEnabled()
  # @see getXGyroOffset()
  # @see getYGyroOffset()
  # @see getZGyroOffset()
  # @see registers.FOC_CONF
  # @see registers.CMD
  def autoCalibrateGyroOffset(self):
    foc_conf = (1 << definitions.FOC_GYR_EN)
    self._reg_write(registers.FOC_CONF, foc_conf)
    self._reg_write(registers.CMD, commands.START_FOC)
    while (not (self._reg_read_bits(registers.STATUS, definitions.STATUS_FOC_RDY, 1))):
      sleep_ms(1)

  # Get offset compensation value for gyroscope X-axis data.
  # The value is represented as an 10-bit two-complement number in
  # units of 0.061 degrees/s per LSB (sign-extended for type).
  # @see registers.OFFSET_3
  # @see registers.OFFSET_6
  def getXGyroOffset(self):
    offset = self._reg_read(registers.OFFSET_3)
    offset |= (self._reg_read_bits(registers.OFFSET_6,
      definitions.GYR_OFFSET_X_MSB_BIT, definitions.GYR_OFFSET_X_MSB_LEN)) << 8
    return self._sign_extend(offset, 10)

  # Set offset compensation value for gyroscope X-axis data.
  # This is used for applying manual calibration constants if required.
  # For auto-calibration, @see autoCalibrateGyroOffset().
  # @see getXGyroOffset()
  # @see registers.OFFSET_3
  # @see registers.OFFSET_6
  def setXGyroOffset(self, offset):
    self._reg_write(registers.OFFSET_3, offset)
    self._reg_write_bits(registers.OFFSET_6, offset >> 8,
      definitions.GYR_OFFSET_X_MSB_BIT, definitions.GYR_OFFSET_X_MSB_LEN)
    getRotationX()  # Read and discard the next data value

  # Get offset compensation value for gyroscope Y-axis data.
  # The value is represented as an 10-bit two-complement number in
  # units of 0.061 degrees/s per LSB (sign-extended for type).
  # @see registers.OFFSET_4
  # @see registers.OFFSET_6
  def getYGyroOffset(self):
    offset = self._reg_read(registers.OFFSET_4)
    offset |= (self._reg_read_bits(registers.OFFSET_6,
      definitions.GYR_OFFSET_Y_MSB_BIT, definitions.GYR_OFFSET_Y_MSB_LEN)) << 8
    return self._sign_extend(offset, 10)

  # Set offset compensation value for gyroscope Y-axis data.
  # This is used for applying manual calibration constants if required.
  # For auto-calibration, @see autoCalibrateGyroOffset().
  # @see getYGyroOffset()
  # @see registers.OFFSET_4
  # @see registers.OFFSET_6
  def setYGyroOffset(self, offset):
    self._reg_write(registers.OFFSET_4, offset)
    self._reg_write_bits(registers.OFFSET_6, offset >> 8,
      definitions.GYR_OFFSET_Y_MSB_BIT, definitions.GYR_OFFSET_Y_MSB_LEN)
    getRotationY()  # Read and discard the next data value

  # Get offset compensation value for gyroscope Z-axis data.
  # The value is represented as an 10-bit two-complement number in
  # units of 0.061 degrees/s per LSB (sign-extended for type).
  # @see registers.OFFSET_5
  # @see registers.OFFSET_6
  def getZGyroOffset(self):
    offset = self._reg_read(registers.OFFSET_5)
    offset |= (self._reg_read_bits(registers.OFFSET_6,
      definitions.GYR_OFFSET_Z_MSB_BIT, definitions.GYR_OFFSET_Z_MSB_LEN)) << 8
    return self._sign_extend(offset, 10)

  # Set offset compensation value for gyroscope Z-axis data.
  # This is used for applying manual calibration constants if required.
  # For auto-calibration, @see autoCalibrateGyroOffset().
  # @see getZGyroOffset()
  # @see registers.OFFSET_5
  # @see registers.OFFSET_6
  def setZGyroOffset(self, offset):
    self._reg_write(registers.OFFSET_5, offset)
    self._reg_write_bits(registers.OFFSET_6, offset >> 8,
      definitions.GYR_OFFSET_Z_MSB_BIT, definitions.GYR_OFFSET_Z_MSB_LEN)
    getRotationZ()  # Read and discard the next data value

  # Get free-fall event acceleration threshold.
  # This register configures the detection threshold for Free Fall event
  # detection. The unit of int_low_th is 1LSB = 7.81mg (min: 3.91mg). Free Fall
  # is detected when the absolute value of the accelerometer measurements for the
  # three axes are each less than the detection threshold. This condition
  # triggers the Free-Fall (low-g) interrupt if the condition is maintained for
  # the duration specified in the int_low_dur field of the INT_LOWHIGH[0]
  # register (@see registers.INT_LOWHIGH_0)
  #
  # For more details on the Free Fall detection interrupt, see Section 2.6.7 of the
  # BMI160 Data Sheet.
  #
  # @return Current free-fall acceleration threshold value (LSB = 7.81mg, 0 = 3.91mg)
  # @see registers.INT_LOWHIGH_1
  def getFreefallDetectionThreshold(self):
    return self._reg_read(registers.INT_LOWHIGH_1)

  # Set free-fall event acceleration threshold.
  # @param threshold New free-fall acceleration threshold value (LSB = 7.81mg, 0 = 3.91mg)
  # @see getFreefallDetectionThreshold()
  # @see registers.INT_LOWHIGH_1
  def setFreefallDetectionThreshold(self, threshold):
    self._reg_write(registers.INT_LOWHIGH_1, threshold)

  # Get free-fall event duration threshold.
  # This register configures the duration threshold for Free Fall event
  # detection. The int_low_dur field of the INT_LOWHIGH[0] register has a unit
  # of 1 LSB = 2.5 ms (minimum 2.5ms).
  #
  # For more details on the Free Fall detection interrupt, see Section 2.6.7 of
  # the BMI160 Data Sheet.
  #
  # @return Current free-fall duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
  # @see registers.INT_LOWHIGH_0
  def getFreefallDetectionDuration(self):
    return self._reg_read(registers.INT_LOWHIGH_0)

  # Set free-fall event duration threshold.
  # @param duration New free-fall duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
  # @see getFreefallDetectionDuration()
  # @see registers.INT_LOWHIGH_0
  def setFreefallDetectionDuration(self, duration):
    self._reg_write(registers.INT_LOWHIGH_0, duration)

  # Get shock event acceleration threshold.
  # This register configures the detection threshold for Shock event
  # detection. The unit of threshold is dependent on the accelerometer
  # sensitivity range (@see getFullScaleAccelRange()):
  #
  # <pre>
  # Full Scale Range | LSB Resolution
  # -----------------+----------------
  # +/- 2g       |  7.81 mg/LSB (0 =  3.91mg)
  # +/- 4g       | 15.63 mg/LSB (0 =  7.81mg)
  # +/- 8g       | 31.25 mg/LSB (0 = 15.63mg)
  # +/- 16g      | 62.50 mg/LSB (0 = 31.25mg)
  # </pre>
  #
  # Shock is detected when the absolute value of the accelerometer measurements
  # for any of the three axes exceeds the detection threshold. This condition
  # triggers the Shock (high-g) interrupt if the condition is maintained without
  # a sign-change for the duration specified in the int_high_dur field of the
  # INT_LOWHIGH[3] register (@see registers.INT_LOWHIGH_3).
  #
  # For more details on the Shock (high-g) detection interrupt, see Section 2.6.8 of the
  # BMI160 Data Sheet.
  #
  # @return Current shock acceleration threshold value
  # @see registers.INT_LOWHIGH_4
  def getShockDetectionThreshold(self):
    return self._reg_read(registers.INT_LOWHIGH_4)

  # Set shock event acceleration threshold.
  # @param threshold New shock acceleration threshold value
  # @see getShockDetectionThreshold()
  # @see registers.INT_LOWHIGH_4
  def setShockDetectionThreshold(self, threshold):
    self._reg_write(registers.INT_LOWHIGH_4, threshold)

  # Get shock event duration threshold.
  # This register configures the duration threshold for Shock event
  # detection. The int_high_dur field of the INT_LOWHIGH[3] register has a unit
  # of 1 LSB = 2.5 ms (minimum 2.5ms).
  #
  # For more details on the Shock detection interrupt, see Section 2.6.8 of
  # the BMI160 Data Sheet.
  #
  # @return Current shock duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
  # @see registers.INT_LOWHIGH_3
  def getShockDetectionDuration(self):
    return self._reg_read(registers.INT_LOWHIGH_3)

  # Set free-fall event duration threshold.
  # @param duration New free-fall duration threshold value (LSB = 2.5ms, 0 = 2.5ms)
  # @see getFreefallDetectionDuration()
  # @see registers.INT_LOWHIGH_3
  def setShockDetectionDuration(self, duration):
    self._reg_write(registers.INT_LOWHIGH_3, duration)

  # Get Step Detection mode.
  # Returns an enum value which corresponds to current mode
  # 0 = Normal Mode
  # 1 = Sensitive Mode
  # 2 = Robust Mode
  # 3 = Unkown Mode
  # For more details on the Step Detection, see Section
  # 2.11.37 of the BMI160 Data Sheet.
  #
  # @return Current configuration of the step detector
  # @see registers.STEP_CONF_0
  # @see registers.STEP_CONF_1
  def getStepDetectionMode(self):
    ret_step_conf0 = 0
    ret_min_step_buf = 0

    ret_step_conf0 = self._reg_read(registers.STEP_CONF_0)

    # min_step_buf is the first 3 bits of RA_STEP_CONF_1
    # the 4th bit is the enable bit (step_cnt_en)
    ret_min_step_buf = self._reg_read_bits(registers.STEP_CONF_1, 0, 3)

    if ((ret_step_conf0 == registers.STEP_CONF_0_NOR) and (ret_min_step_buf == registers.STEP_CONF_1_NOR)):
      return definitions.STEP_MODE_NORMAL
    elif ((ret_step_conf0 == registers.STEP_CONF_0_SEN) and (ret_min_step_buf == registers.STEP_CONF_1_SEN)):
      return definitions.STEP_MODE_SENSITIVE
    elif ((ret_step_conf0 == registers.STEP_CONF_0_ROB) and (ret_min_step_buf == registers.STEP_CONF_1_ROB)):
      return definitions.STEP_MODE_ROBUST
    else:
      return definitions.STEP_MODE_UNKNOWN

  # Set Step Detection mode.
  # Sets the step detection mode to one of 3 predefined sensitivity settings:
  #
  #  @see definitions.STEP_MODE_NORMAL (Recommended for most applications)
  #  @see definitions.STEP_MODE_SENSITIVE
  #  @see definitions.STEP_MODE_ROBUST
  #
  # Please refer to Section 2.11.37 of the BMI160 Data Sheet for more information
  # on Step Detection configuration.
  #
  # @return Set Step Detection mode
  # @see registers.STEP_CONF_0
  # @see registers.STEP_CONF_1
  # @see BMI160StepMode
  def setStepDetectionMode(self, mode):
    step_conf0 = 0
    min_step_buf = 0

    # Applying pre-defined values suggested in data-sheet Section 2.11.37
    if definitions.STEP_MODE_NORMAL == mode:
      step_conf0 = 0x15
      min_step_buf = 0x3
    elif definitions.STEP_MODE_SENSITIVE == mode:
      step_conf0 = 0x2D
      min_step_buf = 0x0
    elif definitions.STEP_MODE_ROBUST == mode:
      step_conf0 = 0x1D
      min_step_buf = 0x7
    else:
      # Unrecognised mode option
      return

    self._reg_write(registers.STEP_CONF_0, step_conf0)
    self._reg_write_bits(registers.STEP_CONF_1, min_step_buf,
      definitions.STEP_BUF_MIN_BIT, definitions.STEP_BUF_MIN_LEN)

  # Get Step Counter enabled status.
  # Once enabled and configured correctly (@see setStepDetectionMode()), the
  # BMI160 will increment a counter for every step detected by the accelerometer.
  # To retrieve the current step count, @see getStepCount().
  #
  # For more details on the Step Counting feature, see Section
  # 2.7 of the BMI160 Data Sheet.
  #
  # @return Current Step Counter enabled status
  # @see registers.STEP_CONF_1
  # @see definitions.STEP_CNT_EN_BIT
  def getStepCountEnabled(self):
    return 0 != (self._reg_read_bits(registers.STEP_CONF_1, definitions.STEP_CNT_EN_BIT, 1))

  # Set Step Counter enabled status.
  #
  # @return Set Step Counter enabled
  # @see getStepCountEnabled()
  # @see registers.STEP_CONF_1
  # @see definitions.STEP_CNT_EN_BIT
  def setStepCountEnabled(self, enabled):
    return self._reg_write_bits(registers.STEP_CONF_1, 0x1 if enabled else 0, definitions.STEP_CNT_EN_BIT, 1)

  # Get current number of detected step movements (Step Count).
  # Returns a step counter which is incremented when step movements are detected
  # (assuming Step Detection mode and Step Counter are configured/enabled).
  #
  # @return Number of steps as an unsigned 16-bit integer
  # @see setStepCountEnabled()
  # @see setStepDetectionMode()
  # @see registers.STEP_CNT_L
  def getStepCount(self):
    buffer = [0]*2
    buffer[0] = self._reg_read(registers.STEP_CNT_L)
    buffer[1] = self._reg_read(registers.STEP_CNT_H)
    return ((buffer[1]) << 8) | buffer[0]

  # Resets the current number of detected step movements (Step Count) to 0.
  #
  # @see getStepCount()
  # @see registers.CMD
  def resetStepCount(self):
    self._reg_write(registers.CMD, commands.STEP_CNT_CLR)

  # Get motion detection event acceleration threshold.
  # This register configures the detection threshold for Motion interrupt
  # generation in the INT_MOTION[1] register. The unit of threshold is
  # dependent on the accelerometer sensitivity range (@see
  # getFullScaleAccelRange()):
  #
  # <pre>
  # Full Scale Range | LSB Resolution
  # -----------------+----------------
  # +/- 2g       |  3.91 mg/LSB
  # +/- 4g       |  7.81 mg/LSB
  # +/- 8g       | 15.63 mg/LSB
  # +/- 16g      | 31.25 mg/LSB
  # </pre>
  #
  # Motion is detected when the difference between the absolute value of
  # consecutive accelerometer measurements for the 3 axes exceeds this Motion
  # detection threshold. This condition triggers the Motion interrupt if the
  # condition is maintained for the sample count interval specified in the
  # int_anym_dur field of the INT_MOTION[0] register (@see registers.INT_MOTION_0)
  #
  # The Motion interrupt will indicate the axis and polarity of detected motion
  # in INT_STATUS[2] (@see registers.INT_STATUS_2).
  #
  # For more details on the Motion detection interrupt, see Section 2.6.1 of the
  # BMI160 Data Sheet.
  #
  # @return Current motion detection acceleration threshold value
  # @see getMotionDetectionDuration()
  # @see registers.INT_MOTION_1
  def getMotionDetectionThreshold(self):
    return self._reg_read(registers.INT_MOTION_1)

  # Set motion detection event acceleration threshold.
  # @param threshold New motion detection acceleration threshold value
  # @see getMotionDetectionThreshold()
  # @see registers.INT_MOTION_1
  def setMotionDetectionThreshold(self, threshold):
    return self._reg_write(registers.INT_MOTION_1, threshold)

  # Get motion detection event duration threshold.
  # This register configures the duration counter threshold for Motion interrupt
  # generation, as a number of consecutive samples (from 1-4). The time
  # between samples depends on the accelerometer Output Data Rate
  # (@see get_accel_rate()).
  #
  # The Motion detection interrupt is triggered when the difference between
  # samples exceeds the Any-Motion interrupt threshold for the number of
  # consecutive samples specified here.
  #
  # For more details on the Motion detection interrupt, see Section 2.6.1 of the
  # BMI160 Data Sheet.
  #
  # @return Current motion detection duration threshold value (#samples [1-4])
  # @see getMotionDetectionThreshold()
  # @see registers.INT_MOTION_0
  def getMotionDetectionDuration(self):
    return 1 + self._reg_read_bits(registers.INT_MOTION_0, definitions.ANYMOTION_DUR_BIT, definitions.ANYMOTION_DUR_LEN)

  # Set motion detection event duration threshold.
  # @param duration New motion detection duration threshold value (#samples [1-4])
  # @see getMotionDetectionDuration()
  # @see registers.INT_MOTION_0
  def setMotionDetectionDuration(self, samples):
    self._reg_write_bits(registers.INT_MOTION_0, samples - 1,
      definitions.ANYMOTION_DUR_BIT, definitions.ANYMOTION_DUR_LEN)

  # Get zero motion detection event acceleration threshold.
  # This register configures the detection threshold for Zero Motion interrupt
  # generation in the INT_MOTION[1] register. The unit of threshold is
  # dependent on the accelerometer sensitivity range
  # (@see getFullScaleAccelRange()) as follows:
  #
  # <pre>
  # Full Scale Range | LSB Resolution
  # -----------------+----------------
  # +/- 2g       |  3.91 mg/LSB
  # +/- 4g       |  7.81 mg/LSB
  # +/- 8g       | 15.63 mg/LSB
  # +/- 16g      | 31.25 mg/LSB
  # </pre>
  #
  # Zero Motion is detected when the difference between the value of
  # consecutive accelerometer measurements for each axis remains smaller than
  # this Motion detection threshold. This condition triggers the Zero Motion
  # interrupt if the condition is maintained for a time duration
  # specified in the int_slo_no_mot_dur field of the INT_MOTION[0] register (@see
  # registers.INT_MOTION_0), and clears the interrupt when the condition is
  # then absent for the same duration.
  #
  # For more details on the Zero Motion detection interrupt, see Section 2.6.9 of
  # the BMI160 Data Sheet.
  #
  # @return Current zero motion detection acceleration threshold value
  # @see getZeroMotionDetectionDuration()
  # @see registers.INT_MOTION_2
  def getZeroMotionDetectionThreshold(self):
    return self._reg_read(registers.INT_MOTION_2)

  # Set zero motion detection event acceleration threshold.
  # @param threshold New zero motion detection acceleration threshold value
  # @see getZeroMotionDetectionThreshold()
  # @see registers.INT_MOTION_2
  def setZeroMotionDetectionThreshold(self, threshold):
    self._reg_write(registers.INT_MOTION_2, threshold)

  # Get zero motion detection event duration threshold.
  # This register configures the duration time for Zero Motion interrupt
  # generation. A time range between 1.28s and 430.08s can be selected, but the
  # granularity of the timing reduces as the duration increases:
  #
  # <pre>
  # Duration       | Granularity
  # -------------------+----------------
  # [1.28 - 20.48]s    |  1.28s
  # [25.6 - 102.4]s    |  5.12s
  # [112.64 - 430.08]s | 10.24s
  # </pre>
  #
  # The Zero Motion interrupt is triggered when the Zero Motion condition is
  # maintained for the duration specified in this register.
  #
  # For more details on the Zero Motion detection interrupt, see Section 2.6.9 of
  # the BMI160 Data Sheet.
  #
  # @return Current zero motion detection duration threshold value
  #       @see BMI160ZeroMotionDuration for a list of possible values
  # @see getZeroMotionDetectionThreshold()
  # @see registers.INT_MOTION_0
  # @see BMI160ZeroMotionDuration
  def getZeroMotionDetectionDuration(self):
    return self._reg_read_bits(registers.INT_MOTION_0, definitions.NOMOTION_DUR_BIT, definitions.NOMOTION_DUR_LEN)

  # Set zero motion detection event duration threshold.
  #
  # This must be called at least once to enable zero-motion detection.
  #
  # @param duration New zero motion detection duration threshold value
  #      @see BMI160ZeroMotionDuration for a list of valid values
  # @see getZeroMotionDetectionDuration()
  # @see registers.INT_MOTION_0
  # @see BMI160ZeroMotionDuration
  def setZeroMotionDetectionDuration(self, duration):
    self._reg_write_bits(registers.INT_MOTION_0, duration,
      definitions.NOMOTION_DUR_BIT, definitions.NOMOTION_DUR_LEN)

  # Get Tap event acceleration threshold.
  # This register configures the detection threshold for Tap event
  # detection. The threshold is expressed a 5-bit unsigned integer.
  # The unit of threshold is dependent on the accelerometer
  # sensitivity range (@see getFullScaleAccelRange()):
  #
  # <pre>
  # Full Scale Range | LSB Resolution
  # -----------------+----------------
  # +/- 2g       |  62.5 mg/LSB (0 =  31.25mg)
  # +/- 4g       | 125.0 mg/LSB (0 =  62.5mg)
  # +/- 8g       | 250.0 mg/LSB (0 = 125.0mg)
  # +/- 16g      | 500.0 mg/LSB (0 = 250.0mg)
  # </pre>
  #
  # A Tap is detected as a shock event which exceeds the detection threshold for
  # a specified duration.  A threshold between 0.7g and 1.5g in the 2g
  # measurement range is suggested for typical tap detection applications.
  #
  # For more details on the Tap detection interrupt, see Section 2.6.4 of the
  # BMI160 Data Sheet.
  #
  # @return Current shock acceleration threshold value
  # @see registers.INT_TAP_1
  def getTapDetectionThreshold(self):
    return self._reg_read_bits(registers.INT_TAP_1, definitions.TAP_THRESH_BIT, definitions.TAP_THRESH_LEN)

  # Set tap event acceleration threshold.
  # @param threshold New tap acceleration threshold value
  # @see getTapDetectionThreshold()
  # @see registers.INT_TAP_1
  def setTapDetectionThreshold(self, threshold):
    self._reg_write_bits(registers.INT_TAP_1, threshold,
      definitions.TAP_THRESH_BIT, definitions.TAP_THRESH_LEN)

  # Get tap shock detection duration.
  # This register configures the duration for a tap event generation.
  #
  # The time will be returned as a 1-bit boolean, with the following
  # values (@see BMI160TapShockDuration)
  #
  # <pre>
  # duration specifier | duration threshold
  # -------------------+----------------
  #  0b0         |  50ms
  #  0b1         |  75ms
  # </pre>
  #
  # For more details on the Tap detection interrupt, see Section 2.6.4 of the
  # BMI160 Data Sheet.
  #
  # @return Current tap detection duration threshold value
  # @see registers.INT_TAP_0
  # @see BMI160TapShockDuration
  def getTapShockDuration(self):
    return 0 != (self._reg_read_bits(registers.INT_TAP_0, definitions.TAP_SHOCK_BIT, 1))

  # Set tap shock detection event duration threshold.
  #
  # @param units New tap detection duration threshold value
  # @see getTapShockDetectionDuration()
  # @see registers.INT_TAP_0
  def setTapShockDuration(self, duration):
    self._reg_write_bits(
      registers.INT_TAP_0, 0x1 if duration else 0, definitions.TAP_SHOCK_BIT, 1)

  # Get tap quiet duration threshold.
  # This register configures the quiet duration for double-tap event detection.
  #
  # The time will be returned as a 1-bit boolean, with the following
  # values (@see BMI160TapQuietDuration)
  #
  # <pre>
  # duration specifier | duration threshold
  # -------------------+----------------
  #  0b0         |  30ms
  #  0b1         |  20ms
  # </pre>
  #
  # For more details on the Tap detection interrupt, see Section 2.6.4 of the
  # BMI160 Data Sheet.
  #
  # @return Current tap quiet detection duration threshold value
  # @see registers.INT_TAP_0
  # @see BMI160TapQuietDuration
  def getTapQuietDuration(self):
    return 0 != (self._reg_read_bits(registers.INT_TAP_0, definitions.TAP_QUIET_BIT, 1))

  # Set tap quiet duration threshold.
  #
  # @param units New tap detection duration threshold value
  # @see getTapQuietDuration()
  # @see registers.INT_TAP_0
  def setTapQuietDuration(self, duration):
    self._reg_write_bits(
      registers.INT_TAP_0, 0x1 if duration else 0, definitions.TAP_QUIET_BIT, 1)

  # Get double-tap detection time window length.
  # This register configures the length of time window between 2 tap events for
  # double-tap event generation.
  #
  # The time will be returned as a 3-bit unsigned integer, with the following
  # values (@see BMI160DoubleTapDuration)
  #
  # <pre>
  # duration specifier | length of time window
  # -------------------+----------------
  #  0b000       |  50ms
  #  0b001       | 100ms
  #  0b010       | 150ms
  #  0b011       | 200ms
  #  0b100       | 250ms
  #  0b101       | 375ms
  #  0b110       | 500ms
  #  0b111       | 700ms
  # </pre>
  #
  # For more details on the Tap detection interrupt, see Section 2.6.4 of the
  # BMI160 Data Sheet.
  #
  # @return Current double-tap detection time window threshold value
  # @see registers.INT_TAP_0
  # @see BMI160DoubleTapDuration
  def getDoubleTapDetectionDuration(self):
    return self._reg_read_bits(registers.INT_TAP_0, definitions.TAP_DUR_BIT, definitions.TAP_DUR_LEN)

  # Set double-tap detection event duration threshold.
  #
  # @param duration New double-tap detection time window threshold value
  # @see getDoubleTapDetectionDuration()
  # @see registers.INT_TAP_0
  def setDoubleTapDetectionDuration(self, duration):
    self._reg_write_bits(registers.INT_TAP_0, duration,
      definitions.TAP_DUR_BIT, definitions.TAP_DUR_LEN)

  # Get Free Fall interrupt enabled status.
  # Will be set 0 for disabled, 1 for enabled.
  # @return Current interrupt enabled status
  # @see registers.INT_EN_1
  # @see definitions.LOW_G_EN_BIT
  # */
  def getIntFreefallEnabled(self):
    return 0 != (self._reg_read_bits(registers.INT_EN_1, definitions.LOW_G_EN_BIT, definitions.LOW_G_EN_LEN))

  # Set Free Fall interrupt enabled status.
  # @param enabled New interrupt enabled status
  # @see getIntFreefallEnabled()
  # @see registers.INT_EN_1
  # @see definitions.LOW_G_EN_BIT
  # */
  def setIntFreefallEnabled(self, enabled):
    self._reg_write_bits(registers.INT_EN_1, 0x1 if enabled else 0,
      durationdefinitions.LOW_G_EN_BIT, durationdefinitions.LOW_G_EN_LEN)

  # Get Shock interrupt enabled status.
  # Will be set 0 for disabled, 1 for enabled.
  # @return Current interrupt enabled status
  # @see registers.INT_EN_1
  # @see definitions.HIGH_G_EN_BIT
  # */
  def getIntShockEnabled(self):
    return 0 != (self._reg_read_bits(registers.INT_EN_1, definitions.HIGH_G_EN_BIT, definitions.HIGH_G_EN_LEN))

  # Set Shock interrupt enabled status.
  # @param enabled New interrupt enabled status
  # @see getIntShockEnabled()
  # @see registers.INT_EN_1
  # @see definitions.HIGH_G_EN_BIT
  # */
  def setIntShockEnabled(self, enabled):
    self._reg_write_bits(registers.INT_EN_1, 0x7 if enabled else 0x0,
      definitions.HIGH_G_EN_BIT, definitions.HIGH_G_EN_LEN)

  # Get Step interrupt enabled status.
  # Will be set 0 for disabled, 1 for enabled.
  # @return Current interrupt enabled status
  # @see registers.INT_EN_2
  # @see definitions.STEP_EN_BIT
  # */
  def getIntStepEnabled(self):
    return 0 != (self._reg_read_bits(registers.INT_EN_2, definitions.STEP_EN_BIT, 1))

  # Set Step interrupt enabled status.
  # @param enabled New interrupt enabled status
  # @see getIntStepEnabled()
  # @see registers.INT_EN_2
  # @see definitions.STEP_EN_BIT
  # */
  def setIntStepEnabled(self, enabled):
    self._reg_write_bits(
      registers.INT_EN_2, 0x1 if enabled else 0x0, definitions.STEP_EN_BIT, 1)

  # Get Motion Detection interrupt enabled status.
  # Will be set 0 for disabled, 1 for enabled.
  # @return Current interrupt enabled status
  # @see registers.INT_EN_0
  # @see definitions.ANYMOTION_EN_BIT
  # */
  def getIntMotionEnabled(self):
    return 0 != (self._reg_read_bits(registers.INT_EN_0, definitions.ANYMOTION_EN_BIT, definitions.ANYMOTION_EN_LEN))

  # Set Motion Detection interrupt enabled status.
  # @param enabled New interrupt enabled status
  # @see getIntMotionEnabled()
  # @see registers.INT_EN_0
  # @see definitions.ANYMOTION_EN_BIT
  # */
  def setIntMotionEnabled(self, enabled):
    # Enable for all 3 axes
    self._reg_write_bits(registers.INT_EN_0, 0x7 if enabled else 0x0,
      definitions.ANYMOTION_EN_BIT, definitions.ANYMOTION_EN_LEN)

  # Get Zero Motion Detection interrupt enabled status.
  # Will be set 0 for disabled, 1 for enabled.
  # @return Current interrupt enabled status
  # @see registers.INT_EN_2
  # @see definitions.NOMOTION_EN_BIT
  # */
  def getIntZeroMotionEnabled(self):
    return 0 != (self._reg_read_bits(registers.INT_EN_2, definitions.NOMOTION_EN_BIT, definitions.NOMOTION_EN_LEN))

  # Set Zero Motion Detection interrupt enabled status.
  # @param enabled New interrupt enabled status
  # @see getIntZeroMotionEnabled()
  # @see registers.INT_EN_2
  # @see definitions.NOMOTION_EN_BIT
  # @see registers.INT_MOTION_3
  # */
  def setIntZeroMotionEnabled(self, enabled):
    if (enabled):
      # Select No-Motion detection mode
      self._reg_write_bits(registers.INT_MOTION_3, 0x1,
        definitions.NOMOTION_SEL_BIT, definitions.NOMOTION_SEL_LEN)
    # Enable for all 3 axes
    self._reg_write_bits(registers.INT_EN_2, 0x7 if enabled else 0x0,
      definitions.NOMOTION_EN_BIT, definitions.NOMOTION_EN_LEN)

  # Get Tap Detection interrupt enabled status.
  # Will be set 0 for disabled, 1 for enabled.
  # @return Current interrupt enabled status
  # @see registers.INT_EN_0
  # @see definitions.S_TAP_EN_BIT
  # */
  def getIntTapEnabled(self):
    return 0 != (self._reg_read_bits(registers.INT_EN_0, definitions.S_TAP_EN_BIT, 1))

  # Set Tap Detection interrupt enabled status.
  # @param enabled New interrupt enabled status
  # @see getIntTapEnabled()
  # @see registers.INT_EN_0
  # @see definitions.S_TAP_EN_BIT
  # */
  def setIntTapEnabled(self, enabled):
    self._reg_write_bits(
      registers.INT_EN_0, 0x1 if enabled else 0, definitions.S_TAP_EN_BIT, 1)

  # Get Tap Detection interrupt enabled status.
  # Will be set 0 for disabled, 1 for enabled.
  # @return Current interrupt enabled status
  # @see registers.INT_EN_0
  # @see definitions.D_TAP_EN_BIT
  # */
  def getIntDoubleTapEnabled(self):
    return 0 != (self._reg_read_bits(registers.INT_EN_0, definitions.D_TAP_EN_BIT, 1))

  # Set Tap Detection interrupt enabled status.
  # @param enabled New interrupt enabled status
  # @see getIntTapEnabled()
  # @see registers.INT_EN_0
  # @see definitions.D_TAP_EN_BIT
  # */
  def setIntDoubleTapEnabled(self, enabled):
    self._reg_write_bits(
      registers.INT_EN_0, 0x1 if enabled else 0, definitions.D_TAP_EN_BIT, 1)

  # Get FIFO Buffer Full interrupt enabled status.
  # Will be set 0 for disabled, 1 for enabled.
  # @return Current interrupt enabled status
  # @see registers.INT_EN_1
  # @see definitions.FFULL_EN_BIT
  # */
  def getIntFIFOBufferFullEnabled(self):
    return 0 != (self._reg_read_bits(registers.INT_EN_1, definitions.FFULL_EN_BIT, 1))

  # Set FIFO Buffer Full interrupt enabled status.
  # @param enabled New interrupt enabled status
  # @see getIntFIFOBufferFullEnabled()
  # @see registers.INT_EN_1
  # @see definitions.FFULL_EN_BIT
  # */
  def setIntFIFOBufferFullEnabled(self, enabled):
    self._reg_write_bits(
      registers.INT_EN_1, 0x1 if enabled else 0x0, definitions.FFULL_EN_BIT, 1)

  # Get Data Ready interrupt enabled setting.
  # This event occurs each time a write operation to all of the sensor registers
  # has been completed. Will be set 0 for disabled, 1 for enabled.
  # @return Current interrupt enabled status
  # @see registers.INT_EN_1
  # @see definitions.DRDY_EN_BIT
  def getIntDataReadyEnabled(self):
    return 0 != (self._reg_read_bits(registers.INT_EN_1, definitions.DRDY_EN_BIT, 1))

  # Set Data Ready interrupt enabled status.
  # @param enabled New interrupt enabled status
  # @see getIntDataReadyEnabled()
  # @see registers.INT_EN_1
  # @see definitions.DRDY_EN_BIT
  def setIntDataReadyEnabled(self, enabled):
    self._reg_write_bits(
      registers.INT_EN_1, 0x1 if enabled else 0x0, definitions.DRDY_EN_BIT, 1)

  # Get accelerometer FIFO enabled value.
  # When set to 1, this bit enables accelerometer data samples to be
  # written into the FIFO buffer.
  # @return Current accelerometer FIFO enabled value
  # @see registers.FIFO_CONFIG_1
  def getAccelFIFOEnabled(self):
    return 0 != (self._reg_read_bits(registers.FIFO_CONFIG_1, definitions.FIFO_ACC_EN_BIT, 1))

  # Set accelerometer FIFO enabled value.
  # @param enabled New accelerometer FIFO enabled value
  # @see getAccelFIFOEnabled()
  # @see registers.FIFO_CONFIG_1
  def setAccelFIFOEnabled(self, enabled):
    self._reg_write_bits(
      registers.FIFO_CONFIG_1, 0x1 if enabled else 0, definitions.FIFO_ACC_EN_BIT, 1)

  # Get gyroscope FIFO enabled value.
  # When set to 1, this bit enables gyroscope data samples to be
  # written into the FIFO buffer.
  # @return Current gyroscope FIFO enabled value
  # @see registers.FIFO_CONFIG_1
  def getGyroFIFOEnabled(self):
    return 0 != (self._reg_read_bits(registers.FIFO_CONFIG_1, definitions.FIFO_GYR_EN_BIT, 1))

  # Set gyroscope FIFO enabled value.
  # @param enabled New gyroscope FIFO enabled value
  # @see getGyroFIFOEnabled()
  # @see registers.FIFO_CONFIG_1
  def setGyroFIFOEnabled(self, enabled):
    self._reg_write_bits(
      registers.FIFO_CONFIG_1, 0x1 if enabled else 0, definitions.FIFO_GYR_EN_BIT, 1)

  # Get current FIFO buffer size.
  # This value indicates the number of bytes stored in the FIFO buffer. This
  # number is in turn the number of bytes that can be read from the FIFO buffer.
  #
  # In "headerless" FIFO mode, it is directly proportional to the number of
  # samples available given the set of sensor data bound to be stored in the
  # FIFO. See @ref getFIFOHeaderModeEnabled().
  #
  # @return Current FIFO buffer size
  # @see registers.FIFO_LENGTH_0
  def getFIFOCount(self):
    buffer = [0]*2
    buffer[0] = registers.FIFO_LENGTH_0
    return ((buffer[1]) << 8) | buffer[0]

  # Reset the FIFO.
  # This command clears all data in the FIFO buffer.  It is recommended
  # to invoke this after reconfiguring the FIFO.
  #
  # @see registers.CMD
  # @see commands.FIFO_FLUSH
  def resetFIFO(self):
    self._reg_write(registers.CMD, commands.FIFO_FLUSH)

  # Reset the Interrupt controller.
  # This command clears interrupt status registers and latched interrupts.
  #
  # @see registers.CMD
  # @see commands.FIFO_FLUSH
  def resetInterrupt(self):
    self._reg_write(registers.CMD, commands.INT_RESET)

  # Get FIFO Header-Mode enabled status.
  # When this bit is set to 0, the FIFO header-mode is disabled, and frames
  # read from the FIFO will be headerless (raw sensor data only).
  # When this bit is set to 1, the FIFO header-mode is enabled, and frames
  # read from the FIFO will include headers.
  #
  # For more information on the FIFO modes and data formats, please refer
  # to Section 2.5 of the BMI160 Data Sheet.
  #
  # @return Current FIFO Header-Mode enabled status
  # @see registers.FIFO_CONFIG_1
  # @see definitions.FIFO_HEADER_EN_BIT
  def getFIFOHeaderModeEnabled(self):
    return 0 != (self._reg_read_bits(registers.FIFO_CONFIG_1, definitions.FIFO_HEADER_EN_BIT, 1))

  # Set FIFO Header-Mode enabled status.
  # @param enabled New FIFO Header-Mode enabled status
  # @see getFIFOHeaderModeEnabled()
  # @see registers.FIFO_CONFIG_1
  # @see definitions.FIFO_HEADER_EN_BIT
  def setFIFOHeaderModeEnabled(self, enabled):
    self._reg_write_bits(
      registers.FIFO_CONFIG_1, 0x1 if enabled else 0, definitions.FIFO_HEADER_EN_BIT, 1)

  # Get data frames from FIFO buffer.
  # This register is used to read and write data frames from the FIFO buffer.
  # Data is written to the FIFO in order of DATA register number (from lowest
  # to highest) corresponding to the FIFO data sources enabled (@see
  # getGyroFIFOEnabled() and getAccelFIFOEnabled()).
  #
  # The data frame format depends on the enabled data sources and also on
  # the FIFO header-mode setting (@see getFIFOHeaderModeEnabled()).
  #
  # It is strongly recommended, where possible, to read whole frames from the
  # FIFO.  Partially-read frames will be repeated until fully read out.
  #
  # If the FIFO buffer has filled to the powhere subsequent writes may
  # cause data loss, the status bit ffull_is automatically set to 1. This bit
  # is located in INT_STATUS[1]. When the FIFO buffer has overflowed, the oldest
  # data will be lost and new data will be written to the FIFO.
  #
  # If the FIFO buffer is empty, reading this register will return a magic number
  # (@see definitions.FIFO_DATA_INVALID) until new data is available. The user should
  # check FIFO_LENGTH to ensure that the FIFO buffer is not read when empty (see
  # @getFIFOCount()).
  #
  # @return Data frames from FIFO buffer
  def getFIFOBytes(self, *data, length):
    # TODO fix here
    if (length):
      data[0] = registers.FIFO_DATA

  # Get full set of interrupt status bits from INT_STATUS[0] register.
  # Interrupts are typically cleared automatically.
  # Please refer to the BMI160 Data Sheet for more information.
  # @return Current interrupt status
  # @see registers.INT_STATUS_0
  def getIntStatus0(self):
    return self._reg_read(registers.INT_STATUS_0)

  # Get full set of interrupt status bits from INT_STATUS[1] register.
  # Interrupts are typically cleared automatically.
  # Please refer to the BMI160 Data Sheet for more information.
  # @return Current interrupt status
  # @see registers.INT_STATUS_1
  def getIntStatus1():
    return self._reg_read(registers.INT_STATUS_1)

  # Get full set of interrupt status bits from INT_STATUS[2] register.
  # Interrupts are typically cleared automatically.
  # Please refer to the BMI160 Data Sheet for more information.
  # @return Current interrupt status
  # @see registers.INT_STATUS_2
  def getIntStatus2(self):
    return self._reg_read(registers.INT_STATUS_2)

  # Get full set of interrupt status bits from INT_STATUS[3] register.
  # Interrupts are typically cleared automatically.
  # Please refer to the BMI160 Data Sheet for more information.
  # @return Current interrupt status
  # @see registers.INT_STATUS_3
  def getIntStatus3(self):
    return self._reg_read(registers.INT_STATUS_3)

  # Get Free Fall interrupt status.
  # This bit automatically sets to 1 when a Free Fall condition
  # is present, and clears when the condition is no longer present.
  #
  # For more details on the Free-Fall (Low-G) detection interrupt, see Section
  # 2.6.7 of the BMI160 Data Sheet.
  #
  # @return Current interrupt status
  # @see registers.INT_STATUS_1
  # @see definitions.LOW_G_INT_BIT
  def getIntFreefallStatus(self):
    return 0 != (self._reg_read_bits(registers.INT_STATUS_1, definitions.LOW_G_INT_BIT, 1))

  # Get Tap Detection interrupt status.
  # This bit automatically sets to 1 when a Tap Detection condition
  # is present, and clears when the condition is no longer present.
  #
  # For more details on the Tap detection interrupt, see Section 2.6.4 of the
  # BMI160 Data Sheet.
  #
  # @return Current interrupt status
  # @see registers.INT_STATUS_0
  # @see definitions.S_TAP_INT_BIT
  def getIntTapStatus(self):
    return 0 != (self._reg_read_bits(registers.INT_STATUS_0, definitions.S_TAP_INT_BIT, 1))

  # Get Double-Tap Detection interrupt status.
  # This bit automatically sets to 1 when a Double-Tap Detection condition
  # is present, and clears when the condition is no longer present.
  #
  # For more details on the Double-Tap detection interrupt, see Section 2.6.4 of the
  # BMI160 Data Sheet.
  #
  # @return Current interrupt status
  # @see registers.INT_STATUS_0
  # @see definitions.D_TAP_INT_BIT
  def getIntDoubleTapStatus(self):
    return 0 != (self._reg_read_bits(registers.INT_STATUS_0, definitions.D_TAP_INT_BIT, 1))

  # Get Shock interrupt status.
  # This bit automatically sets to 1 when a Shock (High-G) Detection condition
  # is present, and clears when the condition is no longer present.
  #
  # For more details on the Shock (High-G) detection interrupt, see Section
  # 2.6.8 of the BMI160 Data Sheet.
  #
  # @return Current interrupt status
  # @see registers.INT_STATUS_1
  # @see definitions.HIGH_G_INT_BIT
  def getIntShockStatus(self):
    return 0 != (self._reg_read_bits(registers.INT_STATUS_1, definitions.HIGH_G_INT_BIT, 1))

  # Check if shock interrupt was triggered by negative X-axis motion
  # @return Shock detection status
  # @see registers.INT_STATUS_3
  # @see definitions.HIGH_G_SIGN_BIT
  # @see definitions.HIGH_G_1ST_X_BIT
  def getXNegShockDetected(self):
    status = self._reg_read(registers.INT_STATUS_3)
    return 0 != ((status & (1 << definitions.HIGH_G_SIGN_BIT)) and (status & (1 << definitions.HIGH_G_1ST_X_BIT)))

  # Check if shock interrupt was triggered by positive X-axis motion
  # @return Shock detection status
  # @see registers.INT_STATUS_3
  # @see definitions.HIGH_G_SIGN_BIT
  # @see definitions.HIGH_G_1ST_X_BIT
  def getXPosShockDetected(self):
    status = self._reg_read(registers.INT_STATUS_3)
    return 0 != (not (status & (1 << definitions.HIGH_G_SIGN_BIT)) and (status & (1 << definitions.HIGH_G_1ST_X_BIT)))

  # Check if shock interrupt was triggered by negative Y-axis motion
  # @return Shock detection status
  # @see registers.INT_STATUS_3
  # @see definitions.HIGH_G_SIGN_BIT
  # @see definitions.HIGH_G_1ST_Y_BIT
  def getYNegShockDetected(self):
    status = self._reg_read(registers.INT_STATUS_3)
    return 0 != ((status & (1 << definitions.HIGH_G_SIGN_BIT)) and (status & (1 << definitions.HIGH_G_1ST_Y_BIT)))

  # Check if shock interrupt was triggered by positive Y-axis motion
  # @return Shock detection status
  # @see registers.INT_STATUS_3
  # @see definitions.HIGH_G_SIGN_BIT
  # @see definitions.HIGH_G_1ST_Y_BIT
  def getYPosShockDetected(self):
    status = self._reg_read(registers.INT_STATUS_3)
    return 0 != (not (status & (1 << definitions.HIGH_G_SIGN_BIT)) and (status & (1 << definitions.HIGH_G_1ST_Y_BIT)))

  # Check if shock interrupt was triggered by negative Z-axis motion
  # @return Shock detection status
  # @see registers.INT_STATUS_3
  # @see definitions.HIGH_G_SIGN_BIT
  # @see definitions.HIGH_G_1ST_Z_BIT
  def getZNegShockDetected(self):
    status = self._reg_read(registers.INT_STATUS_3)
    return 0 != ((status & (1 << definitions.HIGH_G_SIGN_BIT)) and (status & (1 << definitions.HIGH_G_1ST_Z_BIT)))

  # Check if shock interrupt was triggered by positive Z-axis motion
  # @return Shock detection status
  # @see registers.INT_STATUS_3
  # @see definitions.HIGH_G_SIGN_BIT
  # @see definitions.HIGH_G_1ST_Z_BIT
  def getZPosShockDetected(self):
    status = self._reg_read(registers.INT_STATUS_3)
    return 0 != (not (status & (1 << definitions.HIGH_G_SIGN_BIT)) and (status & (1 << definitions.HIGH_G_1ST_Z_BIT)))

  # Get Step interrupt status.
  # This bit automatically sets to 1 when a Step Detection condition
  # is present, and clears when the condition is no longer present.
  #
  # For more details on the Step detection interrupt, see Section
  # 2.6.3 of the BMI160 Data Sheet.
  #
  # @return Current interrupt status
  # @see registers.INT_STATUS_0
  # @see definitions.STEP_INT_BIT
  def getIntStepStatus(self):
    return 0 != (self._reg_read_bits(registers.INT_STATUS_0, definitions.STEP_INT_BIT, 1))

  # Get Motion Detection interrupt status.
  # This bit automatically sets to 1 when a Motion Detection condition
  # is present, and clears when the condition is no longer present.
  #
  # For more details on the Motion detection interrupt, see Section 2.6.1 of the
  # BMI160 Data Sheet.
  #
  # @return Current interrupt status
  # @see registers.INT_STATUS_0
  # @see definitions.ANYMOTION_INT_BIT
  def getIntMotionStatus(self):
    return 0 != (self._reg_read_bits(registers.INT_STATUS_0, definitions.ANYMOTION_INT_BIT, 1))

  # Check if motion interrupt was triggered by negative X-axis motion
  # @return Motion detection status
  # @see registers.INT_STATUS_2
  # @see definitions.ANYMOTION_SIGN_BIT
  # @see definitions.ANYMOTION_1ST_X_BIT
  def getXNegMotionDetected(self):
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != ((status & (1 << definitions.ANYMOTION_SIGN_BIT)) and (status & (1 << definitions.ANYMOTION_1ST_X_BIT)))

  # Check if motion interrupt was triggered by positive X-axis motion
  # @return Motion detection status
  # @seefrom struct import unpack registers.INT_STATUS_2
  # @see definitions.ANYMOTION_SIGN_BIT
  # @see definitions.ANYMOTION_1ST_X_BIT
  def getXPosMotionDetected(self):
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != (not (status & (1 << definitions.ANYMOTION_SIGN_BIT)) and (status & (1 << definitions.ANYMOTION_1ST_X_BIT)))

  # Check if motion interrupt was triggered by negative Y-axis motion
  # @return Motion detection status
  # @see registers.INT_STATUS_2
  # @see definitions.ANYMOTION_SIGN_BIT
  # @see definitions.ANYMOTION_1ST_Y_BIT
  def getYNegMotionDetected(self):
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != ((status & (1 << definitions.ANYMOTION_SIGN_BIT)) and (status & (1 << definitions.ANYMOTION_1ST_Y_BIT)))

  # Check if motion interrupt was triggered by positive Y-axis motion
  # @return Motion detection status
  # @see registers.INT_STATUS_2
  # @see definitions.ANYMOTION_SIGN_BIT
  # @see definitions.ANYMOTION_1ST_Y_BIT
  def getYPosMotionDetected(self):
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != (not (status & (1 << definitions.ANYMOTION_SIGN_BIT)) and (status & (1 << definitions.ANYMOTION_1ST_Y_BIT)))

  # Check if motion interrupt was triggered by negative Z-axis motion
  # @return Motion detection status
  # @see registers.INT_STATUS_2
  # @see definitions.ANYMOTION_SIGN_BIT
  # @see definitions.ANYMOTION_1ST_Z_BIT
  def getZNegMotionDetected(self):
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != ((status & (1 << definitions.ANYMOTION_SIGN_BIT)) and (status & (1 << definitions.ANYMOTION_1ST_Z_BIT)))

  # Check if motion interrupt was triggered by positive Z-axis motion
  # @return Motion detection status
  # @see registers.INT_STATUS_2
  # @see definitions.ANYMOTION_SIGN_BIT
  # @see definitions.ANYMOTION_1ST_Z_BIT
  def getZPosMotionDetected():
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != (not (status & (1 << definitions.ANYMOTION_SIGN_BIT)) and (status & (1 << definitions.ANYMOTION_1ST_Z_BIT)))

  # Check if tap interrupt was triggered by negative X-axis tap
  # @return Tap detection status
  # @see registers.INT_STATUS_2
  # @see definitions.TAP_SIGN_BIT
  # @see definitions.TAP_1ST_X_BIT
  def getXNegTapDetected(self):
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != ((status & (1 << definitions.TAP_SIGN_BIT)) and (status & (1 << definitions.TAP_1ST_X_BIT)))

  # Check if tap interrupt was triggered by positive X-axis tap
  # @return Tap detection status
  # @see registers.INT_STATUS_2
  # @see definitions.TAP_SIGN_BIT
  # @see definitions.TAP_1ST_X_BIT
  def getXPosTapDetected(self):
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != (not (status & (1 << definitions.TAP_SIGN_BIT)) and (status & (1 << definitions.TAP_1ST_X_BIT)))

  # Check if tap interrupt was triggered by negative Y-axis tap
  # @return Tap detection status
  # @see registers.INT_STATUS_2
  # @see definitions.TAP_SIGN_BIT
  # @see definitions.TAP_1ST_Y_BIT
  def getYNegTapDetected(self):
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != ((status & (1 << definitions.TAP_SIGN_BIT)) and (status & (1 << definitions.TAP_1ST_Y_BIT)))

  # Check if tap interrupt was triggered by positive Y-axis tap
  # @return Tap detection status
  # @see registers.INT_STATUS_2
  # @see definitions.TAP_SIGN_BIT
  # @see definitions.TAP_1ST_Y_BIT
  def getYPosTapDetected(self):
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != (not (status & (1 << definitions.TAP_SIGN_BIT)) and (status & (1 << definitions.TAP_1ST_Y_BIT)))

  # Check if tap interrupt was triggered by negative Z-axis tap
  # @return Tap detection status
  # @see registers.INT_STATUS_2
  # @see definitions.TAP_SIGN_BIT
  # @see definitions.TAP_1ST_Z_BIT
  def getZNegTapDetected(self):
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != ((status & (1 << definitions.TAP_SIGN_BIT)) and (status & (1 << definitions.TAP_1ST_Z_BIT)))

  # Check if tap interrupt was triggered by positive Z-axis tap
  # @return Tap detection status
  # @see registers.INT_STATUS_2
  # @see definitions.TAP_SIGN_BIT
  # @see definitions.TAP_1ST_Z_BIT
  def getZPosTapDetected(self):
    status = self._reg_read(registers.INT_STATUS_2)
    return 0 != (not (status & (1 << definitions.TAP_SIGN_BIT)) and (status & (1 << definitions.TAP_1ST_Z_BIT)))

  # Get Zero Motion Detection interrupt status.
  # This bit automatically sets to 1 when a Zero Motion Detection condition
  # is present, and clears when the condition is no longer present.
  #
  # For more details on the Motion detection interrupt, see Section 2.6.9 of the
  # BMI160 Data Sheet.
  #
  # @return Current interrupt status
  # @see registers.INT_STATUS_1
  # @see definitions.NOMOTION_INT_BIT
  def getIntZeroMotionStatus(self):
    return 0 != (self._reg_read_bits(registers.INT_STATUS_1, definitions.NOMOTION_INT_BIT, 1))

  # Get FIFO Buffer Full interrupt status.
  # This bit automatically sets to 1 when a FIFO Full condition has been
  # generated. The bit clears to 0 when the FIFO is not full.
  # @return Current interrupt status
  # @see registers.INT_STATUS_1
  # @see definitions.FFULL_INT_BIT
  def getIntFIFOBufferFullStatus(self):
    return 0 != (self._reg_read_bits(registers.INT_STATUS_1, definitions.FFULL_INT_BIT, 1))

  # Get Data Ready interrupt status.
  # This bit automatically sets to 1 when a Data Ready interrupt has been
  # generated. The bit clears to 0 after the data registers have been read.
  # @return Current interrupt status
  # @see registers.INT_STATUS_1
  # @see definitions.FFULL_INT_BIT
  def getIntDataReadyStatus(self):
    return 0 != (self._reg_read_bits(registers.INT_STATUS_1, definitions.DRDY_INT_BIT, 1))

  # Get interrupt logic level mode.
  # Will be set 0 for active-high, 1 for active-low.
  # @return Current interrupt mode (0=active-high, 1=active-low)
  # @see registers.INT_OUT_CTRL
  # @see definitions.INT1_LVL
  def getInterruptMode(self):
    return not (self._reg_read_bits(registers.INT_OUT_CTRL, definitions.INT1_LVL, 1))

  # Set interrupt logic level mode.
  # @param mode New interrupt mode (0=active-high, 1=active-low)
  # @see getInterruptMode()
  # @see registers.INT_OUT_CTRL
  # @see definitions.INT1_LVL
  def setInterruptMode(self, mode):
    self._reg_write_bits(registers.INT_OUT_CTRL,
      0x0 if mode else 0x1, definitions.INT1_LVL, 1)

  # Get interrupt drive mode.
  # Will be set 0 for push-pull, 1 for open-drain.
  # @return Current interrupt drive mode (0=push-pull, 1=open-drain)
  # @see registers.INT_OUT_CTRL
  # @see definitions.INT1_OD
  def getInterruptDrive(self):
    return 0 != (self._reg_read_bits(registers.INT_OUT_CTRL, definitions.INT1_OD, 1))

  # Set interrupt drive mode.
  # @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
  # @see getInterruptDrive()
  # @see MPU6050_RA_INT_PIN_CFG
  # @see MPU6050_INTCFG_INT_OPEN_BIT
  def setInterruptDrive(self, drive):
    self._reg_write_bits(registers.INT_OUT_CTRL,
      0x1 if drive else 0x0, definitions.INT1_OD, 1)

  # Get interrupt latch mode.  The following options are available:
  #
  # <pre>
  # Latch Mode    | Interrupt Latching
  # --------------+-------------------------
  # 0       | non-latched
  # 1       | temporary, 312.5us pulse
  # 2       | temporary,   625us pulse
  # 3       | temporary,  1.25ms pulse
  # 4       | temporary,   2.5ms pulse
  # 5       | temporary,     5ms pulse
  # 6       | temporary,    10ms pulse
  # 7       | temporary,    20ms pulse
  # 8       | temporary,    40ms pulse
  # 9       | temporary,    80ms pulse
  # 10      | temporary,   160ms pulse
  # 11      | temporary,   320ms pulse
  # 12      | temporary,   640ms pulse
  # 13      | temporary,  1.28s pulse
  # 14      | temporary,  2.56s pulse
  # 15      | latched until cleared (@see resetInterrupt())
  # </pre>
  #
  # Note that latching does not apply to the following interrupt sources:
  # - Data Ready
  # - Orientation (including Flat) detection
  #
  # @return Current latch mode
  # @see registers.INT_LATCH
  # @see BMI160InterruptLatchMode
  def getInterruptLatch(self):
    return self._reg_read_bits(registers.INT_LATCH, definitions.LATCH_MODE_BIT, definitions.LATCH_MODE_LEN)

  # Set interrupt latch mode.
  # @param latch New latch mode
  # @see getInterruptLatch()
  # @see registers.INT_LATCH
  # @see BMI160InterruptLatchMode
  def setInterruptLatch(self, mode):
    self._reg_write_bits(registers.INT_LATCH, mode,
      definitions.LATCH_MODE_BIT, definitions.LATCH_MODE_LEN)

  # Get interrupt enabled status.
  # @return Current interrupt enabled status
  # @see registers.INT_OUT_CTRL
  # @see definitions.INT1_OUTPUT_EN
  # */
  def getIntEnabled(self):
    return 0 != (self._reg_read_bits(registers.INT_OUT_CTRL, definitions.INT1_OUTPUT_EN, 1))

  # Set interrupt enabled status.
  # @param enabled New interrupt enabled status
  # @see registers.INT_OUT_CTRL
  # @see definitions.INT1_OUTPUT_EN
  # */
  def setIntEnabled(self, enabled):
    self._reg_write_bits(
      registers.INT_OUT_CTRL, 0x1 if enabled else 0, definitions.INT1_OUTPUT_EN, 1)

  # Get raw 6-axis motion sensor readings (accel/gyro).
  # Retrieves all currently available motion sensor values.
  # @return gx 16-bit signed integer container for gyroscope X-axis value
  # @return gy 16-bit signed integer container for gyroscope Y-axis value
  # @return gz 16-bit signed integer container for gyroscope Z-axis value
  # @return ax 16-bit signed integer container for accelerometer X-axis value
  # @return ay 16-bit signed integer container for accelerometer Y-axis value
  # @return az 16-bit signed integer container for accelerometer Z-axis value
  # @see getAcceleration()
  # @see getRotation()
  # @see registers.GYRO_X_L
  def getMotion6(self):
    raw = self._regs_read(registers.GYRO_X_L, 12)
    vals = unpack('<6h', bytes(raw))
    return vals

  # Get 3-axis accelerometer readings.
  # These registers store the most recent accelerometer measurements.
  # Accelerometer measurements are written to these registers at the Output Data Rate
  # as configured by @see get_accel_rate()
  #
  # The accelerometer measurement registers, along with the temperature
  # measurement registers, gyroscope measurement registers, and external sensor
  # data registers, are composed of two sets of registers: an internal register
  # set and a user-facing read register set.
  #
  # The data within the accelerometer sensors' internal register set is always
  # updated at the Output Data Rate. Meanwhile, the user-facing read register set
  # duplicates the internal register set's data values whenever the serial
  # interface is idle. This guarantees that a burst read of sensor registers will
  # read measurements from the same sampling instant. Note that if burst reads
  # are not used, the user is responsible for ensuring a set of single byte reads
  # correspond to a single sampling instant by checking the Data Ready interrupt.
  #
  # Each 16-bit accelerometer measurement has a full scale configured by
  # @setFullScaleAccelRange. For each full scale setting, the accelerometers'
  # sensitivity per LSB is shown in the table below:
  #
  # <pre>
  # Full Scale Range | LSB Sensitivity
  # -----------------+----------------
  # +/- 2g       | 8192 LSB/mg
  # +/- 4g       | 4096 LSB/mg
  # +/- 8g       | 2048 LSB/mg
  # +/- 16g      | 1024 LSB/mg
  # </pre>
  #
  # @param x 16-bit signed integer container for X-axis acceleration
  # @param y 16-bit signed integer container for Y-axis acceleration
  # @param z 16-bit signed integer container for Z-axis acceleration
  # @see registers.ACCEL_X_L
  def getAcceleration(self):
    raw = self._regs_read(registers.ACCEL_X_L, 6)
    vals = unpack('<3h', bytes(raw))
    return vals

  # Get X-axis accelerometer reading.
  # @return X-axis acceleration measurement in 16-bit 2's complement format
  # @see getMotion6()
  # @see registers.ACCEL_X_L
  def getAccelerationX(self):
    raw = self._regs_read(registers.ACCEL_X_L, 2)
    val = unpack('<h', bytes(raw))
    return val

  # Get Y-axis accelerometer reading.
  # @return Y-axis acceleration measurement in 16-bit 2's complement format
  # @see getMotion6()
  # @see registers.ACCEL_Y_L
  def getAccelerationY(self):
    raw = self._regs_read(registers.ACCEL_XYL, 2)
    val = unpack('<h', bytes(raw))
    return val

  # Get Z-axis accelerometer reading.
  # @return Z-axis acceleration measurement in 16-bit 2's complement format
  # @see getMotion6()
  # @see registers.ACCEL_Z_L
  def getAccelerationZ(self):
    raw = self._regs_read(registers.ACCEL_Z_L, 2)
    val = unpack('<h', bytes(raw))
    return val

  # Get current internal temperature as a signed 16-bit integer.
  #  The resolution is typically 1/2^9 degrees Celcius per LSB, at an
  #  offset of 23 degrees Celcius.  For example:
  #
  # <pre>
  # Value    | Temperature
  # ---------+----------------
  # 0x7FFF   | 87 - 1/2^9 degrees C
  # ...    | ...
  # 0x0000   | 23 degrees C
  # ...    | ...
  # 0x8001   | -41 + 1/2^9 degrees C
  # 0x8000   | Invalid
  #
  # @return Temperature reading in 16-bit 2's complement format
  # @see registers.TEMP_L
  def getTemperature(self):
    raw = self._regs_read(registers.TEMP_L, 2)
    val = unpack('<h', bytes(raw))[0]
    deg_c = (val*0.001953185)+23
    return deg_c

  # Get 3-axis gyroscope readings.
  # These gyroscope measurement registers, along with the accelerometer
  # measurement registers, temperature measurement registers, and external sensor
  # data registers, are composed of two sets of registers: an internal register
  # set and a user-facing read register set.
  # The data within the gyroscope sensors' internal register set is always
  # updated at the Output Data Rate. Meanwhile, the user-facing read register set
  # duplicates the internal register set's data values whenever the serial
  # interface is idle. This guarantees that a burst read of sensor registers will
  # read measurements from the same sampling instant. Note that if burst reads
  # are not used, the user is responsible for ensuring a set of single byte reads
  # correspond to a single sampling instant by checking the Data Ready interrupt.
  #
  # Each 16-bit gyroscope measurement has a full scale configured by
  # @setFullScaleGyroRange(). For each full scale setting, the gyroscopes'
  # sensitivity per LSB is shown in the table below:
  #
  # <pre>
  # Full Scale Range   | LSB Sensitivity
  # -------------------+----------------
  # +/- 125  degrees/s | 262.4 LSB/deg/s
  # +/- 250  degrees/s | 131.2 LSB/deg/s
  # +/- 500  degrees/s | 65.5  LSB/deg/s
  # +/- 1000 degrees/s | 32.8  LSB/deg/s
  # +/- 2000 degrees/s | 16.4  LSB/deg/s
  # </pre>
  #
  # @param x 16-bit signed integer container for X-axis rotation
  # @param y 16-bit signed integer container for Y-axis rotation
  # @param z 16-bit signed integer container for Z-axis rotation
  # @see getMotion6()
  # @see registers.GYRO_X_L
  def getRotation(self):
    raw = self._regs_read(registers.GYRO_X_L, 6)
    vals = unpack('<3h', bytes(raw))
    return (vals[0], vals[1], vals[2])

  # Get X-axis gyroscope reading.
  # @return X-axis rotation measurement in 16-bit 2's complement format
  # @see getMotion6()
  # @see registers.GYRO_X_L
  def getRotationX(self):
    raw = self._regs_read(registers.GYRO_X_L, 2)
    val = unpack('<h', bytes(raw))
    return val

  # Get Y-axis gyroscope reading.
  # @return Y-axis rotation measurement in 16-bit 2's complement format
  # @see getMotion6()
  # @see registers.GYRO_Y_L
  def getRotationY(self):
    raw = self._regs_read(registers.GYRO_Y_L, 2)
    val = unpack('<h', bytes(raw))
    return val

  # Get Z-axis gyroscope reading.
  # @return Z-axis rotation measurement in 16-bit 2's complement format
  # @see getMotion6()
  # @see registers.GYRO_Y_L
  def getRotationZ(self):
    raw = self._regs_read(registers.GYRO_Z_L, 2)
    val = unpack('<h', bytes(raw))
    return val

  # Read a BMI160 register directly.
  # @param reg register address
  # @return 8-bit register value
  def getRegister(self, reg):
    return self._reg_read(reg)

  # Write a BMI160 register directly.
  # @param reg register address
  # @param data 8-bit register value
  def setRegister(self, reg, data):
    self._reg_write(reg, data)

  # ──────────────────────────────────────────────────────────────────────────────
  #                               I2C COMMUNICATION                               
  # ──────────────────────────────────────────────────────────────────────────────
  def _reg_write(self, reg, data):
    write = i2c_msg.write(self.addr, [reg, data])
    self.bus.i2c_rdwr(write)

  def _reg_read(self, reg):
    return self._regs_read(reg, 1)[0]

  def _regs_read(self, reg, n):
    write = i2c_msg.write(self.addr, [reg])
    sleep_us(2)
    read = i2c_msg.read(self.addr, n)
    self.bus.i2c_rdwr(write, read)
    result = list(read)
    #print('< ', result)
    return result

  def close(self):
    self.bus.close()
