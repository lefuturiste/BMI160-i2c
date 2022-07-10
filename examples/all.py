from time import sleep
from BMI160_i2c import Driver
from BMI160_i2c import definitions

I2C_BUS_ID = 1
BMI160_ADDR = 0x69

print('Trying to initialize the sensor...')
sensor = Driver(BMI160_ADDR, I2C_BUS_ID) 
print('Initialization done')

# Step detection requires interrupt config
# See https://github.com/BoschSensortec/BMI160_driver#configuring-step-detector-interrupt
sensor.setIntStepEnabled(True)
# push-pull mode interrupt
sensor.setInterruptDrive(0)
# 0 = non-latched
sensor.setInterruptLatch(0)
# 0 = active-high
sensor.setInterruptMode(0)
sensor.setIntEnabled(True)
# Normal step detection mode
sensor.setStepDetectionMode(definitions.STEP_MODE_NORMAL)
sensor.resetStepCount()
sensor.setStepCountEnabled(True)
    
while True:
  data = sensor.getMotion6()
  # fetch all gyro and acclerometer values
  print({
    'gx': data[0],
    'gy': data[1],
    'gz': data[2],
    'ax': data[3],
    'ay': data[4],
    'az': data[5]
  })
  print("Step count: ", sensor.getStepCount())
  sleep(0.1)

