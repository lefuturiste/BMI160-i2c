from time import sleep
from BMI160_i2c import Driver

print('Trying to initialize the sensor...')
sensor = Driver()
print('Initialization done')

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
  sleep(0.1)

