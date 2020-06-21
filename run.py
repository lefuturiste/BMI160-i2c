from time import sleep
from BMI2160.i2c import BMI160I2C

print('init ')
sensor = BMI160I2C()

print('init done')
while True:
  print(sensor.getMotion6())
  #print("{0:>8}{1:>8}{2:>8}{3:>8}{4:>8}{5:>8}".format())
  sleep(0.01)