# BMI160-i2c

I2C library to use the Inertial Mesurment Unit BMI160.

This library was tested successfully on a Raspberry Pi 3 B

## Installation

The package is [available on pypi.org](https://pypi.org/project/BMI160-i2c/0.1/).

You can install this package using this command

`python3 -m pip install BMI160-i2c`

**This library require [smbus](https://github.com/kplindegaard/smbus2)**

Install smbus2 using the following command:

`python3 -m pip install smbus2`

## Usage

Wire the breakout board with these lines : GND, 3V3, SAO (to GND), SDA, SCL

Make sure that the device is available at `0x68` or `0x69` i2c address by running this command:

`i2cdetect -y 1`

Example : A little python script to fetch all 6 values from the sensor :

```python
from time import sleep
from BMI160_i2c import Driver

print('Trying to initialize the sensor...')
sensor = Driver(0x68) # change address if needed
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
```

## Documentation

There is so many method available to do whatever you want to do with a sensor of this kind.

Look at all the methods available [here](https://github.com/lefuturiste/BMI160-i2c/blob/master/BMI160_i2c/__init__.py).

## Credits & Related links

- [hanyazou/BMI160-Arduino](https://github.com/hanyazou/BMI160-Arduino/)
- [serioeseGmbH/BMI160](https://github.com/serioeseGmbH/BMI160)
- [IMU BMI160 Bosch product page](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160.html)
- [BMI160 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf)
- [smbus2 docs](https://smbus2.readthedocs.io/en/latest/)

## Contributions

Feel free to open a issue or a pull request I will be happy to answer any questions or help you with this library.

You can also use these alternative methods to contact me: 

- Twitter: [@_le_futuriste](https://twitter.com/_le_futuriste)

- Discord: `lefuturiste#5297`

- Discord server: [https://discord.gg/9M4vVsX](https://discord.gg/9M4vVsX)

## Maintenance

- Increment the version used in `setup.py`
- Build the package: `python3 setup.py sdist bdist_wheel`
- Publish the package: `python3 -m twine upload dist/*`
- Enter `__token__` for the username
- Enter `pypi-{....}` for the password
- And tada!
