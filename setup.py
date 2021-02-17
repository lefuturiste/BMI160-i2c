import setuptools

with open('README.md', 'r') as fh:
    long_description = fh.read()

setuptools.setup(
  name='BMI160_i2c',
  version='0.5',
  author='Matthieu Bessat - lefuturiste',
  author_email='contact@lefuturiste.fr',
  description="A I2C driver library to use the BMI160 sensor (acclerometer/gyro) with python on a raspberrypi",
  long_description=long_description,
  long_description_content_type='text/markdown',
  url='https://github.com/lefuturiste/BMI160-i2c',
  packages=setuptools.find_packages(),
  install_requires=['smbus2'],
  keywords="python lefuturiste driver BMI160 acclerometer gyroscope gyro BMI bosch sensortech smbus2 sensor library i2c",
  classifiers=[
    'Programming Language :: Python :: 3',
    'License :: OSI Approved :: MIT License',
    'Intended Audience :: Developers',
    'Topic :: Software Development :: Libraries',
    'Topic :: System :: Hardware',
  ],
  python_requires='>=3.6',
)

