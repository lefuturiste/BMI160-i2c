import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
  name='BMI160',
  version='0.1',
  scripts=['dokr'] ,
  author="Matthieu Bessat - lefuturiste",
  author_email="contact@lefuturiste.fr",
  description="A I2C driver library to use the BMI160 sensor (acclerometer/gyro) with python on a raspberrypi",
  long_description=long_description,
  long_description_content_type="text/markdown",
  url="https://github.com/javatechy/dokr",
  packages=setuptools.find_packages(),
  install_requires=["smbus2"],
      keywords="adafruit blinka circuitpython micropython circuitplayground bluefruit CLUE ble",

  classifiers=[
    "Programming Language :: Python :: 3",
    "License :: OSI Approved :: MIT License",
    "Operating System :: Raspberry PI",
    "Intended Audience :: Developers",
    "Topic :: Software Development :: Libraries",
    "Topic :: System :: Hardware",
  ],
)

