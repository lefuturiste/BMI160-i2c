from time import sleep

def sleep_ms(duration):
  return sleep(duration * 0.001)

def sleep_us(duration):
  return sleep(duration * 10**-6)
