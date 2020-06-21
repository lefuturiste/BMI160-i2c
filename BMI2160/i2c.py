from smbus2 import SMBus

from BMI2160.driver import BMI160

from BMI2160.sleep import sleep_us, sleep_ms

class BMI160I2C(BMI160):
  def __init__(self, addr=0x68):
    self.addr = addr
    self.bus = SMBus(1)
    super().__init__()

  def _reg_write(self, reg, data):
    data = bytes([reg, data])
    print('> ', data)
    self.bus.write_i2c_block_data(self.addr, 0, data)
    # old: self.i2c.writeto(self.addr, bytes([reg, data]))

  def _reg_read(self, reg):
    return self._regs_read(reg, 1)[0]

  def _regs_read(self, reg, n):
    self.bus.write_i2c_block_data(self.addr, 0, bytes([reg]))
    # old: self.i2c.writeto(self.addr, bytes([reg]))
    sleep_us(2)
    result = self.bus.read_i2c_block_data(self.addr, 0, n)
    print('< ', result)
    #print(result)
    return result
    # old: return self.i2c.readfrom(self.addr, n)
  
  def close(self):
    self.bus.close()
