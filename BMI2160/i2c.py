from smbus2 import SMBus, i2c_msg

from BMI2160.driver import BMI160

from BMI2160.sleep import sleep_us, sleep_ms

class BMI160I2C(BMI160):
  def __init__(self, addr=0x68):
    self.addr = addr
    self.bus = SMBus(1)
    super().__init__()

  def _reg_write(self, reg, data):
    data = [reg, data]
    #print('> ', data)
    write = i2c_msg.write(self.addr, data)
    self.bus.i2c_rdwr(write)
    # old: self.i2c.writeto(self.addr, bytes([reg, data]))

  def _reg_read(self, reg):
    res = self._regs_read(reg, 1)
    #print(res)
    return res[0]

  def _regs_read(self, reg, n):
    #self.bus.write_i2c_block_data(self.addr, 0, [reg])
    write = i2c_msg.write(self.addr, [reg])
    # old: self.i2c.writeto(self.addr, bytes([reg]))
    sleep_us(2)
    read = i2c_msg.read(self.addr, n)
    self.bus.i2c_rdwr(write, read)
    result = list(read)
    #print('< ', result)
    #print(result)
    # old: return self.i2c.readfrom(self.addr, n)
    return result
  
  def close(self):
    self.bus.close()
