## bit field offsets and lengths
ACC_PMU_STATUS_BIT  = (4)
ACC_PMU_STATUS_LEN  = (2)
GYR_PMU_STATUS_BIT  = (2)
GYR_PMU_STATUS_LEN  = (2)
GYRO_RANGE_SEL_BIT  = (0)
GYRO_RANGE_SEL_LEN  = (3)
GYRO_RATE_SEL_BIT   = (0)
GYRO_RATE_SEL_LEN   = (4)
GYRO_DLPF_SEL_BIT   = (4)
GYRO_DLPF_SEL_LEN   = (2)
ACCEL_DLPF_SEL_BIT  = (4)
ACCEL_DLPF_SEL_LEN  = (3)
ACCEL_RANGE_SEL_BIT = (0)
ACCEL_RANGE_SEL_LEN = (4)

## Gyroscope Sensitivity Range options
# see setFullScaleGyroRange()
GYRO_RANGE_2000     = (0)    # +/- 2000 degrees/second
GYRO_RANGE_1000     = (1)    # +/- 1000 degrees/second
GYRO_RANGE_500      = (2)    # +/-  500 degrees/second
GYRO_RANGE_250      = (3)    # +/-  250 degrees/second
GYRO_RANGE_125      = (4)    # +/-  125 degrees/second

## Accelerometer Sensitivity Range options
# see setFullScaleAccelRange()
ACCEL_RANGE_2G      = (0X03) # +/-  2g range
ACCEL_RANGE_4G      = (0X05) # +/-  4g range
ACCEL_RANGE_8G      = (0X08) # +/-  8g range
ACCEL_RANGE_16G     = (0X0C) # +/- 16g range

FOC_ACC_Z_BIT       = (0)
FOC_ACC_Z_LEN       = (2)
FOC_ACC_Y_BIT       = (2)
FOC_ACC_Y_LEN       = (2)
FOC_ACC_X_BIT       = (4)
FOC_ACC_X_LEN       = (2)
FOC_GYR_EN          = (6)