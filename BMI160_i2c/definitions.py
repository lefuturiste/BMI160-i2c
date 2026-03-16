## bit field offsets and lengths
ACC_OFFSET_EN       = (6)
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
STATUS_FOC_RDY = (3)

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

##FIFO config options
FIFO_TIME_EN_BIT    = (1)
FIFO_MAG_EN_BIT     = (5)
FIFO_ACC_EN_BIT     = (6)
FIFO_GYR_EN_BIT     = (7)

## Step counter definitions
STEP_MODE_NORMAL    = (0)
STEP_MODE_SENSITIVE = (1)
STEP_MODE_ROBUST    = (2)
STEP_MODE_UNKNOWN   = (0)
STEP_BUF_MIN_BIT    = (0)
STEP_CNT_EN_BIT     = (3)
STEP_EN_BIT         = (3)
STEP_BUF_MIN_LEN    = (4)

## Interrupt definitions
INT1_OUTPUT_EN      = (3)
INT1_OD             = (2)
INT1_LVL            = (1)
LATCH_MODE_BIT      = (0)
LATCH_MODE_LEN      = (4)

# No Motion
NOMOTION_EN_BIT     = (0)
NOMOTION_EN_LEN     = (3)
NOMOTION_INT_BIT    = (7)
NOMOTION_DUR_BIT    = (2)
NOMOTION_DUR_LEN    = (6)
NOMOTION_SEL_BIT    = (0)
NOMOTION_SEL_LEN    = (1)

# Frequency
ACCEL_RATE_SEL_BIT  = (0)
ACCEL_RATE_SEL_LEN  = (4)
