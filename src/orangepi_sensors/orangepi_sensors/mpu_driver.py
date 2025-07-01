from periphery import I2C
import sys

class MPU6050:
    I2C_BUS = 2
    DEVICE_ADDRESS = 0x68
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0

    def __init__(self):
        try:
            self.i2c = I2C(f"/dev/i2c-{self.I2C_BUS}")
            wakeup_msg = [I2C.Message([self.PWR_MGMT_1, 0x00])]
            self.i2c.transfer(self.DEVICE_ADDRESS, wakeup_msg)
        except I2C.I2CError as e:
            print(f"Error initializing MPU-6050: {e}")
            sys.exit(1)

    def _combine_bytes(self, msb, lsb):
        value = (msb << 8) | lsb
        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value

    def get_sensor_data(self):
        read_op = [I2C.Message([self.ACCEL_XOUT_H]), I2C.Message([0]*14, read=True)]
        self.i2c.transfer(self.DEVICE_ADDRESS, read_op)
        data = read_op[1].data
        accel_x = self._combine_bytes(data[0], data[1]) / self.ACCEL_SCALE
        accel_y = self._combine_bytes(data[2], data[3]) / self.ACCEL_SCALE
        accel_z = self._combine_bytes(data[4], data[5]) / self.ACCEL_SCALE
        gyro_x = self._combine_bytes(data[8], data[9]) / self.GYRO_SCALE
        gyro_y = self._combine_bytes(data[10], data[11]) / self.GYRO_SCALE
        gyro_z = self._combine_bytes(data[12], data[13]) / self.GYRO_SCALE
        return {'accel_x': accel_x, 'accel_y': accel_y, 'accel_z': accel_z, 'gyro_x': gyro_x, 'gyro_y': gyro_y, 'gyro_z': gyro_z}

    def close(self):
        self.i2c.close()