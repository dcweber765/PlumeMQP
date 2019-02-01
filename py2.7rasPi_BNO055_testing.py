import csv
import logging
import sys
import time

from Adafruit_BNO055 import BNO055
bno = BNO055.BNO055()

if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Recording BNO055 data')

meausrementTime = 60
loopFreq = .5

loopNum = meausrementTime*(1/loopFreq)

with open('BNO055_accel_data_tes.csv', mode='w') as csvfile:
    entries = ['AccelerometerX','AccelerometerY','AccelerometerZ', 'GyroscopeX', 'GyroscopeY', 'GyroscopeZ', 'LinAccelX', 'LinAccelY','LinAccelZ']
    writer = csv.DictWriter(csvfile, fieldnames=entries)

    writer.writeheader()

    for i in range(int(loopNum)):
        heading, roll, pitch = bno.read_euler()
        accelX, accelY, accelZ  = bno.read_accelerometer()
        gyroX, gyroY, gyroZ = bno.read_gyroscope()
        LinAccelX, LinAccelY, LinAccelZ = bno.read_linear_acceleration()
        #writer.writerow({accelX, accelY, accelZ, gyroX, gyroY, gyroZ, LinAccelX, LinAccelY, LinAccelZ})
        writer.writerow({'AccelerometerX':accelX,'AccelerometerY':accelY,'AccelerometerZ':accelZ, 'GyroscopeX':gyroX, 'GyroscopeY':gyroY, 'GyroscopeZ':gyroZ, 'LinAccelX':LinAccelX, 'LinAccelY':LinAccelY,'LinAccelZ':LinAccelZ})
        # Print everything out.
        print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}'.format(heading, roll, pitch))
        print('accelX={0:0.2F} accelY={1:0.2F} accelZ={2:0.2F}'.format(accelX, accelY, accelZ))
        time.sleep(.5)
