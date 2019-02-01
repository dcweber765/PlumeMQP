import csv
import logging
import sys
import time
import serial

from Adafruit_BNO055 import BNO055
bno = BNO055.BNO055()
ser0 = serial.Serial("/dev/ttyUSB0", timeout=1)
ser1 = serial.Serial("/dev/ttyUSB1", timeout=1)
ser2 = serial.Serial("/dev/ttyUSB2", timeout=1)
#ser3 = serial.Serial("/dev/ttyUSB3", timeout=1)

#CO2 startup
ser0.flushInput()
ser1.flushInput()
ser2.flushInput()
ser3.flushInput()
time.sleep(1)
ser0.write(b'G\r\n')
ser1.write(b'G\r\n')
ser2.write(b'G\r\n')
ser3.write(b'G\r\n')
print('Zero Point Cal Air')
time.sleep(1)

# IMU startup
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
print('Recoding CO2 data')

meausrementTime = 60
loopFreq = .5

loopNum = meausrementTime*(1/loopFreq)

with open('combined_data.csv', mode='w') as csvfile:
    entries = ['AccelerometerX','AccelerometerY','AccelerometerZ', 'GyroscopeX', 'GyroscopeY', 'GyroscopeZ', 'CO2_0', 'CO2_1','CO2_2']
    writer = csv.DictWriter(csvfile, fieldnames=entries)

    writer.writeheader()

    for i in range(int(loopNum)):
        heading, roll, pitch = bno.read_euler()
        accelX, accelY, accelZ  = bno.read_accelerometer()
        gyroX, gyroY, gyroZ = bno.read_gyroscope()
        LinAccelX, LinAccelY, LinAccelZ = bno.read_linear_acceleration()
        ser0.write(b'Z\r\n')
        resp0 = (ser0.read(10))
        resp0 = resp0[:8]
        fltCo20 = float(resp0[2:])
        ser1.write(b'Z\r\n')
        resp1 = ser1.read(10)
        resp1 = resp1[:8]
        fltCo21 = float(resp1[2:])
        ser2.write(b'Z\r\n')
        resp2 = ser2.read(10)
        resp2 = resp2[:8]
        fltCo22 = float(resp2[2:])
        ser3.write(b'Z\r\n')
        resp3 = ser3.read(10)
        resp3 = resp3[:8]
        fltCo23 = float(resp3[2:])

        writer.writerow({'AccelerometerX':accelX,'AccelerometerY':accelY,'AccelerometerZ':accelZ, 'GyroscopeX':gyroX, 'GyroscopeY':gyroY, 'GyroscopeZ':gyroZ, 'CO2_0': fltCo20 *10, 'CO2_1': fltCo21 *10, 'CO2_2': fltCo22 *10})
        # Print everything out.

        print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}'.format(heading, roll, pitch))
        print('accelX={0:0.2F} accelY={1:0.2F} accelZ={2:0.2F}'.format(accelX, accelY, accelZ))
        print 'CO2 PPM   =', fltCo20 *10
        print 'CO2 PPM 2 =', fltCo21 *10 # multiplierZ
        print 'CO2 PPM 3 =', fltCo22 *10 # multiplierZ
        print 'CO2 PPM 4 ='', fltCo23 *10) # multiplierZ
        time.sleep(.5)
