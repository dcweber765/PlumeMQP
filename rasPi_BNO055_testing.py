import time
import csv
import board
import busio
import adafruit_bno055

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

meausrementTime = 60
loopFreq = .1

loopNum = meausrementTime*(1/loopFreq)

with open('BNO055_accel_data_test.csv', mode='w') as csvfile:
    entries = ['Accelerometer', 'Gyroscope', 'LinAccel']
    writer = csv.DictWriter(csvfile, fieldnames=entries)

    writer.writeheader()

    for i in range(int(loopNum)):
        writer.writerow({'Accelerometer':'{}'.format(sensor.accelerometer)[1:-1], 'Gyroscope':'{}'.format(sensor.gyroscope)[1:-1], 'LinAccel':'{}'.format(sensor.linear_acceleration)[1:-1]})
        print('Temperature: {} degrees C'.format(sensor.temperature))
        print('Accelerometer (m/s^2): {}'.format(sensor.accelerometer)[1:-1])
        print('Magnetometer (microteslas): {}'.format(sensor.magnetometer))
        print('Gyroscope (deg/sec): {}'.format(sensor.gyroscope))
        print('Euler angle: {}'.format(sensor.euler))
        print('Quaternion: {}'.format(sensor.quaternion))
        print('Linear acceleration (m/s^2): {}'.format(sensor.linear_acceleration))
        print('Gravity (m/s^2): {}'.format(sensor.gravity))
        print()
        time.sleep(.1)
