import serial
from datetime import datetime
import time
import csv




ser0 = serial.Serial("/dev/ttyUSB0", timeout=1)
ser1 = serial.Serial("/dev/ttyUSB1", timeout=1)
ser2 = serial.Serial("/dev/ttyUSB2", timeout=1)
ser3 = serial.Serial("/dev/ttyUSB3", timeout=1)

def getCO2Data():
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

    Co2_0 = fltCo20 * 10
    Co2_1 = fltCo21 * 10
    Co2_2 = fltCo22 * 10
    Co2_3 = fltCo23 * 10

    return Co2_0, Co2_1, Co2_2, Co2_3


with open("CO2_SprintIR_riseTime.csv",mode='w') as csvfile:
    entries = ['CO2_0', 'CO2_1','CO2_2','CO2_3', 'time']
    writer = csv.DictWriter(csvfile, fieldnames=entries)

    writer.writeheader()
    # \r\n is CR and LF
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

    for x in range(1200):
        print x
        CO2_0, CO2_1, CO2_2, CO2_3 = getCO2Data()
        writer.writerow({'CO2_0': CO2_0, 'CO2_1': CO2_1, 'CO2_2': CO2_2, 'CO2_3':CO2_3, 'time': str(datetime.now().time())})
        time.sleep(0.1)
    print('Done!')
