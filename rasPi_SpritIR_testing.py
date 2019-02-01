import serial
import time
import csv

ser0 = serial.Serial("/dev/ttyUSB0", timeout=1)
ser1 = serial.Serial("/dev/ttyUSB1", timeout=1)
ser2 = serial.Serial("/dev/ttyUSB2", timeout=1)
#ser3 = serial.Serial("/dev/ttyUSB3", timeout=1)
print ("Python Progam for SprintIR 20% Sensor Development Kit")

meausrementTime = 15
loopFreq = .1

loopNum = meausrementTime*(1/loopFreq)

with open("CO2_SprintIR_dataSHH2.csv",mode='w') as csvfile:
    entries = ['CO2_0', 'CO2_1','CO2_2','CO2_3']
    writer = csv.DictWriter(csvfile, fieldnames=entries)

    writer.writeheader()
    print (loopNum)
    # \r\n is CR and LF
    ser0.flushInput()
    ser1.flushInput()
    ser2.flushInput()
    #ser3.flushInput()
    time.sleep(1)
    ser0.write(b'G\r\n')
    ser1.write(b'G\r\n')
    ser2.write(b'G\r\n')
    #ser3.write(b'G\r\n')
    print('Zero Point Cal Air')
    time.sleep(1)

    for x in range(int(loopNum)):
        ser0.write(b'Z\r\n')
        resp0 = (ser0.read(10))
        resp0 = resp0[:8]
        fltCo20 = float(resp0[2:])
        print ("CO2 PPM   =", fltCo20 *10) # multiplierZ Change data to print on a single writeRow...and figure out serial in py3
        ser1.write(b'Z\r\n')
        resp1 = ser1.read(10)
        resp1 = resp1[:8]
        fltCo21 = float(resp1[2:])
        print ("CO2 PPM 2 =", fltCo21 *10) # multiplierZ
        ser2.write(b'Z\r\n')
        resp2 = ser2.read(10)
        resp2 = resp2[:8]
        fltCo22 = float(resp2[2:])
        print ("CO2 PPM 3 =", fltCo22 *10) # multiplierZ
        # ser3.write(b'Z\r\n')
        # resp3 = ser3.read(10)
        # resp3 = resp3[:8]
        # fltCo23 = float(resp3[2:])
        # print ("CO2 PPM 4 =", fltCo23 *10) # multiplierZ
        writer.writerow({'CO2_0': fltCo20 *10, 'CO2_1': fltCo21 *10, 'CO2_2': fltCo22 *10, 'CO2_3':0})
        time.sleep(0.1)
