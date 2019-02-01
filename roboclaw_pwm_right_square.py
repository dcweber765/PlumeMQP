import time
from roboclaw import Roboclaw

#Windows comport name
#rc = Roboclaw("COM9",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
address = 0x80

rc.ForwardMixed(address, 0)
rc.TurnRightMixed(address, 0)

i = 0

while(i < 5):
        rc.ForwardMixed(address, 45)
        time.sleep(2)
        rc.ForwardMixed(address, 0)
        rc.TurnRightMixed(address, 32)
        time.sleep(1)
        rc.TurnRightMixed(address, 0)
        i = i+1
        print i

rc.ForwardMixed(address, 0)
rc.TurnRightMixed(address, 0)
