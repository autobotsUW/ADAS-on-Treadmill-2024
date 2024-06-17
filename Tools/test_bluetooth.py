import bluetooth
from bluetooth import Protocols
import time

bd_addr = '58:56:00:01:06:67'
# bd_addr = '28:CD:C1:09:63:09'
port = 1
sock = bluetooth.BluetoothSocket(Protocols.RFCOMM)
sock.connect((bd_addr, port))
bluetooth_status = True
speed=0
angle=0
while True:
        angle=120
    # for angle in range(70,111,10):
        print(angle)
        msg = "[{},{}]".format(int(speed), int(angle))
        msb_bytes = msg.encode('UTF-8')
        sock.send(msb_bytes)
        time.sleep(0.1)
        # print(sock.listen())

print('Fin')

