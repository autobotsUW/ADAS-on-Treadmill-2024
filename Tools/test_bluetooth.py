import bluetooth
from bluetooth import Protocols
import time

# bd_addr = '58:56:00:01:06:67'  
# bd_addr = '98:D3:71:FE:AB:41'     #Purple
bd_addr = '98:D3:51:FE:EC:72'     #Red
port = 1
sock = bluetooth.BluetoothSocket(Protocols.RFCOMM)
sock.connect((bd_addr, port))
bluetooth_status = True
speed=50
angle=45
while True:
    # for angle in range(70,111,10):
        # msg = "{},{}\n".format(int(speed), int(angle))
        msg = "[{},{}]".format(int(speed), int(angle))
        print(msg)
        msb_bytes = msg.encode('UTF-8')
        sock.send(msb_bytes)
        time.sleep(0.03)
        # speed+=1
        # if speed>=150:
        #         speed=0
        # print(sock.listen())

print('Fin')


# bluetoothctl
# pair mac adress
# trust mac adress
# exit


