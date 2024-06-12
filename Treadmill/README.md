The treadmill is controlled by a Lenze SMVector. It can be controlled locally by the keypad or via an ethernet network connection.

## Parameter modification:
The display has three modes, and the "M" button is used to switch between them.   
The first mode, which indicates "stop" when the belt is at a standstill, gives information on the belt's status.  
The second, which indicates "PXXX", where XXX is a number, corresponds to the value of the parameter being modified. There are many parameters, so please refer to the controller datasheet to find out what they are used for.   
The third corresponds to the value you wish to associate with the parameter selected in the previous mode.  

## Local use:
For local use, certain parameters must be selected:  
P100 = 0  
P101 = 3  
P110 = 0  
P131 = "speed between 0 and 600 Hz".  
To start the treadmill, go to the first mode and press the green button.
To change speed, modify the value associated with P131.

## Network operation:
For use via the network, with an Ethernet connection, certain parameters must be selected:  
P100 = 3  
P101 = 6  
P121 = 9  
P140 = 14  
P142 = 14  
P400 = 5  
P410-P413 = "Modification of IP address".  
Once these parameters have been set, you need to configure the computer to be on the same local network, e.g. 192.168.124.16.  
Connecting from a browser to the IP address, an interface opens, allowing us to modify the parameters.  
To start the treadmill, set P65 to 97, and 0 to stop it.  
To adjust speed, set parameter P61 to between 0 and 600.
