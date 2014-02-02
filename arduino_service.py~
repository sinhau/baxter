import RobotRaconteur as RR
import serial
import signal
import sys

arduino_interface="""
service arduino_interface
 
object arduino_obj
    function void arduino(int8 axis, int8 speed)
end object
"""
print('Starting server');
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=.1)


commandByte = [0];
 
class arduino_impl(object):
    def arduino(self, axis, speed):
        if axis == -1:
            commandByte[0] = (speed*-1+100)*130/190 + 136;
            if commandByte[0] > 255:
                commandByte[0] = 255;
            if commandByte[0] < 158:
                commandByte[0] = 158;
        if axis == 1:
            commandByte[0] = (speed + 100) * 130 / 200 - 19;
            if commandByte[0] > 115:
                commandByte[0] = 115;
            if commandByte[0] < 0:
                commandByte[0] = 0;
        b = chr(commandByte[0])
        print(ord(b[0]));
        ser.write(b);
        
 
#Create and register a transport
t=RR.TcpTransport()
t.StartServer(5001)
RR.RobotRaconteurNode.s.RegisterTransport(t)


def signal_handler(signal, frame):
        print 'Quitting server'
        RR.RobotRaconteurNode.s.Shutdown()
        ser.close()
        sys.exit(0)
        t.Close()
        
signal.signal(signal.SIGINT, signal_handler)
 
#Register the service type
RR.RobotRaconteurNode.s.RegisterServiceType(arduino_interface)
 
obj=arduino_impl()
 
#Register the service
RR.RobotRaconteurNode.s.RegisterService("Arduino","arduino_interface.arduino_obj",obj)
 
#Wait for program exit to quit

var = 1
while var == 1:
    var = 1
    
#Shutdown
RR.RobotRaconteurNode.s.Shutdown()
