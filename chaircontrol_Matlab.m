%% Connect to XBox RR Service
try
  %'tcp://host:port/NodeName/RegisteredObjectName'
  xbox = RobotRaconteur.Connect('tcp://192.168.1.104:5437/Xbox_controllerServer/xbox_controller');
  xboxIsConnected=1;
catch ME
  disp(ME.message);
  disp('XBox RobotRaconteur Service was not running.');
end

%% Initialize Serial Communication
s1 = serial('/dev/ttyS106');    % define serial port
set(s1, 'DataBits', 8);
set(s1, 'StopBits', 1);
s1.BaudRate=9600;               % define baud rate
set(s1, 'Parity', 'none');
fopen(s1); 

fprintf(s1,'%c', 'w');
pause(2);

%% Control loop
while(1)
     xBoxInput = xbox.controller_input;     
     xInput = (xBoxInput.left_thumbstick_X*-1 + 10000)* 130 /19000 + 136;
     yInput = (xBoxInput.left_thumbstick_Y + 10000)* 130 /20000 - 19;
     if(xInput > 255)
         xInput = 255;
     end
     
     if(xInput < 158)
         xInput = 158;
     end
     
     if(yInput > 115)
         yInput = 115;
     end
     
     if(yInput < 0)
         yInput = 0;
     end
     disp(xInput);
     disp(yInput);    
     
     fwrite(s1,char(xInput), 'char');
     fwrite(s1,char(yInput), 'char');
     pause(.01);
end;