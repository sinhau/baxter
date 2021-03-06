%% Connect to XBox RR Service
try
    %'tcp://host:port/NodfcleName/RegisteredObjectName'
    xbox = RobotRaconteur.Connect('tcp://192.168.1.103:5437/Xbox_controllerServer/xbox_controller');
    xboxIsConnected=1;
catch ME
    disp(ME.message);
    error('XBox RobotRaconteur Service was not running.');
end

try
    baxter = RobotRaconteur.Connect('tcp://localhost:4654/BaxterJointServer/Baxter');
catch ME
    disp(ME.message);
    error('Cannot connect to Baxter.');
end

%% Initialize Serial Communicatio[-0.8544;-0.8805;0.1511;1.9934;-0.0418;-1.1789;-0.1457]n
s1 = serial('/dev/ttyS111');    % define serial port
set(s1, 'DataBits', 8);
set(s1, 'StopBits', 1);
s1.BaudRate=9600;               % define baud rate
set(s1, 'Parity', 'none');
fopen(s1); 

%fprintf(s1,'%c', 'w');
pause(2);

%% Control loop
counter = 0;
while(1)
     xBoxInput = xbox.controller_input;
     x = tic;
     y = 0;
     if xBoxInput.start_button
         baxter.setControlMode(uint8(0));
         while y < 10
             baxter.setJointCommand('left',[-0.7854;-1.0472;0;2.0944;0;-1.0472;-0]);
             baxter.setJointCommand('right',[0.7854;-1.0472;0;2.0944;0;-1.0472;-0]);
             y = toc(x);
         end
     end
         
     xInput = (xBoxInput.left_thumbstick_X*-1 + 10000)* 130 /19000 + 137; 
     yInput = (xBoxInput.left_thumbstick_Y + 10000)* 130 /20000 - 19; 
     if(xInput > 255)
        xInput = 255;
     end
     
     if(xInput < 158)
         xInput = 158;
     end
     
     if(yInput > 82)
         yInput = 82;
     end
     
     if(yInput < 0)
         yInput = 0;
     end
     disp(xInput);
     disp(yInput);    
     
     %DAMPING
     %xInput = 204 + (xInput - 204)/1.5;
     %yInput = 46 + (yInput - 46)/1.5;
     
     
     fwrite(s1,char(xInput), 'char');
     fwrite(s1,char(yInput), 'char');
     pause(.01);
     counter = counter + 1;
     if counter > 200
         counter = 0;
     end;
end;