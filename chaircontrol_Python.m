%% Connect to XBox RR Service
try
  %'tcp://host:port/NodeName/RegisteredObjectName'
  xbox = RobotRaconteur.Connect('tcp://192.168.1.104:5437/Xbox_controllerServer/xbox_controller');
  xboxIsConnected=1;
catch ME
  disp(ME.message);
  disp('XBox RobotRaconteur Service was not running.');
end

o = RobotRaconteur.Connect('tcp://localhost:5001/{0}/Arduino');

%% Control loop
while(1)
     xBoxInput = xbox.controller_input;     
     xInput = (xBoxInput.left_thumbstick_X)/130;
     yInput = (xBoxInput.left_thumbstick_Y )/130;

     disp(xInput);
     disp(yInput);    
     o.arduino(int8(-1), int8(xInput));
     o.arduino(int8(1), int8(yInput));
end;