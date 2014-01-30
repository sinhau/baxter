%% Connect to XBox RR Service
try
  %'tcp://host:port/NodeName/RegisteredObjectName'
  xbox = RobotRaconteur.Connect('tcp://localhost:5437/Xbox_controllerServer/xbox_controller');
  xboxIsConnected=1;
catch ME
  disp(ME.message);
  disp('XBox RobotRaconteur Service was not running.');
end

arduino = RobotRaconteur.Connect('tcp://localhost:5444/ArduinoServer2/arduino2');

%% Control loop
while(1)
     xBoxInput = xbox.controller_input;     
     xInput = (xBoxInput.left_thumbstick_X)/130;
     yInput = (xBoxInput.left_thumbstick_Y )/130;
     
%      if(xInput > 255)
%          xInput = 255;
%      end
%      
%      if(xInput < 158)
%          xInput = 158;
%      end
%      
%      if(yInput > 115)
%          yInput = 115;
%      end
%      
%      if(yInput < 0)
%          yInput = 0;
%      end

     disp(xInput);
     disp(yInput);    
     arduino.setPotentiometer(int32(-1), int32(xInput));
     arduino.setPotentiometer(int32(1), int32(yInput));
     pause(.01);
end;