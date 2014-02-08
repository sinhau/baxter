% Arduino connection
o = RobotRaconteur.Connect('tcp://localhost:5002/{0}/Arduino');
% Xbox 360 controller connection
xbox = RobotRaconteur.Connect('tcp://192.168.1.104:5437/Xbox_controllerServer/xbox_controller');

while(1)
     % Arduino
     xBoxInput = xbox.controller_input;
     xInput = (xBoxInput.left_thumbstick_X)/130
     yInput = (xBoxInput.left_thumbstick_Y )/130
%     disp(xInput);
%     disp(yInput);
     o.arduino(int8(-1), int8(xInput));
     o.arduino(int8(1), int8(yInput));
end