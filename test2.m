% xbox
try
    xbox = RobotRaconteur.Connect('tcp://192.168.1.110:5437/Xbox_controllerServer/xbox_controller');
    xboxIsConnected=1;
catch ME
    disp(ME.message);
    error('XBox RobotRaconteur Service was not running.');
end

% File
fileID = fopen('xboxData.txt','w');

while(1)
   
    input = xbox.controller_input;
    data = [double(input.left_thumbstick_X);double(input.left_thumbstick_Y)];
    fprintf(fileID,'%6.2f %6.2f\n',data);
    
end