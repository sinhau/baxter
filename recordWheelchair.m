clear all;close all;clc;

% xbox
try
    xbox = RobotRaconteur.Connect('tcp://192.168.1.110:5437/Xbox_controllerServer/xbox_controller');
    xboxIsConnected=1;
catch ME
    disp(ME.message);
    error('XBox RobotRaconteur Service was not running.');
end

% Arduino
s1 = serial('/dev/ttyS100');    % define serial port
set(s1, 'DataBits', 8);
set(s1, 'StopBits', 1);
s1.BaudRate=9600;               % define baud rate
set(s1, 'Parity', 'none');
fopen(s1); 

while(1)
    
    xboxInput = xbox.controller_input;
    
    % Recording routine
    if xboxInput.start_button
        record = 1;
        %fileName = input('Enter file name to record in:','s');
        fileName = 'macro.txt';
        fileID = fopen(fileName,'w'); disp('Recording...');
        pause(1);
        while record == 1
           xboxInput = xbox.controller_input;
           if xboxInput.start_button
               record = 0;
               disp('Stopped recording. Hit back button to playback.');pause(1);
           end
           xInput = (xboxInput.left_thumbstick_X*-1 + 10000)* 130 /20000 + 127; 
           yInput = (xboxInput.left_thumbstick_Y + 10000)* 130 /20000;
           xInput = limitVal(129,255,xInput); yInput = limitVal(0,127,yInput);
           fwrite(s1,char(xInput), 'char'); disp(xInput);
           fwrite(s1,char(yInput), 'char'); disp(yInput);
           data = [double(xInput);double(yInput)];
           fprintf(fileID,'%6.2f %6.2f\n',data);
           pause(0.1);
        end
        fclose(fileID);
    
    % Playback routine
    elseif xboxInput.back_button
        %fileName = input('Enter file name to play:','s');
        fileName = 'macro.txt';
        fileID = fopen(fileName,'r'); sizeData = [2 Inf];
        xboxData = fscanf(fileID,'%f %f',sizeData); fclose(fileID);
        xboxData = xboxData';
        disp('Playing...'); pause(1);
        for i = 1:length(xboxData)
            fwrite(s1,char(xboxData(i,1)), 'char'); disp(xboxData(i,1));
            fwrite(s1,char(xboxData(i,2)), 'char'); disp(xboxData(i,2));
            pause(0.1);
        end
        disp('Stopped playing...you have control!');pause(1);
        
    % Regular driving routine    
    else
        xInput = (xboxInput.left_thumbstick_X*-1 + 10000)* 130 /20000 + 127; 
        yInput = (xboxInput.left_thumbstick_Y + 10000)* 130 /20000;
        xInput = limitVal(127,255,xInput); yInput = limitVal(0,129,yInput);
        fwrite(s1,char(xInput), 'char'); disp(xInput);
        fwrite(s1,char(yInput), 'char'); disp(yInput);
        pause(0.1);
    end
           
end


