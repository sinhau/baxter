% Initialize Serial Communication
clear all;close all;clc;
s1 = serial('/dev/ttyACM0');    % define serial port
set(s1, 'DataBits', 8);
set(s1, 'StopBits', 1);
s1.BaudRate=9600;               % define baud rate
set(s1, 'Parity', 'none');
fopen(s1); 
fprintf(s1,'%c', 'w');
pause(2);

kbhit('init');
disp('ready')
while(1)
    
    
    keyPress = kbhit;
    if isempty(keyPress)
        keyPress = ' ';
    end
    switch keyPress
        case 'W'
            fwrite(s1,char(70),'char');
        case 'S'
            fwrite(s1,char(20),'char');
        case 'A'
            fwrite(s1,char(170),'char');
        case 'D'
            fwrite(s1,char(230),'char');
        case 'C'
            fwrite(s1,char(203),'char');
            pause(0.01);
            fwrite(s1,char(52),'char');
        otherwise
    end
    pause(0.01);
            
end