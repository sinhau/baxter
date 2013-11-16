kbhit('init')

while(1)
    
    keyPress = kbhit;
    disp(keyPress);
    if strcmp(keyPress,' ')
        disp('space')
    end
    if isempty(keyPress)
        disp('nothing')
    end
    pause(0.1);
    clc;
    
end
