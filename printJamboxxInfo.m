function printJamboxxInfo(jamboxx)
% Display current x,y, and air values for Jamboxx

    disp(['Jamboxx X-Pos:',num2str(jamboxx.X)]);
    disp(['Jamboxx Y-Pos:',num2str(jamboxx.Y)]);
    disp(['Jamboxx Air-Pos:',num2str(jamboxx.Air)]);
    disp(' ');

    pause(0.3);
    clc;

end