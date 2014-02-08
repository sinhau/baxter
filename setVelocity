% Baxter connections
baxter = RobotRaconteur.Connect('tcp://localhost:4682/BaxterServer/Baxter');

filename = fullfile('baxterData.dat');
m = memmapfile(filename,'Writable',true,'Format','double');

while(1)
    
    while m.Data(1) == 0
        baxter.setJointVelocity('left',zeros(1,7));
        baxter.setJointVelocity('right',zeros(1,7));
    end
    
    qDot_L = m.Data(2:8);
    qDot_R = m.Data(9:15);
    grip_L = m.Data(16);
    grip_R = m.Data(17);
    
    baxter.setJointVelocity('left',qDot_L);
    baxter.setJointVelocity('right',qDot_R);
    baxter.setGripperPosition('left',grip_L);
    baxter.setGripperPosition('right',grip_R);    
    
end