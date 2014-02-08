% Baxter connections
baxter = RobotRaconteur.Connect('tcp://localhost:4682/BaxterServer/Baxter');

filename = fullfile('baxterData.dat');
m = memmapfile(filename,'Writable',true,'Format','double');

while(1)
    
    while m.Data(1) == 0
        baxter.setJointVelocity('left',zeros(7,1));
        baxter.setJointVelocity('right',zeros(7,1));
    end
    
    qDot_L = m.Data(2:8); baxter.setJointVelocity('left',qDot_L);
    qDot_R = m.Data(9:15); baxter.setJointVelocity('right',qDot_R);
    grip_L = m.Data(16); baxter.setGripperPosition('left',grip_L);
    grip_R = m.Data(17); baxter.setGripperPosition('right',grip_R);

end