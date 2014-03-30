% Baxter connections
try
    baxter = RobotRaconteur.Connect('tcp://localhost:4698/BaxterServer/Baxter');
catch ME
    disp(ME.message);
    error('Cannot connect to Baxter');
    
end

filename = fullfile('baxterData.dat');
m = memmapfile(filename,'Writable',true,'Format','double');
old_qDot_L = zeros(7,1); old_qDot_R = zeros(7,1);

while(1)
    
    while m.Data(1) == 0
        baxter.setJointVelocity('left',old_qDot_L);
        baxter.setJointVelocity('right',old_qDot_R);
    end
    
    qDot_L = m.Data(2:8); baxter.setJointVelocity('left',qDot_L); old_qDot_L = qDot_L;
    qDot_R = m.Data(9:15); baxter.setJointVelocity('right',qDot_R); old_qDot_R = qDot_R;
    grip_L = m.Data(16); baxter.setGripperPosition('left',grip_L);
    grip_R = m.Data(17); baxter.setGripperPosition('right',grip_R);

end