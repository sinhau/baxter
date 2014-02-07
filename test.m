clear all;close all;clc;
% This is the main file for Jamboxx based control of Baxter.

%% Robot Raconteur Connections

% Baxter connections
baxter = RobotRaconteur.Connect('tcp://localhost:4682/BaxterServer/Baxter');

x = tic;
y = toc(x);
while y < 5
    baxter.setJointPosition('left',[-0.8544;-0.8805;0.1511;1.9934;-0.0418;-1.1789;-0.1457]);
    baxter.setJointPosition('right',[0.6105;-1.0761;0.1806;2.2415;-0.0065;-1.1340;-1.7675]);
    y = toc(x);
end

x = tic;
while(1)
    
   baxter.setJointVelocity('left',[0;0;0;0;0;0;0]);
   baxter.setJointVelocity('right',[0;0;0;0;0;0;0]);
   toc(x);
   
end