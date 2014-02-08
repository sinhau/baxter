% This is the main file for Jamboxx based control of Baxter.

%% Robot Raconteur Connections

% Baxter connections
baxter = RobotRaconteur.Connect('tcp://localhost:4682/BaxterServer/Baxter');

% Jamboxx connection
jamboxx = RobotRaconteur.Connect('tcp://192.168.1.104:5318/{0}/Jamboxx');




%% File

filename = fullfile('baxterData.dat');
if ~exist(filename,'file')
    [f,msg] = fopen(filename,'wb');
    if f ~= -1
        fwrite(f,[0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0],'double');
        fclose(f);
    else
        error('Cannot open file "%s": %s.',filename,msg);
    end
end
m = memmapfile(filename,'Writable',true,'Format','double');

%% Main

setBaxterConstants;

% Initialize left arm params
linVel_L = [0;0;0];
angVel_L = [0;0;0];
wristVel_L = [];
grip_L = [];
baxter.GripperCalibrate('left');
pause(2);

% Initialize right arm params
linVel_R = [0;0;0];
angVel_R = [0;0;0];
wristVel_R = [];
grip_R = [];
baxter.GripperCalibrate('right');
pause(2);

% Starting pose
x = tic;
y = 0;
while y < 5
    baxter.setJointPosition('left',[-0.8544;-0.8805;0.1511;1.9934;-0.0418;-1.1789;-0.1457]);
    baxter.setJointPosition('right',[0.6105;-1.0761;0.1806;2.2415;-0.0065;-1.1340;-1.7675]);
    y = toc(x);
end

%calibrateJamboxx(jamboxx);
clc; disp('READY TO MOVE...');

while(1) 
    
    m.Data(1) = 0;
    

 
    % Gather joint information
    jointAngles = baxter.JointPositions;
    jointAnglesLeft = jointAngles(1:7);
    jointAnglesRight = jointAngles(8:14);
    
    % Set desired input using Jamboxx
    [linVel_L,linVel_R,wristVel_L,wristVel_R,grip_L,grip_R] = setDesiredInput(jamboxx);
    
    % Calculate full jacobian for both arms
    leftJ = jacobian(baxterConst.leftArm,jointAnglesLeft);
    rightJ = jacobian(baxterConst.rightArm,jointAnglesRight);
    
    % Desired velocities frame correction
        % Left arm
        linVelCorrect_L = rot([0;0;1],pi/4)*linVel_L; % Rotate frame such that x-axis points in front of baxter
        angVelCorrect_L = rot([0;0;1],pi/4)*angVel_L; % Rotate frame such that x-axis points in front of baxter
        allVel_L = [angVelCorrect_L;linVelCorrect_L];
        % Right arm
        linVelCorrect_R = rot([0;0;1],3*pi/4)*linVel_R; % Rotate frame such that x-axis points in front of baxter
        angVelCorrect_R = rot([0;0;1],3*pi/4)*angVel_R; % Rotate frame such that x-axis points in front of baxter
        allVel_R = [angVelCorrect_R;linVelCorrect_R];
    
    % Calculate desired joint angle velocities
        % Left arm
        dampCoeff_L = 0.1;
        if any(allVel_L) || any(wristVel_L)
            qDot_L = leftJ'*pinv(leftJ*leftJ' + dampCoeff_L^2*eye(6,6))*allVel_L; %Damped least squares
            if ~isempty(wristVel_L)
                qDot_L(5:7) = wristVel_L;
            end
            % Limit joint velocity
            for k = 1:length(qDot_L)
                if abs(qDot_L(k)) > baxterConst.jointVelLimit(k)
                    qDot_L(k) = sign(qDot_L(k))*baxterConst.jointVelLimit(k);
                end
            end   
            m.Data(2:8) = qDot_L;
        else
            qDot_L = [0;0;0;0;0;0;0];
            m.Data(2:8) = qDot_L;
        end
        % Right arm
        dampCoeff_R = 0.1;
        if any(allVel_R)  || any(wristVel_R)
            qDot_R = rightJ'*pinv(rightJ*rightJ' + dampCoeff_R^2*eye(6,6))*allVel_R; %Damped least squares
            if ~isempty(wristVel_R)
                qDot_R(5:7) = wristVel_R;
            end
            % Limit joint velocity
            for k = 1:length(qDot_R)
                if abs(qDot_R(k)) > baxterConst.jointVelLimit(k)
                    qDot_R(k) = sign(qDot_R(k))*baxterConst.jointVelLimit(k);
                end
            end  
            m.Data(9:15) = qDot_R;
        else
            qDot_R = [0;0;0;0;0;0;0];
            m.Data(9:15) = qDot_R;
        end
        
    % Publish grip position
        % Left gripper
        if ~isempty(grip_L)
            m.Data(16) = grip_L;
        end
        %Right gripper
        if ~isempty(grip_R)
            m.Data(17) = grip_R;
        end
        m.Data(1) = 1;
        pause(0.1);
end
clc; msgbox('Program stopped!');