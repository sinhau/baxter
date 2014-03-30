% This is the main file for Jamboxx based control of Baxter.

%% Robot Raconteur Connections

% Baxter joint connections
try
    baxter = RobotRaconteur.Connect('tcp://localhost:4654/BaxterJointServer/Baxter');
catch ME
    disp(ME.message);
    error('Cannot connect to Baxter');
end
%terminateBaxter = @(baxter)(disp(baxter));
%cleanupObj1 = onCleanup(terminateBaxter(1));

% Baxter peripherals connections
try
    baxterP = RobotRaconteur.Connect('tcp://localhost:4655/BaxterPeripheralServer/BaxterPeripherals');
catch ME
    disp(ME.message);
    error('Cannot connect to Baxter peripherals');
end
%terminatePeri = @(peri)(disp(peri));
%cleanupObj2 = onCleanup(terminatePeri(2));

% Jamboxx connection
try
    jamboxx = RobotRaconteur.Connect('tcp://192.168.1.103:5318/{0}/Jamboxx');
catch ME
    disp(ME.message);
    error('Cannot connect to Jamboxx');
end

%% Main

setBaxterConstants;

% Initialize left arm params
linVel_L = [0;0;0];
angVel_L = [0;0;0];
wristVel_L = [];
grip_L = [];
baxterP.calibrateGripper('left');
pause(2);
% This is the main file for Jamboxx based control of Baxter.

% Initialize right arm params
linVel_R = [0;0;0];
angVel_R = [0;0;0];
wristVel_R = [];
grip_R = [];
baxterP.calibrateGripper('right');
pause(2);

% Starting pose
x = tic;
y = 0;
while y < 5
    baxter.setControlMode(uint8(0));
    baxter.setJointCommand('left',[-0.7854;-1.0472;0;2.0944;0;-1.0472;-0]);
    baxter.setJointCommand('right',[0.7854;-1.0472;0;2.0944;0;-1.0472;-0]);
    y = toc(x);
end

calibrateJamboxx(jamboxx);
baxter.setControlMode(uint8(1));
clc; disp('READY TO MOVE...');

while(1) 
       
    % Gather joint information
    jointAngles = baxter.joint_positions;
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
        else
            qDot_L = [0;0;0;0;0;0;0];
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
        else
            qDot_R = [0;0;0;0;0;0;0];
        end
% This is the main file for Jamboxx based control of Baxter.
    
    baxter.setControlMode(uint8(1));
    % Publish joint position
	% Left arm
	baxter.setJointCommand('left',qDot_L);
	baxter.setJointCommand('right',qDot_R);
        
    % Publish grip position
        % Left gripper
        if ~isempty(grip_L)
            baxterP.setGripperPosition('left',double(grip_L));
        end
        %Right gripper% This is the main file for Jamboxx based control of Baxter.

        if ~isempty(grip_R)
            baxterP.setGripperPosition('right',double(grip_R));
        end
        pause(0.1);
end
clc; msgbox('Program stopped!');
@(x)terminated(disp(x));