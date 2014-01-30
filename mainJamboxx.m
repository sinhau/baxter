clear all;close all;clc;
% This is the main file for Jamboxx based control of Baxter.

%% Robot Raconteur Connections

% Baxter connections
baxter = RobotRaconteur.Connect('tcp://localhost:4682/BaxterServer/Baxter');

% Jamboxx connection
jamboxx = RobotRaconteur.Connect('tcp://192.168.1.104:5318/{0}/Jamboxx');

% Xbox 360 controller connection
try
  xbox = RobotRaconteur.Connect('tcp://192.168.1.104:5437/Xbox_controllerServer/xbox_controller');
  xboxIsConnected=1;
catch ME
  disp(ME.message);
  disp('XBox RobotRaconteur service was not running.');
end
%xbox = RobotRaconteur.Connect('tcp://192.168.1.104:5437/Xbox_controllerServer/xbox_controller');

%% Initialize Serial Communication
s1 = serial('/dev/ttyS105');    % define serial port
set(s1, 'DataBits', 8);
set(s1, 'StopBits', 1);
s1.BaudRate=9600;               % define baud rate
set(s1, 'Parity', 'none');
fopen(s1); 

fprintf(s1,'%c', 'w');
pause(2);

%% Main

setBaxterConstants;
stopProg = 0;

% Initialize left arm params
linVel_L = [0;0;0];
angVel_L = [0;0;0];
wristVel_L = [];
grip_L = [];
baxter.GripperCalibrate('left');
pause(3);

% Initialize right arm params
linVel_R = [0;0;0];
angVel_R = [0;0;0];
wristVel_R = [];
grip_R = [];
% publisher_rightGripCal.publish([]);
% pause(3);

clc; disp('READY TO MOVE');
while(1) 

    if stopProg
        close all;
        break;
    end
    pause(0.01);
     xBoxInput = xbox.controller_input;     
     xInput = (xBoxInput.left_thumbstick_X*-1 + 10000)* 130 /19000 + 136;
     yInput = (xBoxInput.left_thumbstick_Y + 10000)* 130 /20000 - 19;
     if(xInput > 255)
         xInput = 255;
     end
     
     if(xInput < 158)
         xInput = 158;
     end
     
     if(yInput > 115)
         yInput = 115;
     end
     
     if(yInput < 0)
         yInput = 0;
 
     end
     disp(xInput);
     disp(yInput);    
     
     fwrite(s1,char(xInput), 'char');
     fwrite(s1,char(yInput), 'char');
     pause(.01);
 
    % Gather joint information
    pause(0.01);
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
        if any(allVel_L)
            qDot_L = leftJ'*pinv(leftJ*leftJ' + dampCoeff_L^2*eye(6,6))*allVel_L; %Damped least squares
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
        if any(allVel_R)
            qDot_R = rightJ'*pinv(rightJ*rightJ' + dampCoeff_R^2*eye(6,6))*allVel_R; %Damped least squares
            % Limit joint velocity
            for k = 1:length(qDot_R)
                if abs(qDot_R(k)) > baxterConst.jointVelLimit(k)
                    qDot_R(k) = sign(qDot_R(k))*baxterConst.jointVelLimit(k);
                end
            end     
        else
            qDot_R = [0;0;0;0;0;0;0];
        end

    % Publish desired joint velocities
        % Left arm
        leftJointVel = [qDot_L(1);qDot_L(2);...
            qDot_L(3);qDot_L(4);qDot_L(5);...
            qDot_L(6);qDot_L(7)];
        if ~isempty(wristVel_L)
            leftJointVel(5:7) = wristVel_L;
        end
        baxter.setJointVelocity('left',leftJointVel);
        %Right arm
        rightJointVel = [qDot_R(1);qDot_R(2);...
            qDot_R(3);qDot_R(4);qDot_R(5);...
            qDot_R(6);qDot_R(7)];
        if ~isempty(wristVel_R)
            rightJointVel(5:7) = wristVel_R;
        end
        baxter.setJointVelocity('right',rightJointVel);
    
    % Publish grip position
        % Left gripper
        if ~isempty(grip_L)
            baxter.setGripperPosition('left',double(grip_L));
        end
        %Right gripper
        if ~isempty(grip_R)
            baxter.setGripperPosition('right',double(grip_R));
        end
    
end
clc; msgbox('Program stopped!');