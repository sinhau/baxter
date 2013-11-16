clear all;close all;clc;
% This is the main file!

%% Robot Raconteur Connections

% Connect to ROS Bridge for Baxter.
baxter = RobotRaconteur.Connect('tcp://localhost:34572/{0}/ROSBridge');

% Set up joint velocity publisher
handle_rightJointVel = baxter.publish('/robot/limb/right/command_joint_velocities', 'baxter_msgs/JointVelocities');
publisher_rightJointVel = baxter.get_publishers(handle_rightJointVel);

% Set up joint command mode publisher
handle_rightMode = baxter.publish('/robot/limb/right/joint_command_mode', 'baxter_msgs/JointCommandMode');
publisher_rightMode = baxter.get_publishers(handle_rightMode);

% Set up gripper control
handle_rightGrip = baxter.publish('/sdk/robot/limb/right/accessory/gripper/command_set', 'baxter_msgs/GripperCommand');
publisher_rightGrip = baxter.get_publishers(handle_rightGrip);

% Set up head pan
handle_headPan = baxter.publish('/sdk/robot/head/command_head_pan', 'baxter_msgs/HeadPanCommand');
publisher_headPan = baxter.get_publishers(handle_headPan);

% Set up joint states subscriber
handle_jointStates = baxter.subscribe('/robot/joint_states', 'sensor_msgs/JointState');
subscriber_jointStates = baxter.get_subscribers(handle_jointStates);
baxter_JointStates = subscriber_jointStates.subscriberwire.Connect();

%% Message type setup

% baxter_msgs/JointCommandMode
rightMode.mode = uint8(2);   % 1 for position mode; 2 is for velocity mode; 3 for torque mode

% baxter_msgs/JointVelocities
rightJointVel.names = {{int32(0),'right_e0'};{int32(1),'right_e1'};{int32(2),'right_s0'};{int32(3),'right_s1'};{int32(4),'right_w0'};{int32(5),'right_w1'};{int32(6),'right_w2'}};
rightJointVel.velocities = [0;0;0;0;0;0;0];

% baxter_msgs/GripperCommand
rightGrip = struct('position',single(100),'force',single(100),'velocity',single(20),'holding',single(100),'deadZone',single(3));

% baxter_msgs/HeadPanCommand
headPan = struct('target',single(0),'speed',int32(50));

%% Main

kbhit('init');
setBaxterConstants;

%setPose(baxter,1,baxterConst);

while(1) 
      
    % Gather joint information
    pause(0.01);
    [jointAnglesLeft,jointAnglesRight] = getJointAngles(baxter_JointStates);
    [~,pRight] = fwdKin(baxterConst.rightArm,jointAnglesRight);
    theta = atan2(pRight(1),pRight(2));
    headPan.target = single(-theta-pi/3)
    publisher_headPan.publish(headPan);
    
    % Calculate full jacobian for right arm
    J = jacobian(baxterConst.rightArm,jointAnglesRight);
    
    % Input desired delta position
    deltaPos = [0;0;0];
    deltaX = 0.04; deltaY = 0.04; deltaZ = 0.06;
%     clc;
%     display('Press w(forward),a(left),s(backward),d(right),q(up),e(down),x(stop)');
    pause(0.01);
    keyPress = kbhit;
    if isempty(keyPress)
        keyPress = 'nothing';
    end
    switch keyPress
        case 'W'
            deltaPos(1) = deltaX;
        case 'A'
            deltaPos(2) = deltaY; 
        case 'S'
            deltaPos(1) = -deltaX; 
        case 'D'
            deltaPos(2) = -deltaY;   
        case 'Q'
            deltaPos(3) = deltaZ;
        case 'E'
            deltaPos(3) = -deltaZ; 
        case 'P'
            rightGrip.position = single(0);
            publisher_rightGrip.publish(rightGrip);
        case 'O'
            rightGrip.position = single(100);
            publisher_rightGrip.publish(rightGrip);
        otherwise 
            deltaPos = [0;0;0];         
    end
    deltaPosCorrect = rot([0;0;1],3*pi/4)*deltaPos; % Rotate frame such that x-axis points in front of baxter
    deltaPose = [0;0;0;deltaPosCorrect];
    
    % Calculate desired joint angle velocities
    dampCoeff = 0.1;
    if any(deltaPose)
        qDot = J'*pinv(J*J' + dampCoeff^2*eye(6,6))*deltaPose;
        % Limit angular joint velocity
        for k = 1:length(qDot)
            if abs(qDot(k)) > baxterConst.jointVelLimit(k)
                qDot(k) = sign(qDot(k))*baxterConst.jointVelLimit(k);
            end
        end     
    else
        qDot = [0;0;0;0;0;0;0];
    end

    % Publish desired joint velocities
    rightJointVel.velocities = [qDot(3);qDot(4);qDot(1);qDot(2);qDot(5);qDot(6);qDot(7)];
    publisher_rightMode.publish(rightMode);
    publisher_rightJointVel.publish(rightJointVel);
    
end