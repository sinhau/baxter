clear all;close all;clc;
% This is the main file!

%% Robot Raconteur Connections

% Connect to ROS Bridge for Baxter.
baxter = RobotRaconteur.Connect('tcp://localhost:34572/{0}/ROSBridge');

% Set up joint velocity publisher
handle_leftJointVel = baxter.publish('/robot/limb/left/command_joint_velocities',...
    'baxter_msgs/JointVelocities');
publisher_leftJointVel = baxter.get_publishers(handle_leftJointVel);
handle_rightJointVel = baxter.publish('/robot/limb/right/command_joint_velocities',...
    'baxter_msgs/JointVelocities');
publisher_rightJointVel = baxter.get_publishers(handle_rightJointVel);

% Set up joint command mode publisher
handle_leftMode = baxter.publish('/robot/limb/left/joint_command_mode',...
    'baxter_msgs/JointCommandMode');
publisher_leftMode = baxter.get_publishers(handle_leftMode);
handle_rightMode = baxter.publish('/robot/limb/right/joint_command_mode',...
    'baxter_msgs/JointCommandMode');
publisher_rightMode = baxter.get_publishers(handle_rightMode);

% Set up gripper control
handle_rightGrip = baxter.publish('/sdk/robot/limb/right/accessory/gripper/command_set',...
    'baxter_msgs/GripperCommand');
publisher_rightGrip = baxter.get_publishers(handle_rightGrip);
handle_rightGripCal = baxter.publish('/robot/limb/right/accessory/gripper/command_calibrate',...
    'std_msgs/Empty');
publisher_rightGripCal = baxter.get_publishers(handle_rightGripCal);
handle_leftGrip = baxter.publish('/sdk/robot/limb/left/accessory/gripper/command_set',...
    'baxter_msgs/GripperCommand');
publisher_leftGrip = baxter.get_publishers(handle_leftGrip);
handle_leftGripCal = baxter.publish('/robot/limb/left/accessory/gripper/command_calibrate',...
    'std_msgs/Empty');
publisher_leftGripCal = baxter.get_publishers(handle_leftGripCal);

% Set up joint states subscriber
handle_jointStates = baxter.subscribe('/robot/joint_states', 'sensor_msgs/JointState');
subscriber_jointStates = baxter.get_subscribers(handle_jointStates);
baxter_JointStates = subscriber_jointStates.subscriberwire.Connect();

%% Message type setup

% baxter_msgs/JointCommandMode
leftMode.mode = uint8(2);   % 1 for position mode; 2 is for velocity mode; 3 for torque mode
rightMode.mode = uint8(2); 

% baxter_msgs/JointVelocities
leftJointVel.names = {{int32(0),'left_e0'};{int32(1),'left_e1'};{int32(2),'left_s0'};...
    {int32(3),'left_s1'};{int32(4),'left_w0'};{int32(5),'left_w1'};{int32(6),'left_w2'}};
leftJointVel.velocities = [0;0;0;0;0;0;0];
rightJointVel.names = {{int32(0),'right_e0'};{int32(1),'right_e1'};{int32(2),'right_s0'};...
    {int32(3),'right_s1'};{int32(4),'right_w0'};{int32(5),'right_w1'};{int32(6),'right_w2'}};
rightJointVel.velocities = [0;0;0;0;0;0;0];

% baxter_msgs/GripperCommand
rightGrip = struct('position',single(100),'force',single(100),'velocity',single(20),...
    'holding',single(100),'deadZone',single(3));
leftGrip = struct('position',single(100),'force',single(100),'velocity',single(20),...
    'holding',single(100),'deadZone',single(3));


%% Main

kbhit('init');
setBaxterConstants;
publisher_rightGripCal.publish([]);
pause(3);
publisher_leftGripCal.publish([]);
pause(3);

disp('Ready to move!');
 
while(1) 
      
    % Gather joint information
    [jointAnglesLeft,jointAnglesRight] = getJointAngles(baxter_JointStates);
    
    % Calculate full jacobian for left arm
    leftJ = jacobian(baxterConst.leftArm,jointAnglesLeft);
    rightJ = jacobian(baxterConst.rightArm,jointAnglesRight);
    
    % Input desired delta position
    deltaPosL = [0;0;0];
    deltaXL = 0.04; deltaYL = 0.04; deltaZL = 0.06;
    angVelL = [0;0;0];
    deltaPosR = [0;0;0];
    deltaXR = 0.04; deltaYR = 0.04; deltaZR = 0.06;
    angVelR = [0;0;0];
    
    
%     clc;
%     display('Press w(forward),a(left),s(backward),d(right),q(up),e(down),x(stop)');
    
    pause(0.01);
    keyPress = kbhit;
    if isempty(keyPress)
        keyPress = 'nothing';
    end
    switch keyPress
        % Cases for left arm
        case 'W'
            deltaPosL(1) = deltaXL;
        case 'A'
            deltaPosL(2) = deltaYL; 
        case 'S'
            deltaPosL(1) = -deltaXL; 
        case 'D'
            deltaPosL(2) = -deltaYL;   
        case 'Q'
            deltaPosL(3) = deltaZL;
        case 'E'
            deltaPosL(3) = -deltaZL; 
        case '1'
            leftJointVel.velocities = [0;0;0;0;0.5;0;0];
            publisher_leftMode.publish(leftMode);
            publisher_leftJointVel.publish(leftJointVel);            
        case '2'
            leftJointVel.velocities = [0;0;0;0;-0.5;0;0];
            publisher_leftMode.publish(leftMode);
            publisher_leftJointVel.publish(leftJointVel);             
        case '3'
            leftJointVel.velocities = [0;0;0;0;0;0.5;0];
            publisher_leftMode.publish(leftMode);
            publisher_leftJointVel.publish(leftJointVel);  
        case '4'
            leftJointVel.velocities = [0;0;0;0;0;-0.5;0];
            publisher_leftMode.publish(leftMode);
            publisher_leftJointVel.publish(leftJointVel);  
        case '5'
            leftJointVel.velocities = [0;0;0;0;0;0;0.5];
            publisher_leftMode.publish(leftMode);
            publisher_leftJointVel.publish(leftJointVel);  
        case '6'
            leftJointVel.velocities = [0;0;0;0;0;0;-0.5];
            publisher_leftMode.publish(leftMode);
            publisher_leftJointVel.publish(leftJointVel);  
        case 'Z'
            leftGrip.position = single(0);
            publisher_leftGrip.publish(leftGrip);
        case 'C'
            leftGrip.position = single(100);
            publisher_leftGrip.publish(leftGrip);
        % Cases for right arm
        case 'I'
            deltaPosR(1) = deltaXR;
        case 'J'
            deltaPosR(2) = deltaYR; 
        case 'K'
            deltaPosR(1) = -deltaXR; 
        case 'L'
            deltaPosR(2) = -deltaYR;   
        case 'U'
            deltaPosR(3) = deltaZR;
        case 'O'
            deltaPosR(3) = -deltaZR; 
        case '7'
            rightJointVel.velocities = [0;0;0;0;0.5;0;0];
            publisher_rightMode.publish(rightMode);
            publisher_rightJointVel.publish(rightJointVel);            
        case '8'
            rightJointVel.velocities = [0;0;0;0;-0.5;0;0];
            publisher_rightMode.publish(rightMode);
            publisher_rightJointVel.publish(rightJointVel);             
        case '9'
            rightJointVel.velocities = [0;0;0;0;0;0.5;0];
            publisher_rightMode.publish(rightMode);
            publisher_rightJointVel.publish(rightJointVel);  
        case '0'
            rightJointVel.velocities = [0;0;0;0;0;-0.5;0];
            publisher_rightMode.publish(rightMode);
            publisher_rightJointVel.publish(rightJointVel);  
        case '-'
            rightJointVel.velocities = [0;0;0;0;0;0;0.5];
            publisher_rightMode.publish(rightMode);
            publisher_rightJointVel.publish(rightJointVel);  
        case '='
            rightJointVel.velocities = [0;0;0;0;0;0;-0.5];
            publisher_rightMode.publish(rightMode);
            publisher_rightJointVel.publish(rightJointVel);  
        case 'M'
            rightGrip.position = single(0);
            publisher_rightGrip.publish(rightGrip);
        case '.'
            rightGrip.position = single(100);
            publisher_rightGrip.publish(rightGrip);
        case ' '
            deltaPosL = [0;0;0];
            angVelL = [0;0;0];
            deltaPosR = [0;0;0];
            angVelR = [0;0;0];
    end
    deltaPosCorrectL = rot([0;0;1],pi/4)*deltaPosL; % Rotate frame such that x-axis points in front of baxter
    deltaPoseL = [angVelL;deltaPosCorrectL];
    deltaPosCorrectR = rot([0;0;1],3*pi/4)*deltaPosR; % Rotate frame such that x-axis points in front of baxter
    deltaPoseR = [angVelR;deltaPosCorrectR];
    
    % Calculate desired joint angle velocities for left arm
    dampCoeffL = 0.1;
    if any(deltaPoseL)
        qDotL = leftJ'*pinv(leftJ*leftJ' + dampCoeffL^2*eye(6,6))*deltaPoseL;
        % Limit angular joint velocity
        for k = 1:length(qDotL)
            if abs(qDotL(k)) > baxterConst.jointVelLimit(k)
                qDotL(k) = sign(qDotL(k))*baxterConst.jointVelLimit(k);
            end
        end     
    else
        qDotL = [0;0;0;0;0;0;0];
    end
     % Calculate desired joint angle velocities for right arm
    dampCoeffR = 0.1;
    if any(deltaPoseR)
        qDotR = rightJ'*pinv(rightJ*rightJ' + dampCoeffR^2*eye(6,6))*deltaPoseR;
        % Limit angular joint velocity
        for k = 1:length(qDotR)
            if abs(qDotR(k)) > baxterConst.jointVelLimit(k)
                qDotR(k) = sign(qDotR(k))*baxterConst.jointVelLimit(k);
            end
        end     
    else
        qDotR = [0;0;0;0;0;0;0];
    end

    % Publish desired joint velocities
    leftJointVel.velocities = [qDotL(3);qDotL(4);qDotL(1);qDotL(2);qDotL(5);qDotL(6);qDotL(7)];
    publisher_leftMode.publish(leftMode);
    publisher_leftJointVel.publish(leftJointVel);
    rightJointVel.velocities = [qDotR(3);qDotR(4);qDotR(1);qDotR(2);qDotR(5);qDotR(6);qDotR(7)];
    publisher_rightMode.publish(rightMode);
    publisher_rightJointVel.publish(rightJointVel);
    
end