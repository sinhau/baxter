clear all;close all;clc;
% This is the main file!

%% Robot Raconteur Connections

% Connect to ROS Bridge for Baxter.
baxter = RobotRaconteur.Connect('tcp://localhost:34572/{0}/ROSBridge');

% Set up joint velocity publisher
handle_leftJointVel = baxter.publish('/robot/limb/left/command_joint_velocities', 'baxter_msgs/JointVelocities');
publisher_leftJointVel = baxter.get_publishers(handle_leftJointVel);

% Set up joint command mode publisher
handle_leftMode = baxter.publish('/robot/limb/left/joint_command_mode', 'baxter_msgs/JointCommandMode');
publisher_leftMode = baxter.get_publishers(handle_leftMode);

% Set up joint states subscriber
handle_jointStates = baxter.subscribe('/robot/joint_states', 'sensor_msgs/JointState');
subscriber_jointStates = baxter.get_subscribers(handle_jointStates);
baxter_JointStates = subscriber_jointStates.subscriberwire.Connect();

%% Message type setup

% baxter_msgs/JointCommandMode
leftMode.mode = uint8(2);   % 1 for position mode; 2 is for velocity mode; 3 for torque mode

% baxter_msgs/JointVelocities
leftJointVel.names = {{int32(0),'left_e0'};{int32(1),'left_e1'};{int32(2),'left_s0'};{int32(3),'left_s1'};{int32(4),'left_w0'};{int32(5),'left_w1'};{int32(6),'left_w2'}};
leftJointVel.velocities = [0;0;0;0;0;0;0];

%% GUI

% set(handles.pushbutton1,'string','...')

panel = figure('Visible','off');

button1 = uicontrol('Style','pushbutton','position',[20 350 100 25],...
    'callback',@button1_Callback,'String','Forward');
button2 = uicontrol('Style','pushbutton','position',[20 300 100 25],...
    'callback',@button2_Callback,'String','Backward');
button3 = uicontrol('Style','pushbutton','position',[20 250 100 25],...
    'callback',@button3_Callback,'String','Left');
button4 = uicontrol('Style','pushbutton','position',[20 200 100 25],...
    'callback',@button4_Callback,'String','Right');

set(panel,'Visible','on');




%% Main

kbhit('init');
setBaxterConstants;

%setPose(baxter,1,baxterConst);

while(1) 
      
    % Gather joint information
    [jointAnglesLeft,~] = getJointAngles(baxter_JointStates);
    
    % Calculate full jacobian for left arm
    J = jacobian(baxterConst.leftArm,jointAnglesLeft);
    
    % Input desired delta position
    deltaPos = [0;0;0];
    deltaX = 0.04; deltaY = 0.04; deltaZ = 0.06;
    angVel = [0;0;0];
    rotX = 0.2; rotY = 0.2; rotZ = 0.5;
    
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
        case 'J'
            angVel(1) = -rotX;
        case 'L'
            angVel(1) = rotX;
        case 'I'
            angVel(2) = rotY;
        case 'K'
            angVel(2) = -rotY;
        case 'U'
            angVel(3) = rotZ;
        case 'O'
            angVel(3) = -rotZ;
        case 'X'
            deltaPos = [0;0;0];
            angVel = [0;0;0];
    end
    deltaPosCorrect = rot([0;0;1],pi/4)*deltaPos; % Rotate frame such that x-axis points in front of baxter
    angVelCorrect = rot([0;0;1],pi/4)*angVel;
    deltaPose = [angVelCorrect;deltaPosCorrect];
    
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
    leftJointVel.velocities = [qDot(3);qDot(4);qDot(1);qDot(2);qDot(5);qDot(6);qDot(7)];
    publisher_leftMode.publish(leftMode);
    publisher_leftJointVel.publish(leftJointVel);
    
end