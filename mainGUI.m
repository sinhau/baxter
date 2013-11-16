clear all;close all;clc;
% This is the main file!

%% Robot Raconteur Connections

% Connect to ROS Bridge for Baxter.
baxter = RobotRaconteur.Connect('tcp://localhost:34572/{0}/ROSBridge');

% Set up joint velocity publisher
handle_leftJointVel = baxter.publish('/robot/limb/left/command_joint_velocities',...
    'baxter_msgs/JointVelocities');
publisher_leftJointVel = baxter.get_publishers(handle_leftJointVel);

% Set up joint command mode publisher
handle_leftMode = baxter.publish('/robot/limb/left/joint_command_mode',...
    'baxter_msgs/JointCommandMode');
publisher_leftMode = baxter.get_publishers(handle_leftMode);

% Set up joint states subscriber
handle_jointStates = baxter.subscribe('/robot/joint_states',...
    'sensor_msgs/JointState');
subscriber_jointStates = baxter.get_subscribers(handle_jointStates);
baxter_JointStates = subscriber_jointStates.subscriberwire.Connect();

%% Message type setup

% baxter_msgs/JointCommandMode
leftMode.mode = uint8(2);   % 1 for position mode; 2 is for velocity mode; 3 for torque mode

% baxter_msgs/JointVelocities
leftJointVel.names = {{int32(0),'left_e0'};{int32(1),'left_e1'};...
    {int32(2),'left_s0'};{int32(3),'left_s1'};{int32(4),'left_w0'};...
    {int32(5),'left_w1'};{int32(6),'left_w2'}};
leftJointVel.velocities = [0;0;0;0;0;0;0];

%% GUI

% set(handles.pushbutton1,'string','...')

panel = figure('Visible','off','position',[50 500 1000 500]);

% Buttons for left arm
button1 = uicontrol('Style','togglebutton','position',[150 450 75 25],...
    'callback',@button1_Callback,'String','Forward');
button2 = uicontrol('Style','togglebutton','position',[150 400 75 25],...
    'callback',@button2_Callback,'String','Backward');
button3 = uicontrol('Style','togglebutton','position',[50 400 75 25],...
    'callback',@button3_Callback,'String','Left');
button4 = uicontrol('Style','togglebutton','position',[250 400 75 25],...
    'callback',@button4_Callback,'String','Right');
button5 = uicontrol('Style','togglebutton','position',[350 450 75 25],...
    'callback',@button5_Callback,'String','Up');
button6 = uicontrol('Style','togglebutton','position',[350 400 75 25],...
    'callback',@button6_Callback,'String','Down');
button7 = uicontrol('Style','togglebutton','position',[150 300 75 25],...
    'callback',@button7_Callback,'String','Pitch +');
button8 = uicontrol('Style','togglebutton','position',[150 250 75 25],...
    'callback',@button8_Callback,'String','Pitch -');
button9 = uicontrol('Style','togglebutton','position',[50 250 75 25],...
    'callback',@button9_Callback,'String','Roll -');
button10 = uicontrol('Style','togglebutton','position',[250 250 75 25],...
    'callback',@button10_Callback,'String','Roll +');
button11 = uicontrol('Style','togglebutton','position',[50 200 75 25],...
    'callback',@button11_Callback,'String','Yaw +');
button12 = uicontrol('Style','togglebutton','position',[250 200 75 25],...
    'callback',@button12_Callback,'String','Yaw -');

% Buttons for right arm


set(panel,'Visible','on');




%% Main

setBaxterConstants;
linVel = [0;0;0];
angVel = [0;0;0];

while(1) 
      
    drawnow; 
    
    % Gather joint information
    [jointAnglesLeft,jointAnglesRight] = getJointAngles(baxter_JointStates);
    
    % Calculate full jacobian for both arms
    leftJ = jacobian(baxterConst.leftArm,jointAnglesLeft);
    rightJ = jacobian(baxterConst.rightArm,jointAnglesRight);
    
    % Desired velocities
    linVelCorrect = rot([0;0;1],pi/4)*linVel; % Rotate frame such that x-axis points in front of baxter
    angVelCorrect = rot([0;0;1],pi/4)*angVel; % Rotate frame such that x-axis points in front of baxter
    allVel = [angVel;linVelCorrect];
    
    % Calculate desired joint angle velocities
    dampCoeff = 0.1;
    if any(allVel)
        qDot = leftJ'*pinv(leftJ*leftJ' + dampCoeff^2*eye(6,6))*allVel; %Damped least squares
        % Limit joint velocity
        for k = 1:length(qDot)
            if abs(qDot(k)) > baxterConst.jointVelLimit(k)
                qDot(k) = sign(qDot(k))*baxterConst.jointVelLimit(k);
            end
        end     
    else
        qDot = [0;0;0;0;0;0;0];
    end

    % Publish desired joint velocities
    leftJointVel.velocities = [qDot(3);qDot(4);qDot(1);qDot(2);qDot(5);...
        qDot(6);qDot(7)];
    publisher_leftMode.publish(leftMode);
    publisher_leftJointVel.publish(leftJointVel);
    
end