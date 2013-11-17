clear all;close all;clc;
% This is the main file for GUI based control of Baxter.

%% Robot Raconteur Connections

% Add callback function path
addpath('./callbackFuncs');

% Connect to ROS Bridge for Baxter.
baxter = RobotRaconteur.Connect('tcp://localhost:34572/{0}/ROSBridge');

% Set up joint velocity publisher
    % Left arm
    handle_leftJointVel = baxter.publish(...
        '/robot/limb/left/command_joint_velocities',...
        'baxter_msgs/JointVelocities');
    publisher_leftJointVel = baxter.get_publishers(handle_leftJointVel);
    % Right arm
    handle_rightJointVel = baxter.publish(...
    '/robot/limb/right/command_joint_velocities',...
    'baxter_msgs/JointVelocities');
    publisher_rightJointVel = baxter.get_publishers(handle_rightJointVel);

% Set up joint command mode publisher
    % Left arm
    handle_leftMode = baxter.publish(...
        '/robot/limb/left/joint_command_mode',...
        'baxter_msgs/JointCommandMode');
    publisher_leftMode = baxter.get_publishers(handle_leftMode);
    % Right arm
    handle_rightMode = baxter.publish(...
    '/robot/limb/right/joint_command_mode',...
    'baxter_msgs/JointCommandMode');
    publisher_rightMode = baxter.get_publishers(handle_rightMode);

% Set up gripper control
    % Left arm
    handle_leftGrip = baxter.publish(...
        '/sdk/robot/limb/left/accessory/gripper/command_set',...
        'baxter_msgs/GripperCommand');
    publisher_leftGrip = baxter.get_publishers(handle_leftGrip);
    handle_leftGripCal = baxter.publish(...
        '/robot/limb/left/accessory/gripper/command_calibrate',...
        'std_msgs/Empty');
    publisher_leftGripCal = baxter.get_publishers(handle_leftGripCal);
    % Right arm
    handle_rightGrip = baxter.publish(...
        '/sdk/robot/limb/right/accessory/gripper/command_set',...
        'baxter_msgs/GripperCommand');
    publisher_rightGrip = baxter.get_publishers(handle_rightGrip);
    handle_rightGripCal = baxter.publish(...
        '/robot/limb/right/accessory/gripper/command_calibrate',...
        'std_msgs/Empty');
    publisher_rightGripCal = baxter.get_publishers(handle_rightGripCal);
    

% Set up joint states subscriber
handle_jointStates = baxter.subscribe('/robot/joint_states',...
    'sensor_msgs/JointState');
subscriber_jointStates = baxter.get_subscribers(handle_jointStates);
baxter_JointStates = subscriber_jointStates.subscriberwire.Connect();

%% Message type setup

% baxter_msgs/JointCommandMode
leftMode.mode = uint8(2);   % 1 for position mode; 2 is for velocity mode; 3 for torque mode
rightMode.mode = uint8(2);

% baxter_msgs/JointVelocities
    % Left arm
    leftJointVel.names = {{int32(0),'left_e0'};{int32(1),'left_e1'};...
        {int32(2),'left_s0'};{int32(3),'left_s1'};...
        {int32(4),'left_w0'};{int32(5),'left_w1'};...
        {int32(6),'left_w2'}};
    leftJointVel.velocities = [0;0;0;0;0;0;0];
    % Right arm
    rightJointVel.names = {{int32(0),'right_e0'};{int32(1),'right_e1'};...
        {int32(2),'right_s0'};{int32(3),'right_s1'};...
        {int32(4),'right_w0'};{int32(5),'right_w1'};...
        {int32(6),'right_w2'}};
    rightJointVel.velocities = [0;0;0;0;0;0;0];

% baxter_msgs/GripperCommand
    % Left gripper
    leftGrip = struct('position',single(100),...
        'force',single(100),'velocity',single(20),...
        'holding',single(100),'deadZone',single(3));
    % Right gripper
    rightGrip = struct('position',single(100),...
        'force',single(100),'velocity',single(20),...
        'holding',single(100),'deadZone',single(3));
    

%% GUI

% set(handles.pushbutton1,'string','...')

panel = figure('Visible','off','position',[50 500 1000 500]);

% Buttons for left arm
button1 = uicontrol('Style','togglebutton','position',[350 450 75 25],...
    'callback',@button1_Callback,'String','Forward');
button2 = uicontrol('Style','togglebutton','position',[350 415 75 25],...
    'callback',@button2_Callback,'String','Backward');
button3 = uicontrol('Style','togglebutton','position',[250 415 75 25],...
    'callback',@button3_Callback,'String','Left');
button4 = uicontrol('Style','togglebutton','position',[450 415 75 25],...
    'callback',@button4_Callback,'String','Right');
button5 = uicontrol('Style','togglebutton','position',[550 450 75 25],...
    'callback',@button5_Callback,'String','Up');
button6 = uicontrol('Style','togglebutton','position',[550 415 75 25],...
    'callback',@button6_Callback,'String','Down');
button7 = uicontrol('Style','togglebutton','position',[250 350 75 25],...
    'callback',@button7_Callback,'String','W0+');
button8 = uicontrol('Style','togglebutton','position',[250 315 75 25],...
    'callback',@button8_Callback,'String','W0-');
button9 = uicontrol('Style','togglebutton','position',[350 350 75 25],...
    'callback',@button9_Callback,'String','W1+');
button10 = uicontrol('Style','togglebutton','position',[350 315 75 25],...
    'callback',@button10_Callback,'String','W1-');
button11 = uicontrol('Style','togglebutton','position',[450 350 75 25],...
    'callback',@button11_Callback,'String','W2+');
button12 = uicontrol('Style','togglebutton','position',[450 315 75 25],...
    'callback',@button12_Callback,'String','W2-');
button13 = uicontrol('Style','togglebutton','position',[550 350 75 25],...
    'callback',@button13_Callback,'String','Grip Open');
button14 = uicontrol('Style','togglebutton','position',[550 315 75 25],...
    'callback',@button14_Callback,'String','Grip Close');
text1 = uicontrol('Style','text','position',[50 480 800 20],...
    'String','LEFT ARM');

% Buttons for right arm
button1_R = uicontrol('Style','togglebutton','position',[350 150 75 25],...
    'callback',@button1_R_Callback,'String','Forward');
button2_R = uicontrol('Style','togglebutton','position',[350 115 75 25],...
    'callback',@button2_R_Callback,'String','Backward');
button3_R = uicontrol('Style','togglebutton','position',[250 115 75 25],...
    'callback',@button3_R_Callback,'String','Left');
button4_R = uicontrol('Style','togglebutton','position',[450 115 75 25],...
    'callback',@button4_R_Callback,'String','Right');
button5_R = uicontrol('Style','togglebutton','position',[550 150 75 25],...
    'callback',@button5_R_Callback,'String','Up');
button6_R = uicontrol('Style','togglebutton','position',[550 115 75 25],...
    'callback',@button6_R_Callback,'String','Down');
button7_R = uicontrol('Style','togglebutton','position',[250 50 75 25],...
    'callback',@button7_R_Callback,'String','W0+');
button8_R = uicontrol('Style','togglebutton','position',[250 15 75 25],...
    'callback',@button8_R_Callback,'String','W0-');
button9_R = uicontrol('Style','togglebutton','position',[350 50 75 25],...
    'callback',@button9_R_Callback,'String','W1+');
button10_R = uicontrol('Style','togglebutton','position',[350 15 75 25],...
    'callback',@button10_R_Callback,'String','W1-');
button11_R = uicontrol('Style','togglebutton','position',[450 50 75 25],...
    'callback',@button11_R_Callback,'String','W2+');
button12_R = uicontrol('Style','togglebutton','position',[450 15 75 25],...
    'callback',@button12_R_Callback,'String','W2-');
button13_R = uicontrol('Style','togglebutton','position',[550 50 75 25],...
    'callback',@button13_R_Callback,'String','Grip Open');
button14_R = uicontrol('Style','togglebutton','position',[550 15 75 25],...
    'callback',@button14_R_Callback,'String','Grip Close');
text2 = uicontrol('Style','text','position',[50 200 800 20],...
    'String','RIGHT ARM');

stop = uicontrol('Style','pushbutton','position',[900 50 50 400],...
    'String','EXIT','callback',@stop_Callback);

%% Main

setBaxterConstants;
stopProg = 0;

% Initialize left arm params
linVel_L = [0;0;0];
angVel_L = [0;0;0];
wristVel_L = [];
grip_L = [];
publisher_leftGripCal.publish([]);
pause(3);

% Initialize right arm params
linVel_R = [0;0;0];
angVel_R = [0;0;0];
wristVel_R = [];
grip_R = [];
publisher_rightGripCal.publish([]);
pause(3);

clc; disp('READY TO MOVE');
set(panel,'Visible','on');
while(1) 
    
    if stopProg
        close all;
        break;
    end
    
    drawnow;
    
    % Gather joint information
    pause(0.01);
    [jointAnglesLeft,jointAnglesRight] = getJointAngles(...
        baxter_JointStates);
    
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
        leftJointVel.velocities = [qDot_L(3);qDot_L(4);...
            qDot_L(1);qDot_L(2);qDot_L(5);...
            qDot_L(6);qDot_L(7)];
        if ~isempty(wristVel_L)
            leftJointVel.velocities(5:7) = wristVel_L;
        end
        publisher_leftMode.publish(leftMode);
        publisher_leftJointVel.publish(leftJointVel);
        %Right arm
        rightJointVel.velocities = [qDot_R(3);qDot_R(4);...
            qDot_R(1);qDot_R(2);qDot_R(5);...
            qDot_R(6);qDot_R(7)];
        if ~isempty(wristVel_R)
            rightJointVel.velocities(5:7) = wristVel_R;
        end
        publisher_rightMode.publish(rightMode);
        publisher_rightJointVel.publish(rightJointVel);
    
    % Publish grip position
        % Left gripper
        if ~isempty(grip_L)
            leftGrip.position = grip_L;
            publisher_leftGrip.publish(leftGrip);
        end
        %Right gripper
        if ~isempty(grip_R)
            rightGrip.position = grip_R;
            publisher_rightGrip.publish(rightGrip);
        end
    
end
clc; msgbox('Program stopped!');