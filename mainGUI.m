clear all;close all;clc;
% This is the main file for GUI based control of Baxter.

%% Robot Raconteur Connections

% Add callback function path
addpath('./callbackFuncs');

% Connect to ROS Bridge for Baxter.
baxter = RobotRaconteur.Connect('tcp://localhost:4682/BaxterServer/Baxter');

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
    'callbac
k',@button14_R_Callback,'String','Grip Close');
text2 = uicontrol('Style','text','position',[50 200 800 20],...
    'String','RIGHT ARM');

stop = uicontrol('Style','pushbutton','position',[900 50 50 400],...
    'String','EXIT','callback',@stop_Callback);

%% Main2_Callback,'String','Backward');

setBaxterConstants;
stopProg = 0;

% Initialize left arm params
linVel_L = [0;0;0];
angVel_L = [0;0;0];
wristVel_L = [];
grip_L = [];
baxter.Gripp
erCalibrate('left');
pause(3);

% Initialize right arm params
linVel_R = [0;0;0];
angVel_R = [0;0;0];
wristVel_R = [];
grip_R = [];
% publisher_rightGripCal.publish([]);
% pause(3);

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
    jointAngles = baxter.JointPositions;
    jointAnglesLeft = jointAngles(1:7);
    jointAnglesRight = jointAngles(8:14);
    
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
%         rightJointVel.velocities = [qDot_R(3);qDot_R(4);...
%             qDot_R(1);qDot_R(2);qDot_R(5);...
%             qDot_R(6);qDot_R(7)];
%         if ~isempty(wristVel_R)
%             rightJointVel.velocities(5:7) = wristVel_R;
%         end
%         publisher_rightMode.publish(rightMode);
%         publisher_rightJointVel.publish(rightJointVel);
    
    % Publish grip position
        % Left gripper
        if ~isempty(grip_L)
            baxter.setGripperPosition('left',double(grip_L));
        end
        %Right gripper
%         if ~isempty(grip_R)
%             rightGrip.position = grip_R;
%             publisher_rightGrip.publish(rightGrip);
%         end
    
end
clc; msgbox('Program stopped!');