clear all;close all;clc;
% This is the main file for keyboard based control of Baxter.

%% Robot Raconteur Connections

% Connect to Baxter Bridge
baxter = RobotRaconteur.Connect('tcp://localhost:4683/BaxterServer/Baxter');

%% Main

kbhit('init');
setBaxterConstants;
baxter.GripperCalibrate('left');
pause(3);
%publisher_leftGripCal.publish([]);
%pause(3);

disp('Ready to move!');
 
while(1) 
      
    % Gather joint information
    pause(0.01);
    jointAngles = baxter.JointPositions;
    jointAnglesLeft = jointAngles(1:7);
    jointAnglesRight = jointAngles(8:14);
    
    % Calculate full jacobian for left arm
    leftJ = jacobian(baxterConst.leftArm,jointAnglesLeft);
    rightJ = jacobian(baxterConst.rightArm,jointAnglesRight);
    
    % Input desired delta position
    deltaPosL = [0;0;0];
    deltaXL = 0.04; deltaYL = 0.04; deltaZL = 0.06;
    angVelL = [0;0;0];
    wristVel_L = [];
    deltaPosR = [0;0;0];
    deltaXR = 0.04; deltaYR = 0.04; deltaZR = 0.06;
    angVelR = [0;0;0];
    wristVel_R = []; 
    
    % Keyboard input
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
            wristVel_L = [0.5;0;0];             
        case '2'
            wristVel_L = [-0.5;0;0];               
        case '3'
            wristVel_L = [0;0.5;0];    
        case '4'
            wristVel_L = [0;-0.5;0];    
        case '5'
            wristVel_L = [0;0;1];   
        case '6'
            wristVel_L = [0;0;-1];   
        case 'Z'
            baxter.setGripperPosition('left',double(0));
        case 'C'
            baxter.setGripperPosition('left',double(100));
        % Cases for right arm
%         case 'I'
%             deltaPosR(1) = deltaXR;
%         case 'J'
%             deltaPosR(2) = deltaYR; 
%         case 'K'
%             deltaPosR(1) = -deltaXR; 
%         case 'L'
%             deltaPosR(2) = -deltaYR;   
%         case 'U'
%             deltaPosR(3) = deltaZR;
%         case 'O'
%             deltaPosR(3) = -deltaZR; 
%         case '7'
%             wristVel_R = [0.5;0;0];          
%         case '8'
%             wristVel_R = [-0.5;0;0];               
%         case '9'
%             wristVel_R = [0;0.5;0];   
%         case '0'
%             wristVel_R = [0;-0.5;0];   
%         case '-'
%             wristVel_R = [0;0;1];    
%         case '='
%             wristVel_R = [0;0;-1];   
%         case 'M'
%             rightGrip.position = single(0);
%             publisher_rightGrip.publish(rightGrip);
%         case '.'
%             rightGrip.position = single(100);
%             publisher_rightGrip.publish(rightGrip);
        case ' '
            deltaPosL = [0;0;0];
            angVelL = [0;0;0];
            deltaPosR = [0;0;0];
            angVelR = [0;0;0];
            wristVel_R = [0;0;0];
            wristVel_L = [0;0;0];
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
%     dampCoeffR = 0.1;
%     if any(deltaPoseR)
%         qDotR = rightJ'*pinv(rightJ*rightJ' + dampCoeffR^2*eye(6,6))*deltaPoseR;
%         % Limit angular joint velocity
%         for k = 1:length(qDotR)
%             if abs(qDotR(k)) > baxterConst.jointVelLimit(k)
%                 qDotR(k) = sign(qDotR(k))*baxterConst.jointVelLimit(k);
%             end
%         end     
%     else
%         qDotR = [0;0;0;0;0;0;0];
%     end

    % Publish desired joint velocities for left arm
    leftJointVel = [qDotL(3);qDotL(4);...
        qDotL(1);qDotL(2);qDotL(5);qDotL(6);qDotL(7)];
    if ~isempty(wristVel_L)
        leftJointVel(5:7) = wristVel_L;
    end
    baxter.setJointVelocity('left',leftJointVel);
    % Publish desired joint velocities for right arm
%     rightJointVel.velocities = [qDotR(3);qDotR(4);...
%         qDotR(1);qDotR(2);qDotR(5);qDotR(6);qDotR(7)];
%     if ~isempty(wristVel_R)
%         rightJointVel.velocities(5:7) = wristVel_R;
%     end
%     publisher_rightMode.publish(rightMode);
%     publisher_rightJointVel.publish(rightJointVel);
    
end