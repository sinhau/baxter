% This is the main file for Jamboxx based control of Baxter.

%% Robot Raconteur Connections

% Baxter joint connections
try
    baxterJ = RobotRaconteur.Connect('tcp://localhost:4691/BaxterJointServer/Baxter');
catch err
    disp(['CANNOT CONNECT TO BAXTER JOINT SERVER:',err.message]);
end

% Baxter peripherals connections
try
    baxterP = RobotRaconteur.Connect('tcp://localhost:4690/BaxterPeripheralServer/BaxterPeripherals');
catch err
    disp(['CANNOT CONNECT TO BAXTER PERIPHERALS SERVER:',err.message]);
end

% Jamboxx connection
try
    jamboxx = RobotRaconteur.Connect('tcp://192.168.1.110:5318/{0}/Jamboxx');
catch err
    disp(['CANNOT CONNECT TO JAMBOXX SERVER:',err.message]);
end

% Xbox 360 controller connection
try
    xbox = RobotRaconteur.Connect('tcp://192.168.1.110:5437/Xbox_controllerServer/xbox_controller');
catch err
    disp(['CANNOT CONNECT TO XBOX SERVER:',err.message]);
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
baxterP.setGripperHoldForce('left',100);
baxterP.setGripperMoveForce('left',100);

% Initialize right arm params
linVel_R = [0;0;0];
angVel_R = [0;0;0];
wristVel_R = [];
grip_R = [];
baxterP.calibrateGripper('right');
pause(2);
baxterP.setGripperHoldForce('right',100);
baxterP.setGripperMoveForce('right',100);

% Initialize serial communication
try
    s1 = serial('/dev/ttyS100'); % Define serial port
catch err
    disp(['CHECK IF SERIAL PORT IS VALID:',err.message]);
end
set(s1, 'DataBits', 8);
set(s1, 'StopBits', 1);
s1.BaudRate=9600; % Define baud rate
set(s1, 'Parity', 'none');
try
    fopen(s1); 
catch err
    disp(['WAS s1 CLOSED BEFORE?:',err.message]);
end
pause(2);

% Starting pose
baxterJ.setControlMode(uint8(0)); % Position mode
baxterJ.setJointCommand('left',[-0.7854;-1.0472;0;2.0944;0;-1.0472;-0]);
baxterJ.setJointCommand('right',[0.7854;-1.0472;0;2.0944;0;-1.0472;-0]);
pause(5);
baxterJ.setControlMode(uint8(1)); % Velocity mode

%calibrateJamboxx(jamboxx);

clc; disp('READY...');

while(1) 
       
    % Gather Xbox input
    xboxInput = xbox.controller_input;
    if xboxInput.start_button
        xInput = (0*-1 + 10000)* 130 /20000 + 127; 
        yInput = (0 + 10000)* 130 /20000; 
        fwrite(s1,char(xInput),'char');
        fwrite(s1,char(yInput),'char');
        baxterJ.setControlMode(uint8(0));
        baxterJ.setJointCommand('left',[-0.7854;-1.0472;0;2.0944;0;-1.0472;-0]);
        baxterJ.setJointCommand('right',[0.7854;-1.0472;0;2.0944;0;-1.0472;-0]);
        pause(7);
        baxterJ.setControlMode(uint8(1));
    end

    % Set wheelchair velocity
    xInput = (xboxInput.left_thumbstick_X*-1 + 10000)* 130 /20000 + 127; 
    yInput = (xboxInput.left_thumbstick_Y + 10000)* 130 /20000; 
    xInput = limitVal(129,255,xInput); yInput = limitVal(0,127,yInput);
    fwrite(s1,char(xInput), 'char');
    fwrite(s1,char(yInput), 'char');  

    % Gather joint information
    jointAngles = baxterJ.joint_positions;
    jointAnglesLeft = jointAngles(1:7);
    jointAnglesRight = jointAngles(8:14);

    % Calculate full jacobian for both arms
    leftJ = jacobian(baxterConst.leftArm,jointAnglesLeft);
    rightJ = jacobian(baxterConst.rightArm,jointAnglesRight);
    
    % Set desired input using Jamboxx
    [linVel_L,linVel_R,wristVel_L,wristVel_R,grip_L,grip_R] = setDesiredInput(jamboxx); 
    
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
        qDot_L = [0;0;0;0;0;0;0];
        if any(allVel_L)
            qDot_L = leftJ'*pinv(leftJ*leftJ' + dampCoeff_L^2*eye(6,6))*allVel_L; %Damped least squares
        end
        if ~isempty(wristVel_L)
            qDot_L(5:7) = wristVel_L;
        end

        % Right arm
        dampCoeff_R = 0.1;
        qDot_R = [0;0;0;0;0;0;0];
        if any(allVel_R)
            qDot_R = rightJ'*pinv(rightJ*rightJ' + dampCoeff_R^2*eye(6,6))*allVel_R; %Damped least squares
        end   
        if ~isempty(wristVel_R)
            qDot_R(5:7) = wristVel_R;
        end 
    
    % Publish joint position
    baxterJ.setControlMode(uint8(1));
	baxterJ.setJointCommand('left',qDot_L);
	baxterJ.setJointCommand('right',qDot_R);
        
    % Publish grip position
        % Left gripper
        if ~isempty(grip_L)
            baxterP.setGripperPosition('left',grip_L);
        end
        %Right gripper
        if ~isempty(grip_R)
            baxterP.setGripperPosition('right',grip_R);
        end

   pause(0.1);

end

clc; msgbox('Program stopped!');
