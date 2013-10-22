clear all;close all;clc;

%% Robot Raconteur Connections

% Connect to XBox 360 controller
% xbox = RobotRaconteur.Connect('tcp://192.168.1.106:5437/{0}/xbox_controller');
% xboxIn = xbox.controller_input;

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

% Set up inverse kinematics service client
handle_baxterIK = baxter.client('/sdk/robot/limb/left/solve_ik_position', 'baxter_msgs/SolvePositionIK');
client_baxterIK = baxter.get_clients(handle_baxterIK);

%% Initial parameter setup

% baxter_msgs/JointCommandMode
leftMode.mode = uint8(2);   % 1 for position mode; 2 is for velocity mode; 3 for torque mode

% baxter_msgs/JointVelocities
leftJointVel.names = {{int32(0),'left_e0'};{int32(1),'left_e1'};{int32(2),'left_s0'};{int32(3),'left_s1'};{int32(4),'left_w0'};{int32(5),'left_w1'};{int32(6),'left_w2'}};
leftJointVel.velocities = [0;0;0;0;0;0;0];

% baxter_msgs/SolvePositionIK
headerDef.seq = uint32(1);
stampDef.sec = 1;
stampDef.nsec = 1;
headerDef.stamp = stampDef;
headerDef.frame_id = {{int32(0),'base'}};
positionDef.x = 0.4575;
positionDef.y = 0.6519;
positionDef.z = 0.2388;
orientationDef.x = -0.3668;
orientationDef.y = 0.8859;
orientationDef.z = 0.1081;
orientationDef.w = 0.2621;
poseDef.position = positionDef;
poseDef.orientation = orientationDef;
pose_stampDef.header = headerDef;
pose_stampDef.pose = poseDef;
baxterPose.pose_stamp = pose_stampDef;

%% Main

%calibrateJamboxx(jamboxx);

while(1)
    
    % Gather joint information
    i = 1;
    while(i)  
        if baxter_JointStates.InValueValid == 1 
            jointStates = baxter_JointStates.InValue;
            jointAnglePosition = jointStates.position(3:9);
            i=0;    
        end  
    end
%     while(1)
%         xboxIn
%         pause(0.1);
%         clc;
%     end
    
    ikResp = client_baxterIK.call(baxterPose) 

end

RobotRaconteur.Disconnect(baxter);