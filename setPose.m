function setPose(baxter,pose,baxterConst)
% Drives Baxter to the desired initial pose (only for left arm for now)
% Pose 1: Joint angles are [-0.6147;-0.6220;0.0736;1.9800;0.2428;-1.4596;-0.0859]
% Pose 2: Joint angles are [-0.6197;-1.1735;0.0617;2.0011;-0.0035;0.7152;-0.0280]

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

    % baxter_msgs/JointCommandMode
    leftMode.mode = uint8(2);   % 1 for position mode; 2 is for velocity mode; 3 for torque mode

    % baxter_msgs/JointVelocities
    leftJointVel.names = {{int32(0),'left_e0'};{int32(1),'left_e1'};{int32(2),'left_s0'};{int32(3),'left_s1'};{int32(4),'left_w0'};{int32(5),'left_w1'};{int32(6),'left_w2'}};
    leftJointVel.velocities = [0;0;0;0;0;0;0];
    
    err = ones(7,1);
    errTol = 0.08*ones(7,1);
    kP = 1;
    kD = 0.5;
    switch pose
        case 1
            while any(err > errTol)
                [jointAnglesLeft,~] = getJointAngles(baxter_JointStates);
                desJointAngles = [-0.6147;-0.6220;0.0736;1.9800;0.2428;-1.4596;-0.0859];
                errPrev = err;
                err = desJointAngles - jointAnglesLeft;
                for i = 1:length(err)
                    qDot(i) = kP*err(i) + kD*((err(i)-errPrev(i))/2);
                end
                % Limit joint angle velocity
                for k = 1:length(qDot)
                    if abs(qDot(k)) > baxterConst.jointVelLimit(k)
                        qDot(k) = sign(qDot(k))*baxterConst.jointVelLimit(k);
                    end
                end
                % Publish velocity
                leftJointVel.velocities = [qDot(3);qDot(4);qDot(1);qDot(2);qDot(5);qDot(6);qDot(7)];
                publisher_leftMode.publish(leftMode);
                publisher_leftJointVel.publish(leftJointVel);
            end          
        case 2
            while any(err > errTol)
                [jointAnglesLeft,~] = getJointAngles(baxter_JointStates);
                desJointAngles = [-0.6197;-1.1735;0.0617;2.0011;-0.0035;0.7152;-0.0280];
                errPrev = err;
                err = desJointAngles - jointAnglesLeft;
                for i = 1:length(err)
                    qDot(i) = kP*err(i) + kD*(err(i)-errPrev/2);
                end
                % Limit joint angle velocity
                for k = 1:length(qDot)
                    if abs(qDot(k)) > baxterConst.jointVelLimit(k)
                        qDot(k) = sign(qDot(k))*baxterConst.jointVelLimit(k);
                    end
                end
                % Publish velocity
                leftJointVel.velocities = [qDot(3);qDot(4);qDot(1);qDot(2);qDot(5);qDot(6);qDot(7)];
                publisher_leftMode.publish(leftMode);
                publisher_leftJointVel.publish(leftJointVel);
            end
        otherwise
            clc;
            disp('INVALID POSE CHOICE!');
    end
    
    leftJointVel.velocities = [0;0;0;0;0;0;0];
    publisher_leftMode.publish(leftMode);
    publisher_leftJointVel.publish(leftJointVel);
   
end