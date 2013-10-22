function setBaxterConstants

    x = [1;0;0]; y = [0;1;0]; z = [0;0;1];

    % NOTE:
    % - Origin is at the base of first joint for both arms
    % - Zero configuration is when the joint_states.position reads 0 for all
    %   joints.
    % - Before rotation, y-axis is in the direction the arm is stretched
    %   out in zero configuration
    % - Position and rotation vectors are rotated such that the x-axis for both arms
    %   point straight out in front of Baxter
    
    % Setting up left arm parameters
    leftArm.P = [[0;0;0],[0;0.1;0.235],[0;0.1;0],[0;0.24;-0.095],[0;0.09;0],[0;0.28;-0.02],[0;0.18;0],[0;0.17;0]];
    leftArm.H = [z,-x,y,-x,y,-x,y];
    leftArm.type = [0,0,0,0,0,0,0];
    leftArm.n = 7;
    baxterconst.leftArm = leftArm;
    
    % Setting up right arm parameters
    rightArm.P = rot(z,(pi/4)+(pi/2))*[[0;0;0],[0;0.1;0.235],[0;0.1;0],[0;0.24;-0.095],[0;0.09;0],[0;0.28;-0.02],[0;0.18;0],[0;0.17;0]];
    rightArm.H = rot(z,(pi/4)+(pi/2))*[z,-x,y,-x,y,-x,y];
    rightArm.type = [0,0,0,0,0,0,0];
    rightArm.n = 7;
    baxterconst.rightArm = rightArm;

    % Setting up joint angle limits
    baxterconst.jointLimitsLow = [-1.702, -2.147, -3.054, -1.621, -3.059, -1.571, -3.059, ...
                                -1.702, -2.147, -3.054, -1.621, -3.059, -1.571, -3.059]';
    baxterconst.jointLimitsHigh = [1.702, 1.047, 3.054, 1.047, 3.059, 2.094, 3.059, ...
                                1.702, 1.047, 3.054, 1.047, 3.059, 2.094, 3.059]';
    
    % Setting up joint velocity limits
    baxterconst.jointVelLimit=[2.5 2.5 2.5 2.5 5 5 5 ...
                                2.5 2.5 2.5 2.5 5 5 5]';

assignin('base','baxterConst',baxterconst);