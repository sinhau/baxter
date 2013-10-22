while(1)
      
    % Gather joint information
    i = 1;
    while(i)  
        if baxter_JointStates.InValueValid == 1 
            jointStates = baxter_JointStates.InValue;
            jointAnglesLeft = jointStates.position(10:16); % (See line 27 for the order)
            i=0;    
        end  
    end
    
    % Reorder jointAngle to be [left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2]
    joint34 = jointAnglesLeft(3:4);
    joint12 = jointAnglesLeft(1:2);
    jointAnglesLeft = [joint34;joint12;jointAnglesLeft(5:7)];
    
    clc;
    jointAnglesLeft
    pause(0.1)
end
