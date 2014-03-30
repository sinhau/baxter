function [linVel_L,linVel_R,wristVel_L,wristVel_R,grip_L,grip_R,rightSide] = setDesiredInputXbox(xbox,rightSide,baxter)
% Sets desired linear velocity, wrist joint velocities, and grip positions
% using jamboxx

    input = xbox.controller_input;
    if input.A
        rightSide = ~rightSide;
    end
    
    linVel_L = [0;0;0];
    linVel_R = [0;0;0];
    wristVel_L = [];
    wristVel_R = [];
    grip_L = [];
    grip_R = [];

    x = tic;
    y = 0;
    if input.start_button
        while y < 5
            baxter.setJointPosition('left',[-0.8544;-0.8805;0.1511;1.9934;-0.0418;-1.1789;-0.1457]);
            baxter.setJointPosition('right',[0.6105;-1.0761;0.1806;2.2415;-0.0065;-1.1340;-1.7675]);
            y = toc(x);
        end
    elseif rightSide
        linVel_R = [double(input.right_thumbstick_Y)/10000*0.1;-double(input.right_thumbstick_X)/10000*0.1;double(input.right_trigger)/10000*0.1];
        if input.left_trigger > 50
            linVel_R(3) = -double(input.left_trigger)/10000*0.1;
        end
        if input.left_arrow_right
            wristVel_R = [0.4;0;0];
        elseif input.left_arrow_left
            wristVel_R = [-0.4;0;0];
        else

        end
        if input.left_arrow_down
            wristVel_R = [0;0.4;0];
        elseif input.left_arrow_up
            wristVel_R = [0;-0.4;0];
        else

        end
        if input.B
            wristVel_R = [0;0;0.6];
        elseif input.X
            wristVel_R = [0;0;-0.6];
        else

        end
        if input.right_button
            grip_R = [0];
        elseif input.left_button
            grip_R = [100];
        else

        end
    else
        linVel_L = [double(input.right_thumbstick_Y)/10000*0.1;-double(input.right_thumbstick_X)/10000*0.1;double(input.right_trigger)/10000*0.1];
        if input.left_trigger > 50
            linVel_L(3) = -double(input.left_trigger)/10000*0.1;
        end
        if input.left_arrow_right
            wristVel_L = [0.4;0;0];
        elseif input.left_arrow_left
            wristVel_L = [-0.4;0;0];
        else

        end
        if input.left_arrow_down
            wristVel_L = [0;-0.4;0];
        elseif input.left_arrow_up
            wristVel_L = [0;0.4;0];
        else

        end
        if input.B
            wristVel_L = [0;0;0.6];
        elseif input.X
            wristVel_L = [0;0;-0.6];
        else

        end
        if input.right_button
            grip_L = [0];
        elseif input.left_button
            grip_L = [100];
        else

        end
    end
        
end