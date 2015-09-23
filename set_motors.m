function [ ] = set_motors(V,W)
%Function to set the motor velocities
%The input is the linear speed V(in mm/s),
%and the angular speed W (in rad/s)
 
    global vrep;
    global clientID;
    global left_motor_h;
    global right_motor_h;
    global wheel_dia;
    global robot_width;

    %1. Limit v,w from controller to +/- of their max
    w = max(min(W, 0.785), -0.785); %0.785 rad/s
    v = max(min(V, 100.0), -100.0); %100.0 mm/s
            
    %2. Compute desired vel_r, vel_l needed to ensure w
    Vr = ((2.0*v) + (w*robot_width))/(wheel_dia);
    Vl = ((2.0*v) - (w*robot_width))/(wheel_dia);
                        
    %3. Find the max and min vel_r/vel_l
    vel_rl_max = max(Vr, Vl);
    vel_rl_min = min(Vr, Vl);
    
    % 4. Shift vel_r and vel_l if they exceed max/min vel
    vel_r = Vr;
    vel_l = Vl;
    if (vel_rl_max > 0.5) %1.5 rad/s
        vel_r = Vr - (vel_rl_max - 1.0);
        vel_l = Vl - (vel_rl_max - 1.0);
    end
    if (vel_rl_min < -0.5)
        vel_r = Vr - (vel_rl_min + 1.0);
        vel_l = Vl - (vel_rl_min + 1.0);
    end
       
%     %since the encoders are not quadrature wheel rotation
%     %in reverse could cause trouble with encoders.
%     if vel_r < 0
%         vel_r = 0.0;
%     end
%     if vel_l < 0
%         vel_l = 0.0;
%     end
% %     fprintf('%d %d \n', vel_r, vel_l)

    vrep.simxSetJointTargetVelocity(clientID, left_motor_h, vel_l, vrep.simx_opmode_oneshot_wait);    
    vrep.simxSetJointTargetVelocity(clientID, right_motor_h, vel_r, vrep.simx_opmode_oneshot_wait); 

end

