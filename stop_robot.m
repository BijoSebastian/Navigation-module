function [  ] = stop_robot( )
%This function stops the robot

global fire;
global flag_change;
    
if flag_change
    disp('Robot stop controller engaged');
    set_motors(0,0)
    flag_change = false;
end

fire = 0;
end

