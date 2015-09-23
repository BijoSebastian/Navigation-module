function [] = guard( num )
%The Guard Function
%1.sets fire to desired controller
%2.resets the controller errors Detailed explanation goes here

%Controller errors
global prev_heading_error;
global total_heading_error;
global fire;
global flag_change;
   
prev_heading_error = 0.0;
total_heading_error = 0.0;
    
fire = num;
flag_change = true;


end

