function [gc] = translator( point )
%Function which converts world coordinates to grid coordinates.

%Define the robot parameters
global robot_width;

x = point(1);
y = point(2);

x = floor(x/(robot_width));
y = floor(y/(robot_width));

gc = [x,y];




