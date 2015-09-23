function [npath] = rtranslator( path )
%Function which converts path in grid coordinates back to world coordinates.

%Define the robot parameters
global robot_width;

global state; 
global destination;

npath = path.*robot_width;
npath(1,:) = [state(1),state(2)];
npath(end,:) = destination;
