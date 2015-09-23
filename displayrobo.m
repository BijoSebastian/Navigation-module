function [ h ] = displayrobo(  )
%Function to display the robot 
% Returns the handle of printing the robot

global state;
global fig_lowlevel;

figure(fig_lowlevel);

% get current robot position
Rx = state(1);
Ry = state(2);
h = scatter(Rx,Ry,100,'filled','red');









