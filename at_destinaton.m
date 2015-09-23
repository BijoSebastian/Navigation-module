function [ flag ] = at_destinaton()
%Event which checks if we have reached the destination (within threshold)
%and completely ends the program.

global vrep;
distThresh = 300.0; %more than the threshold distance for 
                    %at_goal for local controller

global destination;
xd = destination(1,1);
yd = destination(2,1);

% get current robot position
global state;
xa = state(1);
ya = state(2);

% Check if we've reached destination
d = sqrt( (xd - xa)^2 + (yd - ya)^2 );
%fprintf(' distance remaining: %f\n', d);

if d <= distThresh
    disp 'Reached Destination';
    flag  = 1;
    vrep.delete(); % call the destructor! 
    disp('Program ended');
    disp('System exit')
else 
    flag  = 0;
end

end

