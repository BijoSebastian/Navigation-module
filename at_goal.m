function [ flag ] = at_goal()
%Event which checks if we have reached the goal (within threshold)

distThresh = 50.0; 

global goal;
xd = goal(1,1);
yd = goal(2,1);

% get current robot position
global state;
xa = state(1);
ya = state(2);

% Check if we've reached goal point
d = sqrt( (xd - xa)^2 + (yd - ya)^2 );
%fprintf(' distance remaining: %f\n', d);

if d <= distThresh
    disp 'Reached goal point';
    flag  = 1;
    %disp('At goal event flaged');
else 
    flag  = 0;
end

end

