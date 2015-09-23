function [ stat ] = isdestination( P1, P2 )
%Function to check if destination is reached.
% global robot_width;
% 
% dist = sqrt((P2(1) - P1(1))^2 + (P2(2) - P1(2))^2);
% if (dist < (sqrt(2)*robot_width))
%     stat = 1;
% else
%     stat = 0;
if (P1 == P2)
    stat = 1;
else
    stat = 0;
end