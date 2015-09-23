function [stat] = clsnchk( P1,P2 )
%Function to check wall collisions
%Considers the robot to be rectangle box having twice the robot width  

global map
global robot_width;

P1 = P1.*robot_width;
P2 = P2.*robot_width;
temp = P2 - P1;
temp = temp/norm(temp);
P1 = P1 -  temp.*(robot_width);
P2 = P2 +  temp.*(robot_width);
A1 = P1 + ([temp(2),-temp(1)].*(robot_width));
B1 = P2 + ([temp(2),-temp(1)].*(robot_width));
A2 = P1 + ([-temp(2),temp(1)].*(robot_width));
B2 = P2 + ([-temp(2),temp(1)].*(robot_width));
xp = [A1(1),B1(1)];
yp = [A1(2),B1(2)];
xm = [];
ym = [];
stat = 0;

for j = 1:size(map,1)
    xm = map(j,1:2);
    ym = map(j,3:4);
    [xi, yi] = polyxpoly(xp, yp, xm, ym);
    if size(xi) ~= 0
        stat =  1;   
        return
    end
end

xp = [A2(1),B2(1)];
yp = [A2(2),B2(2)];
xm = [];
ym = [];
for j = 1:size(map,1)
    xm = map(j,1:2);
    ym = map(j,3:4);
    [xi, yi] = polyxpoly(xp, yp, xm, ym);
    if size(xi) ~= 0
        stat =  1;
        return
    end
end

xp = [A1(1),A2(1)];
yp = [A1(2),A2(2)];
xm = [];
ym = [];
for j = 1:size(map,1)
    xm = map(j,1:2);
    ym = map(j,3:4);
    [xi, yi] = polyxpoly(xp, yp, xm, ym);
    if size(xi) ~= 0
        stat =  1; 
        return
    end
end

xp = [B1(1),B2(1)];
yp = [B1(2),B2(2)];
xm = [];
ym = [];
for j = 1:size(map,1)
    xm = map(j,1:2);
    ym = map(j,3:4);
    [xi, yi] = polyxpoly(xp, yp, xm, ym);
    if size(xi) ~= 0
        stat =  1;   
        return
    end
end








