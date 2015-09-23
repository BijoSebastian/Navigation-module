function [ ] = ao_gtg( )
%The Avoid obstacle and Go to goal controller

global goal
global prev_heading_error
global total_heading_error 
global flag_change
    
if flag_change
    disp('Avoid obstacle and Go to goal controller engaged');
    flag_change = false;
end

%Setting constant velocity
velMult = 50.0; %mm/s

%Controller parameters
Kp = 0.01;
Kd = 0.0;
Ki = 0.0;

xd = goal(1,1);
yd = goal(2,1);

hao_gtg(1) = displayrobo();

%get current robot position
global state;
xa = state(1);
ya = state(2);
thetaa = rad2deg(state(3));

%Determine how far to rotate to face goal point
dt =  (atan2(yd - ya, xd - xa) * (180/3.14159)) - thetaa;
dtgtg = mod((dt + 180), 360) - 180; % restrict to (-180,180);
hao_gtg(2) = line([xa,xa+(1000*(cos(deg2rad(dtgtg + thetaa))))],[ya,ya+(1000*(sin(deg2rad(dtgtg + thetaa))))],'color','green');
drawnow;

%Get sonar readings
sonoffset = [-90 -50 -30 -10 10 30 50 90];
sonweight = [4 2 1 5 5 1 2 4];

r = zeros(8,1);
readings = zeros(2,8);
ultra_sensor_val = get_sonar_range();

for i = 1:8   
    r(i) = ultra_sensor_val(i);
    r(i) = r(i)*sonweight(i);
    angl = ((thetaa - sonoffset(i))/180)*pi;
    delx = r(i)*cos(angl);
    dely = r(i)*sin(angl);
    readings(1,i) = delx;
    readings(2,i) = dely;
end

readings = mean(readings,2);

%Determine how far to rotate to avoid obstacle
dt =  (atan2(readings(2,1),readings(1,1)) * (180/3.14159)) - thetaa;
dtao = mod((dt + 180), 360) - 180; % restrict to (-180,180);
hao_gtg(3) = line([xa,xa+(1000*(cos(deg2rad(dtao + thetaa))))],[ya,ya+(1000*(sin(deg2rad(dtao + thetaa))))],'color','red');

%Combining both in 1:3 ratio 
dtnet = (dtgtg + 3*dtao)/4.0;
%fprintf(' angle remaining: %f\n', dtnet);

hao_gtg(4) = line([xa,xa+(1000*(cos(deg2rad(dtnet + thetaa))))],[ya,ya+(1000*(sin(deg2rad(dtnet + thetaa))))],'color','blue');
drawnow;

%request robot to turn
W = (Kp*dtnet) + (Ki*total_heading_error) + (Kd*(dtnet - prev_heading_error));
set_motors(velMult,W);
total_heading_error = total_heading_error + dtnet;
prev_heading_error = dtnet;

delete(hao_gtg(:));

%Check events
if(at_goal())
    %call stop_robot();
    guard(5);
elseif(unsafe())
    %call ao();
    guard(3);
elseif(no_progress())
    if(chk_wall('l'))
        %call fw('l');
        guard(41);
    else
        %call fw('r');
        guard(42);
    end
elseif(obstacle_cleared())
    %call gtg();
    guard(1);
end

end


