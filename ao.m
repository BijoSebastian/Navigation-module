function [ ] = ao( )
%The Avoid obstacle controller

global prev_heading_error;
global total_heading_error; 
global flag_change;
    
if flag_change
    disp('Avoid obstacle controller engaged');
    flag_change = false;
end

%Setting constant velocity
velMult = 35.0; %mm/s

%Controller parameters
Kp = 0.005;
Kd = 0.0;
Ki = 0.0;

hao(1) = displayrobo();

%get current robot position
global state;
xa = state(1);
ya = state(2);
thetaa = rad2deg(state(3));

%Get sonar readings
sonoffset = [-90 -50 -30 -10 10 30 50 90];
sonweight = [4 2 1 5 5 1 2 4];

r = zeros(8,1);
readings = zeros(2,8);
ultra_sensor_val = get_sonar_range();

for i = 1:8
    
    r(i) = ultra_sensor_val(i);    
    if(r(i) < 200)
        r(i) = r(i)- 5000.0;
    end    
    r(i) = r(i)*sonweight(i);
    angl = ((thetaa - sonoffset(i))/180)*pi;
    delx = r(i)*cos(angl);
    dely = r(i)*sin(angl);
    readings(1,i) = delx;
    readings(2,i) = dely;
    hao(1+i) = line([xa,xa+delx],[ya,ya+dely],'color','red');
end
readings = mean(readings,2);
hao(10) = line([xa,xa+readings(1,1)],[ya,ya+readings(2,1)],'color','blue');
drawnow;

%Determine how far to rotate to avoid obstacle
dt =  (atan2(readings(2,1),readings(1,1)) * (180/3.14159)) - thetaa;
dt = mod((dt + 180), 360) - 180; % restrict to (-180,180);     
%fprintf(' angle remaining: %f\n', dt);

%request robot to turn
W = (Kp*dt) + (Ki*total_heading_error) + (Kd*(dt - prev_heading_error));
set_motors(velMult,W);
total_heading_error = total_heading_error + dt;
prev_heading_error = dt;

delete(hao(:));

%Check events
if(at_goal())
    %call stop_robot();
    guard(5);
elseif(obstacle_cleared())
    %call ao_gtg();
    guard(2);
end

end


