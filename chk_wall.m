function [ flag ] = chk_wall(side)
%Event to check whether to continue following the given side.

global goal;
xd = goal(1,1);
yd = goal(2,1);

%Define the robot parameters
global robot_width;

dfw = 2*robot_width;%Follow wall threshold in mm

%get current robot position
global state;
xa = state(1);
ya = state(2);
thetaa = rad2deg(state(3));

%Determine how far to rotate to face goal point
dt =  (atan2(yd - ya, xd - xa) * (180/3.14159)) - thetaa;
dtgtg = mod((dt + 180), 360) - 180; % restrict to (-180,180);

Ugtg = [cos(deg2rad(dtgtg + thetaa));sin(deg2rad(dtgtg + thetaa))];

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
Uao = [cos(deg2rad(dtao + thetaa));sin(deg2rad(dtao + thetaa))];


r = zeros(4,1);
readings = zeros(2,4);
ultra_sensor_val = get_sonar_range();  

if(side == 'l')
    for i = 1:4  
        r(i) = ultra_sensor_val(i);
        if(r(i) > 2000)
            r(i) = 2000;    
        end
        angl = ((thetaa - sonoffset(i))/180)*pi;
        delx = r(i)*cos(angl);
        dely = r(i)*sin(angl);
        readings(1,i) = delx;
        readings(2,i) = dely;
    end
else
    for i = 1:4
        r(i) = ultra_sensor_val(9-i);
        if(r(i) > 2000)
            r(i) = 2000;
        end
        angl = ((thetaa - sonoffset(9-i))/180)*pi;
        delx = r(i)*cos(angl);
        dely = r(i)*sin(angl);
        readings(1,i) = delx;
        readings(2,i) = dely;
    end
end
  
[~,Ix] = sort(r);

if(Ix(1) > Ix(2))
    temp = Ix(1);
    Ix(1) = Ix(2);
    Ix(2) = temp;
end
  
Ufw_t = readings(:,Ix(2)) - readings(:,Ix(1));

Upfw_t = Ufw_t/norm(Ufw_t);

temp = (dot(readings(:,Ix(1)),Upfw_t))*Upfw_t;
Ufw_p = readings(:,Ix(1)) - temp;
  
Upfw_p = Ufw_p - (dfw*(Ufw_p/norm(Ufw_p)));

Ufw  = ((Upfw_p) + (4*Upfw_t))/5;
Ufw = Ufw/norm(Ufw);

Sigma  = [Ugtg Uao]\Ufw;

if((Sigma(1,1) > 0) && (Sigma(2,1) > 0))
    flag = 1;
else
    flag = 0;
end

end