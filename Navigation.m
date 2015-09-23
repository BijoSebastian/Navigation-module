%May your creation have only good things to say about its creator.
                                                            %R. Seigwart
%Eggertsed's low level navigation system along with EKF-SLAM and A* planner
%PPV: 1. Shutdown all firewalls and virus protection thingies.
%     2. Make sure the simulation is already running in VREP before
%       you start the code.
%     3. All distances in millimeters and angles in radiens.

clc;
clear all;
close all;

%Global variables
global vrep; %for VREP
global clientID;

global ultra_sensor_h; %Handles
global left_motor_h;
global right_motor_h;
global left_wheel_h;
global right_wheel_h;

global wheel_dia; %Robot params
global mm_per_tick;
global ticks_per_rev;
global robot_width;

global leftMotorPreviousAngle; %Encoder
global rightMotorPreviousAngle;
global leftRotation;
global rightRotation;
global prev_ltick;
global prev_rtick;

global state; %EKF variables 
global covariance;
global num_lndmrks;
global provisional;%provisinal landmark list
global map;%Line segmant map

global destination;

global goal;%Low level controller
global dprogress; %in mm
global flag_change;
global fire; %Whether or not to run

global fig_global; %Figure handles
global fig_laser;
global fig_lowlevel;

%Initialisations
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

%All in millimeters
wheel_dia = 195.0;
ticks_per_rev = 500;
mm_per_tick = (wheel_dia*pi)/ticks_per_rev;
robot_width = 365.0; 

if (clientID>-1)
    disp('Connected to remote API server');
    
    %Obtaining all desired handles
    
    %Retrieve Ultrasonic sensor arrays and initiate sensors
    ultra_sensor_val = []; %empty array for sensor measurements
    ultra_sensor_h = [];%empty array for sensor handles
    for i =1:16
        [~, sensor_handle] = vrep.simxGetObjectHandle(clientID,strcat('Pioneer_p3dx_ultrasonicSensor',num2str(i)),vrep.simx_opmode_oneshot_wait); 
        ultra_sensor_h = [ultra_sensor_h, sensor_handle]; %keep list of handles        
        
        [~, ~, detectedPoint, ~, ~] = vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming);
        ultra_sensor_val = [ultra_sensor_val,norm(detectedPoint)]; %get list of values
    end    
    
    %Retrieve motors.
    [~, left_motor_h] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait); 
    [~, right_motor_h] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait);                 
    vrep.simxSetJointTargetVelocity(clientID, left_motor_h, 0.0, vrep.simx_opmode_oneshot);    
    vrep.simxSetJointTargetVelocity(clientID, right_motor_h, 0.0, vrep.simx_opmode_oneshot);  

    %Initiating encoders
    [~, left_wheel_h] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftWheel',vrep.simx_opmode_oneshot_wait); 
    [~, right_wheel_h] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightWheel',vrep.simx_opmode_oneshot_wait);                 
    [~, ~] = vrep.simxGetObjectOrientation(clientID, left_wheel_h, -1, vrep.simx_opmode_streaming);
    [~, ~] = vrep.simxGetObjectOrientation(clientID, right_wheel_h, -1, vrep.simx_opmode_streaming);
    [~, angle] = vrep.simxGetObjectOrientation(clientID, left_wheel_h, -1, vrep.simx_opmode_buffer);
    leftMotorPreviousAngle = angle(3);
    [~, angle] = vrep.simxGetObjectOrientation(clientID, right_wheel_h, -1, vrep.simx_opmode_buffer);
    rightMotorPreviousAngle = angle(3);
    
    leftRotation = 0.0;
    rightRotation = 0.0;
    prev_ltick = 0.0;
    prev_rtick = 0.0;
    
    %Initiating LASER
    [~,~] = vrep.simxGetStringSignal(clientID,'measuredDataAtThisTime',vrep.simx_opmode_streaming);
    
    vrep.simxAddStatusbarMessage(clientID,'Begin Simulation',vrep.simx_opmode_oneshot);     
    
    %%%%%%%%%%%%%% for validation purposes only    
    [~, pioneer_h] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait); 
    [~, position] = vrep.simxGetObjectPosition(clientID, pioneer_h, -1, vrep.simx_opmode_streaming);
    [~, orientation] = vrep.simxGetObjectOrientation(clientID, pioneer_h, -1, vrep.simx_opmode_streaming);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
else
    disp('Failed connecting to remote API server');
    vrep.delete(); % call the destructor! 
end

%Initializing EKF
%Initial state.
state = [0.0, 0.0, 0.0];
%Initial covariance.
covariance = zeros(3);
%The number of landmarks observed.
num_lndmrks = 0;
provisional = [];
map = [];

%Initialize all figures
fig_laser = figure('name', 'Laser scanner.','Position',[10000,10000,400,300]);
axis equal;

fig_lowlevel = figure('name','Low level controller.', 'Position',[10000,10000,400,300]);
axis([-3000.0 3000.0 -6000.0 5000.0]);

fig_global = figure('name', 'Global Map.');
axis([-3000.0 3000.0 -1000.0 6000.0]);
hold on;

%Define destination 
destination = [2000.0; 3300.0];
%Display destination 
scatter(destination(1),destination(2),100.0,'filled','green');
drawnow;

%Get some initial observations for landmarks before you start moving.
for wait = 1:15    
    [~, position] = vrep.simxGetObjectPosition(clientID, pioneer_h, -1, vrep.simx_opmode_buffer);
    [~, orientation] = vrep.simxGetObjectOrientation(clientID, pioneer_h, -1, vrep.simx_opmode_buffer);
    state(1) = position(1)*1000.0;%in mm
    state(2) = position(2)*1000.0;%in mm
    state(3) = orientation(3);    
    EKF_Prediction();
    EKF_Correction();
end

%Start navigation module
while ~at_destinaton()
    disp('Running planner')
    path = AStarfunc();
    
    %Display path
    figure(fig_global);
    axis([-3000.0 3000.0 -3000.0 6000.0]);
    hold on;
    len_h = 1;
    for j = 1:size(path,1)-1
        h_global(len_h) = plot([path(j,1), path(j+1,1)], [path(j,2), path(j+1,2)], 'color', 'blue');
        len_h = len_h + 1;
        h_global(len_h) = scatter(path(j,1), path(j,2), 50, 'filled', 'blue'); 
        len_h = len_h + 1;
    end    
    drawnow;    
    
    %Set local goal
    goal = path(2,:)';   
    figure(fig_lowlevel);
    axis([-3000.0 3000.0 -6000.0 5000.0]);
    hold on;
    h_local = scatter(goal(1),goal(2),100,'filled','green');        
    %Begin the low level controller loop    
    %The prograss variable initialisation
    dprogress = -1;
    
    %It all begins at the Go to goal controller (gtg).
    guard(1)
    flag_change = true;
    fire = 1;
        
    while fire %Till there burns the fire    
        [~, position] = vrep.simxGetObjectPosition(clientID, pioneer_h, -1, vrep.simx_opmode_buffer);
        [~, orientation] = vrep.simxGetObjectOrientation(clientID, pioneer_h, -1, vrep.simx_opmode_buffer);
        state(1) = position(1)*1000.0;%in mm
        state(2) = position(2)*1000.0;%in mm
        state(3) = orientation(3); %in rad
        %fprintf('vrep %d %d  %d \n', state(1), state(2), state(3));   
        EKF_Prediction();
        EKF_Correction();
        
        if fire == 1
            gtg();
            continue
        elseif fire == 2
            ao_gtg();
            continue         
        elseif fire == 3
            ao();    
            continue    
        elseif fire == 41
            fw('l');
            continue               
        elseif fire == 42
            fw('r');
            continue
        else
            stop_robot();
        end
        
    end
        
    delete(h_global(:));
    clear h_global;
    delete(h_local);
    clc   
end



    
    
    
    