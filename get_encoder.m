function [l_tick, r_tick] = get_encoder()
%Function which returns encoder movement in ticks
    
    global vrep; 
    global clientID;
    global left_wheel_h;
    global right_wheel_h;
    global leftMotorPreviousAngle;
    global rightMotorPreviousAngle;
    global leftRotation;
    global rightRotation;
    global ticks_per_rev;

    %Determine how much radians per each pulse need to walk
    RPP = (2*pi) / ticks_per_rev;

    [~, anglel] = vrep.simxGetObjectOrientation(clientID, left_wheel_h, -1, vrep.simx_opmode_buffer);
    leftGama = anglel(3);
    leftRotation = leftRotation + getAngleDiff(leftGama, leftMotorPreviousAngle);
    leftMotorPreviousAngle = leftGama;
    l_tick = floor(leftRotation / RPP);
    
    [~, angler] = vrep.simxGetObjectOrientation(clientID, right_wheel_h, -1, vrep.simx_opmode_buffer);
    rightGama = angler(3);
    rightRotation = rightRotation + getAngleDiff(rightGama, rightMotorPreviousAngle);
    rightMotorPreviousAngle = rightGama;
    r_tick = floor(rightRotation / RPP);
    
       
end


