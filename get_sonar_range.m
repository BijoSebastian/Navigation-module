function [ultra_sensor_val] = get_sonar_range( )
%Function to get SONAR readings 
%Return all reading in array.
%Unit in millimeters

    global vrep; 
    global clientID;
    global ultra_sensor_h;     
    
    ultra_sensor_val = []; %empty array for new sensor measurements
        for i =1:16                                     
            [errorCode,detectionState,detectedPoint,~,~] = vrep.simxReadProximitySensor(clientID,ultra_sensor_h(i),vrep.simx_opmode_buffer);     
            if errorCode || ~detectionState  
                ultra_sensor_val = [ultra_sensor_val,5000.0];
            else            
                ultra_sensor_val = [ultra_sensor_val,norm(detectedPoint)*1000.0]; %get list of values     
            end
        end 

end

