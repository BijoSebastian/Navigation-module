function [ flag ] = at_obstacle()
%Event which checks if we have detected any obstacle (within safe distance)
  
d_safe = 350.0;
flag = 0;
ultra_sensor_val = get_sonar_range();

  for i = 1:8
  
    r = ultra_sensor_val(i);
    if(r < d_safe)
        flag = 1;
        %disp('At obstacle event flaged');
        break
    end
    
  end
 
    
end

