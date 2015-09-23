function [ flag ] = obstacle_cleared()
%Event which checks if we have caleared all obstacle (within safe distance)
  
d_safe = 500.0;
flag = 1;
ultra_sensor_val = get_sonar_range();

  for i = 1:8
  
    r = ultra_sensor_val(i);
    if(r < d_safe)
        flag = 0;
    end
    
  end
 
 if(flag == 1)
     %disp('obstacle cleared event flagged');
 end
 
end

