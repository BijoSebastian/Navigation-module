function [ flag ] = unsafe()
%Event which checks if we have detected any obstacle (within unsafe distance)
  
d_unsafe = 200.0;
flag = 0;
ultra_sensor_val = get_sonar_range();

  for i = 1:8
  
    r = ultra_sensor_val(i);
    if(r < d_unsafe)
        flag = 1;
        %disp('unsafe event flagged');
        break
    end
    
  end
 
  
end
