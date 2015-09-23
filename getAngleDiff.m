function [ diff ] = getAngleDiff(a1, a2)
%Auxilliary function for encoder

if (a1 >= 0 && a2 >= 0) 
    diff = a1 - a2;
elseif (a1 < 0 && a2 >=0)
    diff =  a2 - (a1 * -1);    
elseif (a1 < 0 && a2 < 0)
    diff =  a1 - a2;
elseif (a1 >= 0 && a2 < 0) 
    diff = a1 - (a2 * -1);    
else
    diff =  0.0;
end 

end

