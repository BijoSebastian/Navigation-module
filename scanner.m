function [ R,Alpha,Ls ] = scanner( theta,rho )
%Function to extract lines out of scanner measurements.
    
    %Constants
    %The line splitting dist in split.
    d_thresh_split = 50;
    %The lines merging dist in merge
    d_thresh_merge = 100;
    %The number of points needede to be considered a valid line.
    num_point_thresh = 12;
    
    %Initialise
    L = {[rho',theta']};
    i = 1;
    R = [];
    Alpha = [];
    
    %Split
    while i <= size(L,2)
        temp = L{i};
        if(size(temp,1)<7)%3
            L = {L{1:i-1},L{i+1:end}};
            continue
        end
        rho = temp(:,1);
        theta = temp(:,2);
        [r,alpha] = fitline_polar(rho',theta');
        [X,Y] = pol2cart(theta,rho);
        diff = abs(X(2:end-1)*cos(alpha) + Y(2:end-1)*sin(alpha) - r);
        [D,I] = max(diff);
        I = I+2;
        if D > d_thresh_split
            L{i} = [rho(1:I-1),theta(1:I-1)];
            L{end+1} = [rho(I:end),theta(I:end)];
        else
            i = i+1;
            R = [R;r];
            Alpha = [Alpha;alpha];
        end
    end
    [X,Y] = pol2cart(Alpha,R);
    
    %Merge
    for i =1:size(X,1)-1
        j = i+1;
        while j <= size(X,1)
            dx = X(i) - X(j);
            dy = Y(i) - Y(j);        
            if(sqrt((dx^2) + (dy^2)) <  d_thresh_merge)
                X = [X(1:j-1);X(j+1:end)];
                Y = [Y(1:j-1);Y(j+1:end)];
                temp = L{i};
                temp = [temp;L{j}];
                L{i} = temp;
                L = {L{1:j-1},L{j+1:end}};          
            else
                j = j+1;
            end
        end
    end
    
    %Fit
    R = [];
    Alpha = [];
    Ls = [];
    for i = 1:size(L,2)
        temp = L{i};
        if(size(temp,1) >  num_point_thresh)
            rho = temp(:,1);
            theta = temp(:,2);
            [X,Y] = pol2cart([theta(1),theta(end)],[rho(1),rho(end)]);
            Ls = [Ls; [X,Y]];          
            [r,alpha] = fitline_polar(rho',theta');
            R = [R;r];
            Alpha = [Alpha;alpha];
        end
    end
end

