function [ ] = EKF_Correction( )
%Function to perform the corection step of EKF
    
    global state;
    global map;
    global covariance;
    global provisional;
    global num_lndmrks; 
    global fig_laser;
    global fig_global;

    figure(fig_laser)
    axis equal;
    hold on;
    
    %EKF correction constants
    mea_dist_stddev = 200.0;%5000
    mea_ang_stddev = (15.0/180.0)*pi; %15
    d_thresh_match = 800;
    sigmax_nl = 1000;
    sigmay_nl = 1000;
    
    %Get Scan data
    [theta,rho,h_laser(1)] = get_laser();
    
    %Extract lines
    [R, Alpha, Ls] = scanner(theta, rho); 
    [Xs,Ys] = pol2cart(Alpha, R);
    h_laser(2) = scatter(Xs,Ys, 50,'filled','red');
    %display the detected lines 
    len_hs = 2; 
    %display the detected lines
    for j = 1:size(R)                                               
        len_hs = len_hs + 1;               
        h_laser(len_hs) = plot(Ls(j,1:2), Ls(j,3:4), 'color', 'green');
        len_hs = len_hs + 1;               
        h_laser(len_hs) = scatter(Ls(j,1:2), Ls(j,3:4), 30, 'filled', 'green');
    end
    
    %Correction step
    x = state(1);
    y = state(2);
    theta = state(3);
    for j = 1:size(Xs)  
        if(num_lndmrks ~= 0)          
            Walls = state(4:end);
            r_w = Walls(1:2:end)';
            alpha_w = Walls(2:2:end)';
            
            %Measurement predicition
            alpha  = alpha_w-theta;
            r = r_w - (x*cos(alpha_w) +  y*sin(alpha_w));            
            
            %Matching the predicitions with scans. 
            [Xp,Yp] = pol2cart(alpha, r);            
            dx = Xs(j) - Xp;
            dy = Ys(j) - Yp;
            dist_match = sqrt((dx.^2) + (dy.^2));
            [small,ind] = min(dist_match);
            if(small < d_thresh_match)
                [alpha_p, r_p] = cart2pol(Xp(ind),Yp(ind));
                [alpha_s, r_s] = cart2pol(Xs(j),Ys(j));
                len_hs = len_hs + 1;
                h_laser(len_hs) = plot([Xp(ind),Xs(j)],[Yp(ind),Ys(j)],'color', 'black', 'linewidth', 2);
                
                len_hs = len_hs + 1;
                h_laser(len_hs) = scatter(Xp(ind),Yp(ind), 30,'filled','black');                
                
                %Applying corrections.
                H = [-cos(alpha_w(ind)), -sin(alpha_w(ind)), 0.0;
                    0.0, 0.0, -1.0];                    
                %Modify H based on numer of landmarks               
                temp = [1.0, x*sin(alpha_w(ind))+y*cos(alpha_w(ind));
                        0.0, 1.0];
                H = [H, zeros(2,2*(ind - 1)), temp, zeros(2,2*(num_lndmrks - ind))];
                
                %Measurement covariance
                Q = diag([mea_dist_stddev^2, mea_ang_stddev^2]);
                
                %Kalman Gain
                temp = (H*covariance*H') + Q;
                K = (covariance*H')/temp;
                
                %Correcting state
                diff = [r_s - r_p; mod((alpha_s - alpha_p) + pi,2*pi) - pi];%Just to be sure.                                
                state = (state' + K*diff)';
                
                %Correcting covariance                
                covariance = (eye(3+2*num_lndmrks) - K*H)*covariance;
                
                %Apply correction to map(the lines may be getting longer)
                %Transform to world coordinate system.
                p1 = [Ls(j,1);Ls(j,3)];
                p2 = [Ls(j,2);Ls(j,4)];             
                %Rotate
                R = [cos(theta) -sin(theta);
                     sin(theta)  cos(theta)];
                p1 = R*p1;
                p2 = R*p2;                  
                %Translate
                p1 = p1 + [x;y];
                p2 = p2 + [x;y];
                Xm = [p1(1), p2(1)];
                Ym = [p1(2), p2(2)];                                    
                
                %check to see if length increased
                Xmo = map(ind,1:2);
                Ymo = map(ind,3:4);
                old = [Xmo(2) - Xmo(1), Ymo(2) - Ymo(1)];
                new = [Xmo(2) - Xm(1), Ymo(2) - Ym(1)];
                if(norm(old) > norm(new))
                    Xm(1) = Xmo(1);
                    Ym(1) = Ymo(1);
                end
                old = [Xmo(2) - Xmo(1), Ymo(2) - Ymo(1)];
                new = [Xm(2) - Xmo(1), Ym(2) - Ymo(1)];
                if(norm(old) > norm(new))
                    Xm(2) = Xmo(2);
                    Ym(2) = Ymo(2);
                end
                
                %A slight corrction
                for k = 1:2
                    Xm(k) = (Xm(k)*r_w(ind))/(Xm(k)*cos(alpha_w(ind)) + Ym(k)*sin(alpha_w(ind)));
                    Ym(k) = (Ym(k)*r_w(ind))/(Xm(k)*cos(alpha_w(ind)) + Ym(k)*sin(alpha_w(ind)));
                end
                map(ind,:) = [Xm,Ym];
                
                continue;
            end
        end
        
        %Check if newely observed landmark is in provisional list
        p_rw = provisional(1:3:end)';
        p_alphaw = provisional(2:3:end)';
        
        %Measurement predicition
        p_r = p_rw - (x*cos(p_alphaw) +  y*sin(p_alphaw));
        p_alpha  = p_alphaw-theta;
         
        %Matching the predicitions with scans. 
        [p_x, p_y] = pol2cart(p_alpha, p_r);
        dx = Xs(j) - p_x;
        dy = Ys(j) - p_y;
        dist_match = sqrt((dx.^2) + (dy.^2));
        [small,ind] = min(dist_match);
        if(small < d_thresh_match)
            provisional(3*ind) = provisional(3*ind) + 1;
            
            if provisional(3*ind) == 7
                %Add the newly observed landmark to state vector                
                %Transform to world coordinate system.
                [alpha_s, r_s] = cart2pol(Xs(j),Ys(j));
                alpha_sw = alpha_s + theta;
                r_sw = r_s + (x*cos(alpha_sw) +  y*sin(alpha_sw));
                
                %Update state
                state = [state , r_sw, alpha_sw];
                
                %Update covariance.
                [nr,nc] = size(covariance);
                temp = [sigmax_nl^2, 0.0;
                        0.0, sigmay_nl^2];
                covariance = [covariance, zeros(nr,2); zeros(2,nc), temp];
                num_lndmrks = num_lndmrks+1;
                
                
                %Add linesegment to map %Transform to world coordinate system.
                p1 = [Ls(j,1);Ls(j,3)];
                p2 = [Ls(j,2);Ls(j,4)];             
                %Rotate
                R = [cos(theta) -sin(theta);
                     sin(theta)  cos(theta)];
                p1 = R*p1;
                p2 = R*p2;                  
                %Translate
                p1 = p1 + [x;y];
                p2 = p2 + [x;y];
                Xm = [p1(1), p2(1)];
                Ym = [p1(2), p2(2)];               
                map = [map; [Xm,Ym]];                                
            end
        else
            %Add the newly observed landmark to provisional list:
            %Transform to world coordinate system.
            [alpha_s, r_s] = cart2pol(Xs(j),Ys(j));
            alpha_sw = alpha_s + theta;
            r_sw = r_s + (x*cos(alpha_sw) +  y*sin(alpha_sw));            
            provisional = [provisional, r_sw, alpha_sw, 0];
        end                         
    end

    %Following code is purely for display purposes    
    figure(fig_global);
    axis([-3000.0 3000.0 -3000.0 6000.0]);
    hold on;            

    %show actual robot
    Rx = state(1);
    Ry = state(2);
    h_global(1) = scatter(Rx,Ry,75,'filled','red');
    x = 500*cos(state(3));
    y = 500*sin(state(3));
    h_global(2) =  plot([state(1),state(1)+x],[state(2),state(2)+y],'color','black','linewidth',2);
    
    %Display landmarks;
    Walls = state(4:end);
    r_w = Walls(1:2:end)';
    alpha_w = Walls(2:2:end)';   
    [Xp,Yp] = pol2cart(alpha_w, r_w);
    h_global(3) = scatter(Xp, Yp, 20, 'filled', 'red'); 
    
    %Display map
    for j = 1:size(map,1)
        h_global(2 + j) = plot(map(j,1:2), map(j,3:4), 'color', 'black', 'linewidth', 3);
    end
    drawnow;
    
    global vrep; 
    global clientID;
    vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot);
    pause(0.15);
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
    delete(h_laser(:));
    delete(h_global(:));
end

