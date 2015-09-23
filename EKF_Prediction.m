function [ ] = EKF_Prediction( )
%Function to perform the prediction step of EKF
    
    global state; 
    global covariance;
    global num_lndmrks;      
    global mm_per_tick;
    global robot_width;
    global prev_ltick;
    global prev_rtick;
    
    %Kalman Filter prediction constants
    cntrl_mf = 0.0035;%0.35;
    cntrl_tf = 0.006;%0.6;
    
    %Initialize
    x = state(1);
    y = state(2);
    theta = state(3);
        
    %Ticks to mm
    [l_tick, r_tick] = get_encoder();
    l = mm_per_tick*(l_tick - prev_ltick);    
    r = mm_per_tick*(r_tick - prev_rtick);    
    prev_rtick = r_tick;
    prev_ltick = l_tick;
    
    %Epsilon Control 
    sigma2_l = (cntrl_mf*l)^2 + (cntrl_tf*(r-l))^2;
    sigma2_r = (cntrl_mf*r)^2 + (cntrl_tf*(r-l))^2;
    eps_cont = diag([sigma2_l, sigma2_r]);
    
    %V matrix
    V = [cos(theta)/2, cos(theta)/2;
         sin(theta)/2, sin(theta)/2;
         -1/robot_width, 1/robot_width];     
    %Modify V based on numer of landmarks
    V = [V; zeros(2*num_lndmrks, 2)];
    
    d_c = (l + r)/2.0;
    
    %G matrix
    G = [1, 0, -d_c*sin(theta);
         0, 1, d_c*cos(theta);
         0, 0, 1];
    %Modify G based on numer of landmarks
    G = [G,zeros(3,2*num_lndmrks);
         zeros(2*num_lndmrks,3),eye(2*num_lndmrks)];
    
    %Compute covariance
    covariance = G*covariance*G' + V*eps_cont*V';
    
%     %Compute state
%     alpha = (r - l)/robot_width;
%     
%     x = x + d_c*cos(theta);
%     y = y + d_c*sin(theta);
%     theta  = mod((theta + alpha),2*pi);   
%     fprintf('prediction %d %d  %d \n', x, y, theta);
% There is no encoder implimentation in V-REP
% The encoder which I simulated throught code is not working fine either
% So use V-REP reported location for prediction 
% Later uncomment above codes for proper prediction.

end

