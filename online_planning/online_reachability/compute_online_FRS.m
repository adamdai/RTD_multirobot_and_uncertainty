% Compute forward stochastic reachable set of an n-dimensional LQG system 
% for a space of trajectory paramters
% 
% Leverages (slightly modified) linear reachability math from A. Shetty 2019
% 
% Inputs:
%   x_start - 
%   p_0   -
%   v_0   -
%   a_0   -
%   v_max -
%   sys   - struct containing system matrices A,B,and C, LQR gain K, and  
%           noise covariance matrices Q and R  
%   P0    - initial state covariance
%   N     - trajectory length
%   LPM   - linear planning model
%   coeff_hist - 
%   sigma_bound - sigma confidence bound for extracting bounding FRS
%
% Outputs:
%   pXrs - cell array of probZonotopes representing stochastic forward 
%          reachable sets in time
%   Xrs  - cell array of zonotopes representing deterministic forward 
%          reachable sets in time
%   

function [pXrs, Xrs, coeff_hist] = compute_online_FRS(x_start, p_0, v_0, a_0, v_max, sys, P0, LPM, coeff_hist, sigma_bound)
    
    % retrieve system matrices from sys struct
    A = sys.A; B = sys.B; C = sys.C;
    K = sys.K; Q = sys.Q; R = sys.R;
    
    % define trajectory parameter space
    S = zonotope([v_0(1) 0     0; 
                  a_0(1) 0     0; 
                  0      v_max 0;
                  v_0(2) 0     0; 
                  a_0(2) 0     0; 
                  0      0     v_max]);
    
    % parameters
    n = size(A,1); % system dimension
    m = dim(S); % trajectory parameter dimension
    
    coeff_thresh = 0;

    % trajectory length
    N = length(LPM.time);
    
    % nominal initial state
    x_0 = [p_0; v_0];
    
    % starting stochastic reachable set (at very beginning of trajectory,
    % not at start of this planning step, i.e. different from initial state)
    X_start = probZonotope(x_start,cov2probGen(P0),sigma_bound);
    
    % trajectory parameter to control input mapping
    a_mat = LPM.acceleration;
    M = cell(1,length(a_mat));
    for i = 1:N
        M{i} = [a_mat(:,i)' zeros(1,3); 
                zeros(1,3)  a_mat(:,i)'];
    end
    
    % initial state covariance
    P = P0; 
    P = P(1:n,1:n);
    
    % initial state estimation error
    WpZ = probZonotope(zeros(n,1),cov2probGen(Q),sigma_bound);
    VpZ = probZonotope(zeros(2,1),cov2probGen(R),sigma_bound);
    WpZ_c = center(WpZ); WpZ_g = generators(mean(WpZ)); WpZ_cov = sigma(WpZ);
    VpZ_c = center(VpZ); VpZ_g = generators(mean(VpZ)); VpZ_cov = sigma(VpZ);
    
    % initialize FRS
    pXrs = cell(1,N);
    Xrs = cell(1,N);
    pXrs{1} = [eye(n); zeros(m,n)] * x_0 + [zeros(n,m); zeros(m,m)] * S ...
              + [eye(n); zeros(m,n)]*(X_start + -x_start);
    umeanZ = mean(pXrs{1});
    covZ = cov2zonotope(sigma(pXrs{1}),sigma_bound,n);
    Xrs{1} = deleteZeros(umeanZ + covZ);
    
    % retrieve coeffs
    a = coeff_hist.a; b = coeff_hist.b;
    c = coeff_hist.c; d = coeff_hist.d;
    e = coeff_hist.e; 
    p = coeff_hist.p; q = coeff_hist.q;
    
    for t = 2:N
        
        % update coeffs a and b
        a = (A-B*K)*a;
        b = (A-B*K)*b - B*K*e;

        % update previous c and d coeffs 
        c = multiprod(A-B*K,c) - multiprod(B*K,p);
        d = multiprod(A-B*K,d) - multiprod(B*K,q);
        
        % append new c and d coeffs 
        c = cat(3,c,eye(n));  
        d = cat(3,d,zeros(n,2));

        % calculate all CpZ and DpZ terms
        all_CpZ_c = cat(1, sum(c,3)*WpZ_c, zeros(m,1));
        all_CpZ_g = cat(1, reshape(multiprod(c,WpZ_g),n,[]), zeros(m,size(WpZ_g,2)*size(c,3)));
        all_CpZ_cov = [sum(multiprod(multiprod(c,WpZ_cov),permute(c,[2 1 3])),3) zeros(n,m); zeros(m,n) zeros(m,m)];
        all_CpZ = probZonotope([all_CpZ_c all_CpZ_g], cov2probGen(all_CpZ_cov), sigma_bound);
        
        all_DpZ_c = cat(1, sum(d,3)*VpZ_c, zeros(m,1));
        all_DpZ_g = cat(1, reshape(multiprod(d,VpZ_g),n,[]), zeros(m,size(VpZ_g,2)*size(d,3)));
        all_DpZ_cov = [sum(multiprod(multiprod(d,VpZ_cov),permute(d,[2 1 3])),3) zeros(n,m); zeros(m,n) zeros(m,m)];
        all_DpZ = probZonotope([all_DpZ_c all_DpZ_g], cov2probGen(all_DpZ_cov), sigma_bound);

        % compute reachable set
        AB_coeff = 0;
        for i = 1:t-1
            AB_coeff = AB_coeff + A^(t-i) * B * M{i};
        end

        % reachability expression
        pXrs{t} = [A^t; zeros(m,n)] * x_0 + [AB_coeff; eye(m,m)] * S ... 
                  + [a - b; zeros(m,n)] * (X_start + -x_start) + all_CpZ + all_DpZ;
        
        % extract 3-sigma bound zonotope
        umeanZ = mean(pXrs{t});
        covZ = cov2zonotope(sigma(pXrs{t}),sigma_bound,n);
        Xrs{t} = deleteZeros(umeanZ + covZ);

        % online filter steps
        P_pred = A*P*A' + Q;
        L = P_pred*C'/(C*P_pred*C' + R);
        P = P_pred - L*C*P_pred;

        % update coeff e
        e = (eye(n) - L*C)*A*e;
        
        % update previous p and q coeffs 
        p = multiprod((eye(n)-L*C)*A,p);
        q = multiprod((eye(n)-L*C)*A,q);

        % append new p and q coeffs 
        p = cat(3,p,-(eye(n) - L*C));  
        q = cat(3,q,L);

    end
    
    % drop c and d coeffs below threshold to save computation and memory
    % since WpZ and VpZ are constant (for now), we can drop all c and d
    % coefficients below the threshold disregarding order
    c_check_idx = vecnorm(reshape(c,[],size(c,3))) > coeff_thresh;
    c = c(:,:,c_check_idx);
    p = p(:,:,c_check_idx);
    
    d_check_idx = vecnorm(reshape(d,[],size(d,3))) > coeff_thresh;
    d = d(:,:,d_check_idx);
    q = q(:,:,d_check_idx);
    
    % return updated coeffs
    coeff_hist.a = a; coeff_hist.b = b;
    coeff_hist.c = c; coeff_hist.d = d;
    coeff_hist.e = e; 
    coeff_hist.p = p; coeff_hist.q = q;
end