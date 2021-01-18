% Compute forward stochastic reachable set of an n-dimensional LQG system 
% for a space of trajectory paramters
% 
% Leverages (slightly modified) linear reachability math from A. Shetty 2019
% 
% Inputs:
%   v_0   -
%   a_0   -
%   v_max -
%   sys   - struct containing system matrices A,B,and C, LQR gain K, and  
%           noise covariance matrices Q and R  
%   P0    - initial state covariance
%   N     - trajectory length
%   LPM   - linear planning model
%   sigma_bound - sigma confidence bound for extracting bounding FRS
%
% Outputs:
%   pXrs - cell array of probZonotopes representing stochastic forward 
%          reachable sets in time
%   Xrs  - cell array of zonotopes representing deterministic forward 
%          reachable sets in time
%   

function [pXrs, Xrs] = compute_online_FRS(p_0, v_0, a_0, v_max, sys, P0, LPM, sigma_bound)
    
    % retrieve system matrices from sys struct
    A = sys.A; B = sys.B; C = sys.C;
    K = sys.K; Q = sys.Q; R = sys.R;

    % trajectory length
    N = length(LPM.time);
    
    % define trajectory parameter space
    S = zonotope([v_0(1) 0     0; 
                  a_0(1) 0     0; 
                  0      v_max 0;
                  v_0(2) 0     0; 
                  a_0(2) 0     0; 
                  0      0     v_max]);
    
    % nominal initial state
    x_0 = [p_0; v_0];
    
    % initial probabilistic zonotope state
    X0 = probZonotope(x_0,cov2probGen(P0),sigma_bound);
    
    % trajectory parameter to control input mapping
    a_mat = LPM.acceleration;
    M = cell(1,length(a_mat));
    for i = 1:N
        M{i} = [a_mat(:,i)' zeros(1,3); 
                zeros(1,3)  a_mat(:,i)'];
    end
    
    % parameters
    n = size(A,1); % system dimension
    m = dim(S); % trajectory parameter dimension
    
    % initial state covariance
    P = sigma(X0); 
    P = P(1:n,1:n);
    
    % initial state estimation error
    WpZ = probZonotope(zeros(n,1),cov2probGen(Q),sigma_bound);
    VpZ = probZonotope(zeros(2,1),cov2probGen(R),sigma_bound);

    % initialize FRS
    pXrs = cell(1,N);
    Xrs = cell(1,N);
    pXrs{1} = [eye(n); zeros(m,n)] * x_0 + [zeros(n,m); zeros(m,m)] * S ...
              + [eye(n); zeros(m,n)]*(X0 + -x_0);
    umeanZ = mean(pXrs{1});
    covZ = cov2zonotope(sigma(pXrs{1}),sigma_bound,n);
    Xrs{1} = deleteZeros(umeanZ + covZ);

    % recursive coefficients
    coeff_a = eye(n); coeff_b = 0; % initialize 
    coeff_c = cell(1,N); coeff_d = cell(1,N);
    coeff_c{1} = nan; coeff_d{1} = nan;

    coeff_e = eye(n);
    coeff_p = cell(1,N); coeff_q = cell(1,N);
    coeff_p{1} = nan; coeff_q{1} = nan;
    
    for t = 2:N
        
        % update coeffs a and b
        coeff_a = (A-B*K)*coeff_a;
        coeff_b = (A-B*K)*coeff_b - B*K*coeff_e;

        % update coeffs c and d
        for i = 2:t-1
            coeff_c{i} = (A-B*K)*coeff_c{i} - B*K*coeff_p{i};
            coeff_d{i} = (A-B*K)*coeff_d{i} - B*K*coeff_q{i};
        end

        % add new coeffs c and d
        coeff_c{t} = eye(n);  coeff_d{t} = zeros(n,2);

        % calculate all CpZ and DpZ terms
        all_CpZ = [coeff_c{t}; zeros(m,n)] * WpZ;
        all_DpZ = [coeff_d{t}; zeros(m,2)] * VpZ;
        for i = 2:t-1
            all_CpZ = all_CpZ + [coeff_c{i}; zeros(m,n)] * WpZ;
            all_DpZ = all_DpZ + [coeff_d{i}; zeros(m,2)] * VpZ;
        end

        % compute reachable set
        AB_coeff = 0;
        for i = 1:t-1
            AB_coeff = AB_coeff + A^(t-i) * B * M{i};
        end

        % reachability expression
        pXrs{t} = [A^t; zeros(m,n)] * x_0 + [AB_coeff; eye(m,m)] * S ... 
                  + [coeff_a-coeff_b; zeros(m,n)] * (X0 + -x_0) + all_CpZ + all_DpZ;
        
        % extract 3-sigma bound zonotope
        umeanZ = mean(pXrs{t});
        covZ = cov2zonotope(sigma(pXrs{t}),sigma_bound,n);
        Xrs{t} = deleteZeros(umeanZ + covZ);

        % online filter steps
        P_pred = A*P*A' + Q;
        L = P_pred*C'/(C*P_pred*C' + R);
        P = P_pred - L*C*P_pred;

        % update coeff e
        coeff_e = (eye(n) - L*C)*A*coeff_e;

        % update coeffs p and q
        for i = 2:t-1
            coeff_p{i} = (eye(n) - L*C)*A*coeff_p{i};
            coeff_q{i} = (eye(n) - L*C)*A*coeff_q{i};
        end

        % add coeffs p and q for new w and v
        coeff_p{t} = -(eye(n) - L*C);  coeff_q{t} = L;

    end
end