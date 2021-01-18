% Step forward an LQG system
% (for now): assumes 1 step - x_nom,u_nom are single vectors
% 
% Inputs:
%   sys    - struct containing system matrices A,B,and C, LQR gain K, and  
%            noise covariance matrices Q and R  
%   N      - trajectory length
%   N_traj - number of trajectories to generate
%   u_nom  - nominal input sequence
%   x0     - initial state
%   P0     - initial covariance
%
% Outputs:
%   X - n x N x N_traj array containing N_traj n-dimensional trajectories 
%       of length N
%   

function [x,x_est,P] = LQG_step(sys,x,x_est,P,x_nom,u_nom)
    
    % retrieve system matrices from sys struct
    A = sys.A; B = sys.B; C = sys.C;
    K = sys.K; Q = sys.Q; R = sys.R;
    
    n = size(A,1); % system dimension
    m = size(R,1); % measurement dimension

    % apply feedback control
    err = x_est - x_nom;
    u = u_nom - K*err;

    % dynamics
    w = mvnrnd(zeros(n,1), Q, 1)';
    x = A*x + B*u + w;

    % noisy measurement
    v = mvnrnd(zeros(m,1), R, 1)';
    z = C*x + v;

    % Kalman filter predict
    x_pred = A*x_est + B*u;
    P_pred = A*P*A' + Q;

    % Kalman filter update
    L = P_pred*C'/(C*P_pred*C' + R);
    x_est = x_pred + L*(z - C*x_pred);
    P = P_pred - L*C*P_pred;

end