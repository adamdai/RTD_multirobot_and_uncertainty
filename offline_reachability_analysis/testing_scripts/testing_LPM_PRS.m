%% description
% Use the Linear Planning Model to compute a "Planning Reachable Set (PRS)"
% which describes the reachable positions of planned trajectories over a 
% space of chosen trajectory parameters.
%
% In online planning, we will have online estimates of our initial
% condition (v_0, a_0), so we assume those to be fixed, and compute the PRS
% over a space of possible peak velocities.

%% setup

% load planning model
load('quadrotor_linear_planning_model.mat')

% specify some arbitary initial condition 
v_0 = [1; -1; 1];
a_0 = [0; 0; 0];

% maximum velocity
v_max = 3;

% trajectory parameter space (zonotope) 
v_pk_c = [0; 0; 0];
K_c = [v_0; a_0; v_pk_c];
K_G = [zeros(6,1), zeros(6,1), zeros(6,1);
       [v_max;0;0],[0;v_max;0],[0;0;v_max]];
K = zonotope(K_c,K_G);

% transform LPM 
N = size(LPM.time,2);
P_LPM = zeros(3*N,9);
for i = 1:N
    p = LPM.position(:,i)';
    P_LPM(3*i+1,:) = [p, zeros(1,6)];
    P_LPM(3*i+2,:) = [zeros(1,3), p, zeros(1,3)];
    P_LPM(3*i+3,:) = [zeros(1,6), p];
end
