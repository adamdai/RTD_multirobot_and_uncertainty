%% description
% Use the Linear Planning Model to compute a "Planning Reachable Set (PRS)"
% which describes the reachable positions of planned trajectories over a 
% space of chosen trajectory parameters.
%
% In online planning, we will have online estimates of our initial
% condition (v_0, a_0), so we assume those to be fixed, and compute the PRS
% over a space of possible peak velocities.
%
% As an extension, we can consider uncertainty in v_0 and a_0 (e.g.
% provided by state estimator covariance), and compute the PRS over
% uncertain (zonotope) v_0 and a_0 in addition to v_peak.

%% setup

% load planning model
load('quadrotor_linear_planning_model.mat')

% maximum velocity
v_max = 3;

% initialize PRS
PRS = cell(1,N);

% % trajectory parameter space (zonotope) 
% v_pk_c = [0; 0; 0];
% K_c = [v_0; a_0; v_pk_c];
% K_G = [zeros(6,1), zeros(6,1), zeros(6,1);
%        [v_max;0;0],[0;v_max;0],[0;0;v_max]];
% K = zonotope(K_c,K_G);
% 
% % transform LPM 
% N = size(LPM.time,2);
% P_LPM = zeros(3*N,9);
% for i = 1:N
%     p = LPM.position(:,i)';
%     P_LPM(3*i+1,:) = [p, zeros(1,6)];
%     P_LPM(3*i+2,:) = [zeros(1,3), p, zeros(1,3)];
%     P_LPM(3*i+3,:) = [zeros(1,6), p];
% end

%% zero initial conditions
% P_lpm = LPM.position';
% 
% % peak velocity space zonotope
% c_pk = [0;0;0];
% G_pk = v_max * eye(3);
% V_pk = zonotope(c_pk,G_pk);
% 
% figure(1); hold on
% for i = 1:N
%     pos = P_lpm(i,3) * V_pk;
%     PRS{i} = augment(pos,V_pk);
%     plot(PRS{i})
% end
% 
% % slice in v_peak dimensions (4,5,6) for v_peak = (1,1,1)
% zonotope_slice(PRS{4},[4;5;6],[1;1;1]);

%% fixed initial conditions
% v0 = [4, 2, -5];
% a0 = [5, 5, -5];
% k0 = [v0; a0];
% 
% p0 = P_lpm(:,1:2) * k0;
% 
% figure(2); hold on
% for i = 1:N
%     plot(PRS{i} + [p0(i,:)';zeros(3,1)])
% end

%% uncertain initial conditions
V0 = zonotope([4;2;-5],0.1*eye(3));
A0 = zonotope([5;5;-5],0.1*eye(3));

% zeros zonotope
Z0 = zonotope([0;0;0],zeros(3));

figure(3); hold on
for i = 1:N
    pos_v0 = P_lpm(i,1) * V0;
    pos_a0 = P_lpm(i,2) * A0;
    pos_vpk = P_lpm(i,3) * V_pk;
    PRS{i} = augment(pos_v0,Z0) + augment(pos_a0,Z0) + augment(pos_vpk,V_pk);
    plot(PRS{i})
end

% slice in v_peak dimensions (4,5,6) for v_peak = (-2,2,1)
slc = cell(1,N);
figure(3); hold on
for i = 1:N
    slc{i} = zonotope_slice(PRS{i},[4;5;6],[-0.5;2.5;1]);
    plot(slc{i},[1 2],'r');
end

