%% description
% Sample initial and peak velocities according to the scheme described in 
% Kousik et al. 2019

%% parameters 

% load planning model
load('quadrotor_linear_planning_model.mat')

% maximum speed [m/s]
v_max = 3;



% trajectory parameter space bounds
v_0_max = 2; a_0_max = 2; v_peak_max = 2;
% discretization (for now use the same for all parameters)
dv = 0.5;

% trajectory parameter space
v_0_range = -v_0_max:dv:v_0_max;
a_0_range = -a_0_max:dv:a_0_max;
v_peak_range = -v_peak_max:dv:v_peak_max;

% store trajectories
num_trajs = length(v_0_range) * length(a_0_range) * length(v_peak_range);
P = zeros(3,size(LPM.time,2),num_trajs);

%% sampling

% iterate over cover elements D^(j)


% iterate through trajectory parameter space
figure(1); hold on; axis equal
for v_peak_x = -v_peak_max:dv:v_peak_max
    for v_peak_y = -v_peak_max:dv:v_peak_max
        for v_peak_z = -v_peak_max:dv:v_peak_max
            k = [1 0 v_peak_x;
                 3 -2 v_peak_y;
                 0 3 v_peak_z];
            p = k * LPM.position;
            plot3(p(1,:),p(2,:),p(3,:));
        end
    end
end



N = size(LPM.time,2);

a_mat = LPM.acceleration;