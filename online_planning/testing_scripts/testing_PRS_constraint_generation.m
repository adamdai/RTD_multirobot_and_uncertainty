%% description
% This script demonstrates how we create constraints for trajectory
% optimization for a single RTD receding-horizon planing iteration. We do
% the following:
%   1. slice the PRS by k_v (initial velocity parameter) and k_a (initial
%      acceleration parameter)
%   2. iterate through each time step of the PRS and each obstacle
%   3. for each time step and each obstacle, shift the obstacle (which is a
%      volume in workspace) into "local" coordinates relative to the
%      trajectory determined by k_v and k_a
%   4. determine if the obstacle overlaps with the box of distances
%      reachable for any choice of k_peak at the current time step
%   5. if the obstacle is reachable, intersect the k_peak box with the
%      obstacle box to eliminate all unsafe k_peak choices
%
% This script then plots a bunch of k_peak (i.e., peak velocity) samples
% and obstacles, as long as there are fewer than 10 obstacles.
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: 26 Oct 2020
% Updated: 1 Nov 2020
%
%% user parameters
% initial conditions
x_0 = zeros(3,1) ;
v_0 = [0.5;0;0] ;
a_0 = [0;0;0] ;

% v_peak list
n_v_peak = 1000 ;

% number of obstacles to try
n_obs = 100 ;

%% automated from here
% load the PRS
load('quadrotor_PRS_sparse_v_max_5_a_max_10.mat')

% make a list of v_peak values
v_peak = 5.*rand(3,n_v_peak) - 2.5 ;
% v_peak = make_v_peak_sphere(PRS.v_max,11,v_0) ;
% v_peak = [1.0 1.0  0.0 1.0 -1.0 ;
%           0.0 1.0 -1.0 0.5  0.5;
%           0.0 0.0  1.0 0.0  1.0] ;
n_v_peak = size(v_peak,2) ;

% make a list of obstacles
% obs_centers = [2 1 0.5;
%                0 1 -1 ;
%                0 0 1] ;
obs_centers = 10.*rand(3,n_obs) - 5 ;

% obs_side_lengths = 2.*rand(3,n_obs) ;
obs_side_lengths = 0.5.*ones(3,n_obs) ;
n_obs = size(obs_centers,2) ;

%% slicing for initial conditions
% subtract x_0 from the center of each obstacle
obs_centers = obs_centers - repmat(x_0,1,n_obs) ;

% get values to slice k_v and k_a
k_v_coeff = v_0 ./ PRS.v_max ;
k_a_coeff = a_0 ./ PRS.a_max ;

% slice the PRS by k_v and k_a, and shift by x_0
n_time = PRS.n_zonotopes ;
k_v_offset = k_v_coeff * PRS.zonotope_widths.k_v ;
k_a_offset = k_a_coeff * PRS.zonotope_widths.k_a ;
initial_condition_offset =  k_v_offset + k_a_offset ;

% get k_peak coefficient values
k_peak = v_peak ./ PRS.v_max ;

%% obstacle check
tic
% get obstacle upper and lower bounds
%%% HERE IS WHERE WE WOULD ADD TRACKING ERROR NORMALLY %%%
obs_half_side_lengths = 0.5.*obs_side_lengths ;

% set up to save each lower and upper bound of distances reacha
k_peak_unsafe_lbs = nan(3, n_time*n_obs) ;
k_peak_unsafe_ubs = nan(3, n_time*n_obs) ;
n_unsafe = 1 ;

% for each FRS time step...
for idx_time = 1:n_time
    % create new unsafe log
    v_peak_unsafe_log = false(1,n_v_peak) ;
    
    % get the PRS width at the current time
    k_peak_width = PRS.zonotope_widths.k_peak(idx_time) ;
    
    % make a k_peak box for the current time
    d_peak_lb = -k_peak_width.*ones(3,1) ;
    d_peak_ub =  k_peak_width.*ones(3,1) ;
    
    % for each obstacle...
    for idx_obs = 1:n_obs
        % shift the obstacle by the corresponding k_v and k_a values
        obs_center = obs_centers(:,idx_obs) - initial_condition_offset(:,idx_time) ;
        
        % store the upper and lower obstacle bounds
        obs_lb = obs_center - obs_half_side_lengths(:,idx_obs) ;
        obs_ub = obs_center + obs_half_side_lengths(:,idx_obs) ;
        
        % check if the k_peak box overlaps the obstacle box (i.e., do there
        % exist choices of k_peak that can reach the obstacle?)
        if all(d_peak_ub >= obs_lb & d_peak_lb <= obs_ub)
            % intersect the two boxes
            d_peak_lb_int = max([d_peak_lb,obs_lb],[],2) ./ k_peak_width ;
            d_peak_ub_int = min([d_peak_ub,obs_ub],[],2) ./ k_peak_width ;
            
            % check each bound
            chk = all(k_peak >= d_peak_lb_int & k_peak <= d_peak_ub_int,1) ;
            
            v_peak_unsafe_log = v_peak_unsafe_log | chk ;

            % store the new bounds
            k_peak_unsafe_lbs(:,n_unsafe) = d_peak_lb_int ;
            k_peak_unsafe_ubs(:,n_unsafe) = d_peak_ub_int ;
            
            n_unsafe = n_unsafe + 1 ;
        end
    end
    
    % delete unsafe trajs
    k_peak(:,v_peak_unsafe_log) = [] ;
    n_v_peak = size(k_peak,2) ;
end
toc

% % clean up extra columns
% % tic
% if n_unsafe < (n_time*n_obs)
%     k_peak_unsafe_lbs(:,n_unsafe:end) = [] ;
%     k_peak_unsafe_ubs(:,n_unsafe:end) = [] ;
% end
% % toc
% 
% % tic
% % create A and b matrices
% b_con = [-k_peak_unsafe_lbs ; k_peak_unsafe_ubs] ;
% b_con = b_con(:) ;
% A_con = repmat([eye(3) ; -eye(3)],n_unsafe-1,1) ;
% % toc
% 
% % tic
% % test the constraints
% C_eval = A_con*k_peak + b_con ;
% C_rshp = reshape(C_eval,6,[]) ;
% C_min = min(C_rshp,[],1) + 1e-6;
% C_rshp_2 = reshape(C_min,size(C_eval,1)/6,[]) ;
% C_max = max(C_rshp_2,[],1) ;
% v_peak_unsafe_log = C_max >= 0 ;
% toc

%% plotting setup
if n_obs <= 10
    % create obstacles
    obs = cell(1,n_obs) ;
    for idx = 1:n_obs
        obs{idx} = box_obstacle('center',obs_centers(:,idx),'side_lengths',obs_side_lengths(:,idx)) ;
    end
    
    % create trajectory for PRS
    trajs = cell(1,n_v_peak) ;
    for idx = 1:n_v_peak
        [~,Z_idx,~] = quadrotor_planning_model_3D(v_0,a_0,v_peak(:,idx),...
            0,PRS.t_peak,PRS.t_total,PRS.t_sample) ;
        X_idx = Z_idx(1:3,:) + repmat(x_0,1,size(Z_idx,2)) ;
        trajs{idx} = X_idx ;
    end
    
    %% plotting
    figure(1) ; clf ; axis equal ; hold on ; grid on ; view(3)
    
    % plot obstacle
    for idx = 1:n_obs
        plot(obs{idx})
    end
    
    % plot trajectory
    v_peak_idxs = 1:n_v_peak ;
    n_v_peak_plot = 60 ;
    if n_v_peak > n_v_peak_plot % plot no more than 20 random trajs
        v_peak_idxs = datasample(v_peak_idxs,n_v_peak_plot,'Replace',false) ;
    end
    for idx = v_peak_idxs
        if v_peak_unsafe_log(idx)
            plot_style = 'r-' ;
        else
            plot_style = 'b-' ;
        end
        plot_path(trajs{idx},plot_style,'linewidth',1)
        %plot_path(center_k_peak(:,1:3:end),'b.','markersize',15)
    end
    
    xlabel('x')
    ylabel('y')
    zlabel('z')
    set(gca,'FontSize',20)
end