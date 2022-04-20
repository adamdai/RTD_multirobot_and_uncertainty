%% description
% This script tests reachability-based planning for multiple robots modeled
% as Linear Quadratic Gaussian (LQG) systems
%
% Authors: Adam Dai
%          Shreyas Kousik
% Created: 12 Jan 2021
% Updated: 
%   
clear ; clc
%% user parameters
% seed
%rng('default')
% plotting parameters
flag_save_gif = true ;
gif_delay_time = 1/20 ; % 1/fps
gif_filename = 'multi_agent_uncertain_planning.gif' ;

% simulation timing parameters
t_sim_total = 10 ; % [s]
t_sim_sample = 0.1 ; % [s]

% world and obstacle parameters (obstacles are all static for now)
% obstacles are rectangular zonotopes with generators (1/2 side-length) 
% between w_obs_min and w_obs_max 
n_dim = 2 ;
world_bounds = 5.*[-1,1,-1,1] ; % 2-D world
n_obs = 10 ;
w_obs_min = 0.5 ; % minimum obstacle width [m] 
w_obs_max = 1.0 ; % maximum obstacle width [m]
r_goal_reached = 0.3 ; % [m] stop planning when within this dist of goal

flag_gen_world = false ; % whether to generate a new world or load a previously saved one
flag_save_world = false ; % whether to save the generated world
save_world_dir = 'C:\Users\adamd\OneDrive\Documents\Stanford\Research\NAVLab\projects\RTD\RTD_multirobot_and_uncertainty\online_planning\worlds' ; 
file_load_world = 'collision_example.mat' ; % mat file to load world

% agent parameters
r_agents = 0.25 ; % [m]
v_max = 2 ; % [m/s] max allowed velocity magnitude along either dimension
             % (i.e. -v_max <= v_x <= v_max & -v_max <= v_y <= v_max)
delta_v_peak_max = 3 ; % [m/s] max 2-norm change in v_peak allowed between plans

% planning parameters -- note that the length of t_plan, t_check, and
% t_recheck should be the same, and equal to the number of agents that you
% want to tootle around
%  -> use 
n_agents = 4 ;
t_plan = 0.1 * ones(1,n_agents) ; % [s] amount of time allotted for planning
t_check = 0.1 * ones(1,n_agents) ; % [s] amount of time allotted for check
t_recheck = 0.1 * ones(1,n_agents) ; % [s] amount of time allotted for recheck
n_plan_max = 10000 ; % max number of plans to evaluate


%% coloring parameters

color_obs = 'r' ; 
color_goal = 'g' ;
color_reach = 'b' ;
color_face_robot = [150 241 255] / 255 ; % light blue
color_edge_robot = [0 32 96] / 255 ; % dark blue

color_plan = [0.5 0.5 1] ; % blue
color_frs = [205 248 255] / 255 ; % light blue


%% automated from here
% load the linear planning model
load('quadrotor_linear_planning_model.mat')

% set up simulation timing
T_sim_vec = 0:t_sim_sample:t_sim_total ;
n_t_sim = length(T_sim_vec) ;


%% double integrator system
n_s = 4; % state dimension
n_i = 2; % input dimension
n_m = 2; % measurement dimension
pos_i = 1:2; % position indices

% trajectory discretization and length (planning horizon)
t_f = LPM.t_total; dt = LPM.t_sample; N = length(LPM.time); t_peak = LPM.t_peak;

% system
A = eye(n_s);
A(1:2,3:4) = dt*eye(2); % dyanmics matrix
B = [dt^2/2 0; 
     0      dt^2/2; 
     dt     0; 
     0      dt]; % input matrix
C = [1 0 0 0;  
     0 1 0 0]; % measurement matrix
 
K = dlqr(A,B,eye(n_s),eye(n_i)); % feedback law u = -Kx

Q = 0.01*[dt^3/3 0      dt^2/2 0;  
         0      dt^3/3 0      dt^2/2; 
         dt^2/2 0      dt     0; 
         0      dt^2/2 0      dt]; % process noise covariance
R = 0.001*eye(n_m); % measurement noise covariance

% form sys struct
sys.A = A; sys.B = B; sys.C = C;
sys.K = K; sys.Q = Q; sys.R = R;


%% other variables needed for reachability 

k_dim = [7; 10]; % dimensions in which v_peak exists
obs_dim = [1,; 2]; % obstacles live in x-y workspace
sigma_bound = 3 ; % use a 3-sigma confidence bound
n_params = 6 ; % total number of trajectory params (v0, v_peak, a0) for x and y
n_FRS = n_s + n_params ; % dimension of the FRS zonotopes

P0 = 0.1*diag([0.01 0.01 0.0001 0.0001]) ; % initial state covariance
P0_full = zeros(n_FRS); P0_full(1:n_s,1:n_s) = P0 ; % initial covariance over state and params

c_k = [0;0]; g_k = [v_max;v_max]; % center and generators for v_peak space

% zonotope representing volume of a single agent - use to buffer obstacles
agent_zono = zonotope([zeros(2,1) (r_agents/2) * eye(2)]); 

%% agent setup
% get the number of agents
n_agents = length(t_plan) ;

% set up agent timing
t_next_plan = zeros(1,n_agents) ; % each agent will replan at this time
t_next_check = t_plan ; % each agent will check its plan at this time
t_next_recheck = t_plan + t_check ;
t_committed = zeros(1,n_agents) ;
n_t_plan = length(LPM.time) ;

% set up check for goals reached by the agents and the planners
chk_goal_reached_by_agent = false(1,n_agents) ;
chk_goal_reached_by_plan = false(1,n_agents) ;


%% world setup
% if load_world empty, generate a new world
if flag_gen_world 
    % generate uniformly-distributed random obstacles
    O_ctr = rand_in_bounds(world_bounds,[],[],n_obs) ; % obstacle positions
    O_wid = rand_range(w_obs_min,w_obs_max,[],[],2,n_obs) ; % obstacle widths

    % obstacle zonotope array
    O_zono = cell(1,n_obs) ;
    O_buff = cell(1,n_obs) ; % buffered by agent width
    for i = 1:n_obs % (is there a vectorized way to do this?)
        % randomly rotate generators
        O_gen = O_wid(:,i) .* eye(n_dim) ;
        O_gen = rotation_matrix_2D(pi*rand()) * O_gen;
        %O_gen = rand_range(w_obs_min,w_obs_max,[],[],n_dim,n_dim) ;
        O_zono{i} = zonotope([O_ctr(:,i) O_gen]) ;
        O_buff{i} = O_zono{i} + agent_zono ; 
    end

    % create random start and goal locations that are feasible to get to
    disp(' generating starting locations')
    P_start = make_random_feasible_locations(n_agents,r_agents,O_buff,world_bounds) ;
    P_goal = make_random_feasible_locations(n_agents,r_agents,O_buff,world_bounds) ;
    
    % save the world to a .mat file
    if flag_save_world
        world.obs = O_zono ;
        world.obs_buff = O_buff ;
        world.p_start = P_start ;
        world.p_goal = P_goal ;
        filename = strcat('world_',num2str(n_agents),'_agents_',num2str(n_obs),'_obs') ;
        disp(' saving generated world')
        save(fullfile(save_world_dir,filename),'world') ;
    end

else
    % otherwise, load the world
    disp(' loading previous world')
    load(fullfile(save_world_dir,file_load_world)) ;
    O_zono = world.obs ;
    O_buff = world.obs_buff ;
    P_start = world.p_start ;
    P_goal = world.p_goal ;
end


%% initialize agents and plans
% initialize the position, velocity, and acceleration of each agent as
% columns of the agent_state array (3*n_dim x n_agents)
agent_state = [P_start ;
               zeros(2*n_dim,n_agents)] ;

% initialize state estimate and covariance for each agent's Kalman filter
agent_state_est = [P_start ;
                   zeros(n_dim,n_agents)] ;
agent_cov = repmat(P0,1,1,n_agents) ;

% initialize reachable sets for each agent based on initial covariance
agent_state_set = cell(1,n_agents) ;
for i = 1:n_agents
   agent_state_set{i} = [agent_state(1:n_s,i); zeros(n_params,1)] + deleteZeros(cov2zonotope(P0_full,sigma_bound,n_FRS)) ; 
end

% history of noise term coefficients used in FRS computation
agent_coeff_hist = cell(1,n_agents) ;
for i = 1:n_agents
   agent_coeff_hist{i}.a = eye(n_s) ; 
   agent_coeff_hist{i}.b = 0 ; 
   agent_coeff_hist{i}.c = zeros(n_s,n_s,0) ; 
   agent_coeff_hist{i}.d = zeros(n_s,2,0) ; 
   agent_coeff_hist{i}.e = eye(n_s) ; 
   agent_coeff_hist{i}.p = zeros(n_s,n_s,0) ; 
   agent_coeff_hist{i}.q = zeros(n_s,2,0) ; 
end
           
% initialize plan histories of each agent; 
% each committed and pending plan history is a struct containing:
%  time - time vector associated with the plan 
%  plan - planned nominal trajectory
%  FRS - FRS of the planned trajectory
plans_committed.time = cell(1,n_agents) ;
plans_committed.plan = cell(1,n_agents) ;
plans_committed.FRS = cell(1,n_agents) ;
plans_committed.vpeak = zeros(n_dim,n_agents) ;

plans_pending.time = cell(1,n_agents) ;
plans_pending.plan = cell(1,n_agents) ;
plans_pending.FRS = cell(1,n_agents) ;
plans_pending.vpeak = zeros(n_dim,n_agents) ;

for i = 1:n_agents
   plans_committed.FRS{i} = cell(1,N) ;
   plans_pending.FRS{i} = cell(1,N) ; 
end

% planning limits
V_bounds = delta_v_peak_max.*repmat([-1,1],1,n_dim) ;

% populate the agent plans list with initial plans where they stay in place
for idx_agent = 1:n_agents
    plans_committed.time{idx_agent} = LPM.time ;
    plans_committed.plan{idx_agent} = repmat(agent_state(:,idx_agent),1,length(LPM.time)) ;
    plans_committed.FRS{idx_agent}(:) = agent_state_set(idx_agent) ;
    
    plans_pending.time{idx_agent} = LPM.time ;
    plans_pending.plan{idx_agent} = repmat(agent_state(:,idx_agent),1,length(LPM.time)) ;
    plans_pending.FRS{idx_agent}(:) = agent_state_set(idx_agent) ;
end
           

%% plot setup
% plotting order/priority (from lowest to highest):
%  1. obstacles
%  2. FRSes
%  3. goal/start positions
%  4. agents/plans

figure(1) ; clf ; axis equal ; hold on ; grid on ;
axis(1.25.*(world_bounds))

% obstacles
for idx = 1:n_obs
    p = plot(O_zono{idx},[1, 2],'FaceColor',color_obs,'EdgeColor',color_obs,'Filled',true);
    p.FaceAlpha = 0.1;
end

% FRSes
plot_data_FRS = [] ;
for idx_agent = 1:n_agents
    % plot FRS (just plot the first zonotope for now since they are all the
    % same across time initially)
    h_FRS = plot(plans_committed.FRS{idx_agent}{1},[1, 2],'FaceColor',color_frs,'EdgeColor',color_frs,'Filled',true) ;
    h_FRS.FaceAlpha = 0.1 ;
    plot_data_FRS = [plot_data_FRS, h_FRS] ;
end

% start positions
plot_path(agent_state(1:n_dim,:),'go','markersize',15,'linewidth',1.5)

% goal areas
% for idx = 1:n_agents
%     plot_disc(r_goal_reached,P_goal(:,idx),'facecolor',color_goal,'edgecolor',...
%         color_goal,'facealpha',0.1)
% end
plot_path(P_goal,'gp','markersize',15,'markerfacecolor',color_goal)

% current agent positions
plot_data_agents = [] ;
plot_data_plans = [] ;
for idx_agent = 1:n_agents
    % plot plans
    h_plans = plot_path(plans_committed.plan{idx_agent}(1:n_dim,:),...
        '-','color',color_plan,'linewidth',1.25) ;
    plot_data_plans = [plot_data_plans, h_plans] ;
    
    % plot agent
    h_agent = plot_disc(r_agents,agent_state(1:n_dim,idx_agent),...
        'facecolor',color_face_robot,'edgecolor',color_edge_robot,'facealpha',1.0,'linewidth',2) ;
    plot_data_agents = [plot_data_agents, h_agent] ;
end


%% set up to save gif
if flag_save_gif
    % get current figure
    fh = get(groot,'CurrentFigure') ;
    frame = getframe(fh) ;
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    
    imwrite(imind,cm,gif_filename,'gif', 'Loopcount',inf,...
        'DelayTime',gif_delay_time) ;
end


%% run simulation
t_spent_recheck = 0 ;

tic_real_time = tic ;
for idx = 1:n_t_sim
    % get the current time
    t_sim = T_sim_vec(idx) ;
    
    disp(['t = ',num2str(t_sim)])
    
    % FOR DEBUGGING: check for agent collisions
%     chk_pos = check_agent_pos_collisions(agent_state, 2*r_agents);
%     if ~chk_pos
%         disp('COLLISION')
%     end
    
    % iterate through the agents to update them for the current time (note
    % that we can randomize the order of this in the future)
    for idx_agent = 1:n_agents
        %% setup
        % get the current agent's plan
        T_old = plans_committed.time{idx_agent} ;
        X_old = plans_committed.plan{idx_agent} ;
        FRS_old = plans_committed.FRS{idx_agent} ;
        
        if t_sim >= t_next_plan(idx_agent) && (~chk_goal_reached_by_plan(idx_agent))
        %% planning
        % if the current time is the current agent's next replan time, then
        % that agent needs to replan (duh)
            disp(['  agent ',num2str(idx_agent),' planning'])
            
            %% planning setup
            % start timer
            tic_start_plan = tic ;
            
            % get the initial condition for the current plan
            x_0 = match_trajectories(t_sim+t_plan,T_old,X_old) ;    
            
            p_0 = x_0(1:n_dim,idx_agent) ;
            v_0 = x_0(n_dim + (1:n_dim),idx_agent) ;
            a_0 = x_0(2*n_dim + (1:n_dim),idx_agent) ;
            
            x_start = [P_start(:,idx_agent); zeros(2,1)] ;
            
            % compute FRS from initial conditions 
            hist = agent_coeff_hist{idx_agent} ; 
            [pXrs, FRS_idx, hist] = compute_online_FRS(x_start, p_0, v_0, a_0, v_max, sys, P0, LPM, hist, sigma_bound) ;
            agent_coeff_hist{idx_agent} = hist ; 
            
            % create the time vector for the new plan (to start at t_plan)
            T_new = LPM.time + t_sim + t_plan(idx_agent) ;
            
            % get the reachable sets of all other agents to use for
            % inter-agent collision checking
            [P_other, FRS_other] = get_other_agents_committed_plans(n_agents,...
                idx_agent,plans_committed,n_dim,T_new) ;
            
            % generate inter-agent collision constraints from reachable sets
            [A_con_FRS, b_con_FRS] = generate_collision_constraints(FRS_idx,FRS_other,k_dim,obs_dim);
            
            % generate obstacle collision constraints from reachable sets
            [A_con_obs, b_con_obs] = generate_obs_constraints(FRS_idx,O_zono,k_dim,obs_dim); %% use O_buff instead
            
            %% find new plan (i.e., perform trajectory optimization)
            % this is using a sampling-based approach to attempt to find a
            % new plan for the current agent within time t_plan; note that
            % time is "paused" artificially during this trajectory
            % optimization
            
            % get a bunch of random potential v_peak samples
            V_peak = rand_in_bounds(V_bounds,[],[],n_plan_max) ;
            
            % eliminate samples that exceed the max velocity and max delta
            % from the initial velocity
            V_peak_mag = vecnorm(V_peak) ;
            delta_V_peak_mag = vecnorm(V_peak - repmat(v_0,1,n_plan_max)) ;
            V_peak_test = (V_peak_mag > v_max | delta_V_peak_mag > delta_v_peak_max) ;
            V_peak(:,V_peak_test) = [] ;
            
            % get the number of potentially-feasible samples to iterate
            % through
            n_V_peak = size(V_peak,2) ;
            
            % get the final positions for each of these v_peaks; this is
            % used to compute the cost of every v_peak sample, because we
            % want to minimize the distance between each trajectory's final
            % position and the global goal
            LPM_p_final = LPM.position(:,end) ;
            P_from_v_0_and_a_0 = [v_0, a_0] * LPM_p_final(1:2) + p_0 ; % plug in v_0 and a_0
            P_from_v_peak = LPM_p_final(3) * V_peak + repmat(P_from_v_0_and_a_0,1,n_V_peak) ; % plug in v_peak
            
            % the array P_from_v_peak is of size n_dim x n_V_peak; each ith
            % column in this array is the final position achieved by the
            % trajectory parameterized by k = (v_0, a_0, V_peak(:,i))
            
            % sort these V_peaks by their (sub)optimality in terms of
            % distance of each final position to the global goal
            dist_to_goal = vecnorm(P_from_v_peak - repmat(P_goal(:,idx_agent),1,n_V_peak)) ;
            [~,V_sort_idxs] = sort(dist_to_goal,'ascend') ;
            V_peak = V_peak(:,V_sort_idxs) ;
            
            % iterate through the V_peaks until one is feasible
            idx_v_peak = 1 ;
            flag_v_peak_feas = false ; % pessimism
            while ((idx_v_peak <= n_V_peak)) %&& ...
                   %(toc(tic_start_plan) < t_plan(idx_agent))) % ignore timing constraint for now
                
                % get the position trajectory for the current v_peak
                v_peak = V_peak(:,idx_v_peak) ;
                
                % slice the FRS by this v_peak 
                FRS_slc = cell(1,N);
                FRS_slc{1} = FRS_idx{1}; % initial set is not sliceable w.r.t inputs
                for i = 2:length(FRS_idx) 
                    FRS_slc{i} = zonotope_slice(FRS_idx{i},k_dim,v_peak);
                end
                
                % compute nominal plan for this v_peak 
                P_idx = [v_0, a_0, v_peak]*LPM.position + repmat(p_0,1,n_t_plan) ;
                
                % check against other FRSes
                % evaluate collision check constraints for this v_peak
                % =====
                chk_FRS = check_agent_FRS_collisions(A_con_FRS,b_con_FRS,v_peak,c_k,g_k) ;
                
                % check against obstacles
                % =====
                chk_obs = check_agent_FRS_collisions(A_con_obs,b_con_obs,v_peak,c_k,g_k) ;
                
                % stop optimizing if found a feasible plan
                if chk_obs && chk_FRS % && chk_plans
                    flag_v_peak_feas = true ;
                    break
                else
                    % increment index
                    idx_v_peak = idx_v_peak + 1 ;
                end
            end
            
            if ~flag_v_peak_feas
               % if no new plan was found, continue the previous plan
               disp('    found no new plan')
               %disp(['    chk_obs: ',num2str(chk_obs),' chk_FRS: ',num2str(chk_FRS)])
               
               T_log = T_old >= t_sim ;
               
               % also, increase the length of the old plan by t_plan
               T_new = [T_old(T_log), T_old(end) + t_plan(idx_agent)] ;
               X_new = [X_old(:,T_log), X_old(:,end)] ;          
               FRS_new = [FRS_old(:,T_log), FRS_old(:,end)] ;
            else
               % otherwise, create a new plan
               disp('    found new plan')
               
               X_new = [[v_0, a_0, v_peak]*LPM.position + repmat(p_0,1,n_t_plan) ;
                        [v_0, a_0, v_peak]*LPM.velocity ;
                        [v_0, a_0, v_peak]*LPM.acceleration] ;

               FRS_new  = FRS_slc ;
                    
               % check if the global goal has been reached
               if vecnorm(X_new(1:n_dim,end) - P_goal(:,idx_agent)) < r_goal_reached
                   chk_goal_reached_by_plan(idx_agent) = true ;
               end
            end           
            
            %% planning wrap up
            % append the previous trajectory to the new trajectory
            T_old_log = T_old < T_new(1) ;
            T_new = [T_old(T_old_log), T_new] ;
            X_new = [X_old(:,T_old_log), X_new] ;
            FRS_new = [FRS_old(:,T_old_log), FRS_new] ;
            
            % update plans pending
            plans_pending.time{idx_agent} = T_new ;
            plans_pending.plan{idx_agent} = X_new ;
            plans_pending.FRS{idx_agent} = FRS_new ;
            plans_pending.vpeak(:,idx_agent) = v_peak ;
            
            % set the times to begin checking if a new plan was found, or
            % else try planning again at the current time plus t_plan
            if flag_v_peak_feas
                t_next_check(idx_agent) = t_sim + t_plan(idx_agent) ;
                t_next_recheck(idx_agent) = t_next_check(idx_agent) + t_check(idx_agent) ;
                t_next_plan(idx_agent) = t_next_recheck(idx_agent) + t_plan(idx_agent) ;
            else
                t_next_plan(idx_agent) = t_sim + t_plan(idx_agent) ;
                t_next_check(idx_agent) = inf ;
                t_next_recheck(idx_agent) = inf ;
            end
        
            % figure out how much time was spent
            t_plan_spent = toc(tic_start_plan) ;
            
            %% plotting plans
            % plot new plan
            plot_data_plans(idx_agent).XData = X_new(1,:) ;
            plot_data_plans(idx_agent).YData = X_new(2,:) ;
            
            % for 3-D
            if n_dim == 3
                plot_data_plans(idx_agent).ZData = X_new(3,:) ;
            end
            
            % set line style to pending
            plot_data_plans(idx_agent).LineStyle = '--' ;
            
            %% plotting FRS
            % get the vertices of all the zonotopes across time and form 
            % them all into one big vector of vertices to be plotted together
            FRS_vertices = [] ;
            FRS_faces = [] ; 
            face_idx = 1 ; % running count of next idx to use in faces 
            for i = 1:length(FRS_new)
                
                % project the zonotope into workspace and organize vertices
                % in CCW order for plotting
                zono_vertices = points_to_CCW(vertices(zonotope(FRS_new{i}.Z(1:2,:)))) ;
                FRS_vertices = [FRS_vertices zono_vertices] ;
                % append nan padding to faces to make num vertices consistent
                zono_faces = face_idx:face_idx+length(zono_vertices)-1 ; 
                padding = zono_faces(end) * ones(1,9-length(zono_vertices)) ; % assuming max number vertices = 9
                FRS_faces = [FRS_faces; [zono_faces padding]] ;
                face_idx = zono_faces(end) + 1 ; % update face_idx
            end
            
            plot_data_FRS(idx_agent).Vertices = FRS_vertices' ;
            plot_data_FRS(idx_agent).Faces = FRS_faces ;
            
        %% checking
        elseif t_sim >= t_next_check(idx_agent)
            disp(['  agent ',num2str(idx_agent),' checking'])
            
            tic_start_check = tic ;
            
            % get the pending plan
            T_pend = plans_pending.time{idx_agent} ;
            X_pend = plans_pending.plan{idx_agent} ;
            FRS_pend = plans_pending.FRS{idx_agent} ;
            vpeak_pend = plans_pending.vpeak(:,idx_agent) ;
            
            if ~isempty(X_pend)
                %% old plan collision checking
                % get the position coordinates of the plan
                P_pend = X_pend(1:n_dim,:) ;
                
                % get the committed trajectories of all other agents
                P_other = get_other_agents_committed_plans(n_agents,...
                    idx_agent,plans_committed,n_dim,T_pend) ;
                
                % check against the other plans
                chk_plans = check_agent_plan_collisions(P_other,P_pend,...
                    n_agents,r_agents) ;
                
                %% FRS collision checking
                % may have to implement different (simpler) checking that
                % just checks 2 FRSes against each other 
                % - FRS_pend is already sliced whereas FRS_idx is unsliced
%                 [P_other, FRS_other] = get_other_agents_committed_plans(n_agents,...
%                     idx_agent,plans_committed,n_dim,T_new) ;
%             
%                 [A_con_FRS, b_con_FRS] = generate_collision_constraints(FRS_pend,FRS_other,k_dim,obs_dim);
%                 
%                 chk_plans = check_agent_FRS_collisions(A_con_FRS,b_con_FRS,vpeak_pend,c_k,g_k) ;
                
                % make sure we didn't take too much time
                chk_time = toc(tic_start_check) < t_check(idx_agent) ;
                 
                if chk_plans
                    % if we passed the check, do nothing and move on to the
                    % recheck
                    disp('    new plan check ok')
                else
                    % if the agent failed the check, discard its pending plan
                    plans_pending.time{idx_agent} = [] ;
                    plans_pending.plan{idx_agent} = [] ;
                    plans_pending.FRS{idx_agent} = [] ;
                    disp('    new plan check fail')
                    
                    % skip the re-check and start planning
                    t_next_plan(idx_agent) = t_sim + t_check(idx_agent) ;
                end
            end
            
            % mark this check as complete by setting time to inf
            t_next_check(idx_agent) = inf ;
            
        %% rechecking
        elseif t_sim >= t_next_recheck(idx_agent)
            tic_start_recheck = tic ;
            
            disp(['  agent ',num2str(idx_agent),' rechecking'])
            
            % get the pending plan for the current agent
            T_pend = plans_pending.time{idx_agent} ;
            X_pend = plans_pending.plan{idx_agent} ;
            FRS_pend = plans_pending.FRS{idx_agent} ;
            
            if ~isempty(X_pend)
                % if our pending plan isn't empty, then check if other
                % agents have published a plan during our check
                idx_other_agents = 1:n_agents ;
                idx_other_agents(idx_agent) = [] ; % indices of other agents
                t_committed_others = t_committed(idx_other_agents) ;
                chk_others_pub = any((t_committed_others >= t_next_check(idx_agent)) & ...
                    (t_sim < t_committed_others)) ;
                
            else
                chk_others_pub = true ;
            end
            
            % if the current pending plan is nonempty and no new plans have
            % been published since the current agent began checking, then
            % we're ok to commit to our new plan
            if ~chk_others_pub
                disp('    new plan recheck ok')
                
                % if the plan check was ok, commit it
                plans_committed.time{idx_agent} = T_pend ; % time
                plans_committed.plan{idx_agent} = X_pend ; % traj
                plans_committed.FRS{idx_agent} = FRS_pend ; % FRS
                t_committed(idx_agent) = t_sim ;
            end
            
            % mark this recheck as complete by setting its time to inf
            t_next_recheck(idx_agent) = inf ;
            
            t_spent_recheck(idx) = toc(tic_start_recheck) ;
            
            %% update committed plan line style
            if ~isempty(X_pend)
                plot_data_plans(idx_agent).LineStyle = '-' ;
            end
        end
        
        %% state update
        % step the LQG system forward in order to update the state, state
        % estimate, and covariance of this agent
        
        % retrive state, state estimate, and covariance to feed into LQG step
        current_state = agent_state(:,idx_agent);
        x = current_state(1:4); x_est = agent_state_est(:,idx_agent);
        P = agent_cov(:,:,idx_agent);
        % get x_nom and u_nom from plan/LPM
        nom_state = match_trajectories(t_sim,T_old,X_old) ;
        x_nom = nom_state(1:4); u_nom = nom_state(5:6);
        % step thru LQG system to update states
        [x,x_est,P] = LQG_step(sys,x,x_est,P,x_nom,u_nom);
        % save updated states back into storing variables
        agent_state(:,idx_agent) = [x; u_nom]; agent_state_est(:,idx_agent) = x_est;
        agent_cov(:,:,idx_agent) = P;
        
        % check if the agent reached the goal
        chk_goal_reached_by_agent(idx_agent) = vecnorm(...
            agent_state(1:n_dim,idx_agent) - P_goal(:,idx_agent)) <= r_goal_reached ;
        
        %% plotting agents
        % plot agent
        if n_dim == 2
            V_new = make_circle(r_agents,100,agent_state(1:2,idx_agent)) ;
            plot_data_agents(idx_agent).Vertices = V_new' ;
        else
            error('3-D plot not done yet!')
        end
    end
    %% pause for plots to update
    if flag_save_gif && (mod(t_sim,gif_delay_time) < 1e-6)
        % get current figure
        fh = get(groot,'CurrentFigure') ;
        frame = getframe(fh) ;
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
    
        % append to gif
        imwrite(imind,cm,gif_filename,'gif','WriteMode','append',...
            'DelayTime',gif_delay_time) ;
    else
        pause(t_sim_sample)
    end
    
    %% end simulation if all goals were reached
    if all(chk_goal_reached_by_agent)
        disp('all goals have been reached by all agents!')
        break
    end    
end

t_sim_total = toc(tic_real_time) ;

disp(['Total real time spent: ',num2str(t_sim_total,'%0.2f'),' s'])

%% helper functions
% given the obstacles and world bounds, find a set of initial locations in
% bounds which are sufficiently far from the obstacles and from each other
function P_out = make_random_feasible_locations(n_agents,r_agents,O_buff,world_bounds)
    flag_P_feas = false ;
    
    while ~flag_P_feas
        % create random positions
        P_out = rand_in_bounds(world_bounds,[],[],n_agents) ;
        
        n_dim = size(P_out,1) ;
        
        % check position distances to obstacles
        obs_feas = true ;
        obs_buffer = 0.25 ;
        % for each position
        for i = 1:n_agents
            % for each obstacle
            for j = 1:length(O_buff)
                % convert position to zonotope with some buffering
                Z_pos = zonotope([P_out(:,i) obs_buffer * eye(n_dim)]) ;
                % check if they intersect
                if isIntersecting(Z_pos,O_buff{j})
                    obs_feas = false ;
                    break
                end
            end
            if ~obs_feas
                break
            end
        end
        
        % check position distances to each other (greater than 4 agent
        % radiuses apart)
        D_to_self = dist_points_to_points(P_out,P_out) ;
        D_to_self(D_to_self == 0) = nan ; % ignore same points
        pos_feas = ~any(D_to_self(:) < 4*r_agents) ;
        
        % if positions are not too close to obstacle and not too close to
        % each other, break
        if obs_feas && pos_feas
            break
        end
    end
end

% retrieve the committed plans and FRSes of other agents, and line them up
% with the current agent's planned time
function [P_other, FRS_other] = get_other_agents_committed_plans(n_agents,idx_agent,plans_committed,n_dim,T_new)
    idx_other_agents = 1:n_agents ;
    idx_other_agents(idx_agent) = [] ; % indices of other agents
    
    P_other = [] ;
    FRS_other = [] ;
    for idx_other = idx_other_agents
        % get the time and plan
        T_idx = plans_committed.time{idx_other} ;
        P_idx = plans_committed.plan{idx_other}(1:n_dim,:) ;
        
        % get the FRS
        FRS_idx = plans_committed.FRS{idx_other} ;

        % get all the parts of the plan that are relevant to the
        % current agent's new plan
        P_idx = match_trajectories(T_new,T_idx,P_idx) ;
        FRS_idx = match_cells(T_new,T_idx,FRS_idx) ;

        % add the new plan to the old plan
        P_other = [P_other, P_idx] ; % should preallocate this
        FRS_other = [FRS_other, FRS_idx] ;
    end
end

% check that, at all timesteps, agent's plan does not come closer than 
% 2 agent radii to other agents' plans
function chk_plans = check_agent_plan_collisions(P_other,P_agent,n_agents,r_agents)
% return TRUE if the plan is ok, and FALSE if the plan is NOT ok
    if ~isempty(P_other)
        dist_to_others = vecnorm(P_other - repmat(P_agent,1,n_agents-1)) ;
        chk_plans = ~any(dist_to_others <= 2*r_agents) ;
    else
        chk_plans = false ;
    end
end


% check if any agents are currently colliding
% use for debugging
function chk_pos = check_agent_pos_collisions(agent_state, buff)
% return TRUE if no collisions, FALSE if there is a collision
    P_agents = agent_state(1:2,:);
    D = pdist(P_agents'); % pairs of distances between agents
    if min(D) < buff
        chk_pos = false;
    else
        chk_pos = true;
    end
end



% given half-space constraints {A_con, b_con} on the v_peak trajectory 
% parameters space, evaluate a specific v_peak to see if it satisfies the 
% constraints
function chk_FRS = check_agent_FRS_collisions(A_con,b_con,v_peak,c_k,g_k)
% return TRUE if the plan is ok, and FALSE if the plan is NOT ok
    c = inf ;
    lambdas = (v_peak - c_k)./g_k; % given a parameter, get coefficients on k_slc_G generators
    for k = 1:length(A_con) % ============ since initial set is not sliceable w.r.t. inputs
        c_tmp = A_con{k}*lambdas - b_con{k}; % A*lambda - b <= 0 means inside unsafe set
        c_tmp = max(c_tmp); % max of this <= 0 means inside unsafe set
        c = min(c, c_tmp); % take smallest max. if it's <=0, then unsafe
    end
    if c > 0
        chk_FRS = true;
    else
        chk_FRS = false;
    end
    
end