%% description
% This script tests out asynchronous, decentralized planning for multiple
% agents using a linear planning model.
%
% Authors: Shreyas Kousik
% Created: 5 Jan 2021
% Updated: 7 Jan 2021
clear
%% user parameters
% plotting parameters
flag_save_gif = false ;

% simulation timing parameters
t_sim_total = 5 ; % [s]
t_sim_sample = 0.02 ; % [s]

% world and obstacle parameters (obstacles are all static for now)
n_dim = 2 ;
world_bounds = 5.*[-1,1,-1,1] ; % 2-D world
n_obs = 5 ;
r_obs_min = 0.1 ; % minimum obstacle radius [m]
r_obs_max = 1.0 ; % maximum obstacle radius [m]

% agent parameters (each of these should be a vector of length n_agents)
t_delay_start = 0.3 ; % [s] how long to delay planning start for each agent 
r_agents = 0.5 ; % [m]

% planning parameters
t_plan = [0.1 0.2 0.1] ; % [s] amount of time allotted for planning
v_max = 5 ; % [m/s] max allowed velocity (enforced with 2-norm)
delta_v_peak_max = 3 ; % [m/s] max 2-norm change in v_peak allowed between plans
n_plan_max = 10000 ; % max number of plans to evaluate

%% automated from here
% load the linear planning model
load('quadrotor_linear_planning_model.mat')

% set up simulation timing
T_sim_vec = 0:t_sim_sample:t_sim_total ;
n_t_sim = length(T_sim_vec) ;

%% agent setup
% get the number of agents
n_agents = length(t_plan) ;

% initialize the position, velocity, and acceleration of each agent as
% columns of the agent_state array (3*n_dim x n_agents)
agent_state = [rand_in_bounds(world_bounds,[],[],n_agents) ;
               zeros(2*n_dim,n_agents)] ;

% set up agent timing
t_next_plan = zeros(1,n_agents) ; % each agent will replan at this time
n_t_plan = length(LPM.time) ;

% initialize cells to store the plan history of each agent; the first cell
% row contains the time vector for the current plan, and the second row
% contains the plan itself as a 3-by-n_t_agent list of positions
plans = cell(2,n_agents) ;

% populate the agent plans list with initial plans
for idx_agent = 1:n_agents
    plans{1,idx_agent} = LPM.time ;
    plans{2,idx_agent} = repmat(agent_state(:,idx_agent),1,length(LPM.time)) ;
end

% planning limits
V_bounds = delta_v_peak_max.*repmat([-1,1],1,n_dim) ;

%% world setup
% generate uniformly-distributed random obstacles
O_ctr = rand_in_bounds(world_bounds,[],[],n_obs) ; % obstacle positions
O_rad = rand_range(r_obs_min,r_obs_max,[],[],1,n_obs) ;

% set up obstacle radii for checking
O_rad_mat = repmat(O_rad(:),1,n_t_plan) + r_agents ;

% generate random goal positions
flag_goal_feas = false ;
O_rad_goal_mat = repmat(O_rad(:),1,n_agents) + 2*r_agents ;
while ~flag_goal_feas
    % create random positions
    p_goal = rand_in_bounds(world_bounds,[],[],n_agents) ;
    
    % check position distances to obstacles
    D_goals = dist_points_to_points(O_ctr,p_goal) ;
    
    if ~any(D_goals(:) < O_rad_goal_mat(:))
        break
    end
end

%% plot setup
figure(1) ; clf ; axis equal ; hold on ; grid on ;
axis(1.25.*(world_bounds))

% start positions
plot_path(agent_state(1:n_dim,:),'go','markersize',12,'linewidth',1.5)

% goal positions
plot_path(p_goal,'gp','markersize',12,'markerfacecolor','g')

% obstacles
for idx = 1:n_obs
    plot_disc(O_rad(idx),O_ctr(:,idx),'facecolor','r','edgecolor','r',...
        'facealpha',0.1)
end

% current agent positions
plot_data_agents = [] ;
plot_data_plans = [] ;
for idx_agent = 1:n_agents
    % plot agent
    h_agent = plot_disc(r_agents,agent_state(1:n_dim,idx_agent),...
        'facecolor','b','edgecolor','b','facealpha',0.1) ;
    plot_data_agents = [plot_data_agents, h_agent] ;
    
    % plot plans
    h_plans = plot_path(plans{2,idx_agent}(1:n_dim,:),'-','color',[0.5 0.5 1],'linewidth',1.25) ;
    plot_data_plans = [plot_data_plans, h_plans] ;
end

%% run simulation
for idx = 1:n_t_sim
    % get the current time
    t_sim = T_sim_vec(idx) ;
    
    disp(['t = ',num2str(t_sim)])
    
    % iterate through the agents to update them for the current time (note
    % that we can randomize the order of this in the future)
    for idx_agent = 1:n_agents
        %% setup
        % get the current agent's plan
        T_old = plans{1,idx_agent} ;
        X_old = plans{2,idx_agent} ;
        
        %% planning
        % if the current time is the current agent's next replan time, then
        % that agent needs to replan (duh)
        if t_sim >= t_next_plan(idx_agent)
            disp(['  agent ',num2str(idx_agent),' planning'])
            
            %% planning setup
            % start timer
            tic_start_plan = tic ;
            
            % get the initial condition for the current plan
            x_0 = match_trajectories(t_sim+t_plan,T_old,X_old) ;    
            
            p_0 = x_0(1:n_dim,idx_agent) ;
            v_0 = x_0(n_dim + (1:n_dim),idx_agent) ;
            a_0 = x_0(2*n_dim + (1:n_dim),idx_agent) ;
            
            % create the time vector for the new plan (to start at t_plan)
            T_new = LPM.time + t_sim + t_plan(idx_agent) ;
            
            % get the current plans of all other agents, and buffer them by
            % the amount of time extra that is needed to cover the duration
            % of our current plan
            idx_other_agents = 1:n_agents ;
            idx_other_agents(idx_agent) = [] ; % indices of other agents
            
            P_other = [] ;
            for idx_other = idx_other_agents
                % get the time and plan
                T_idx = plans{1,idx_other} ;
                P_idx = plans{2,idx_other}(1:n_dim,:) ;
                
                % get all the parts of the plan that are relevant to the
                % current agent's new plan
                P_idx = match_trajectories(T_new,T_idx,P_idx) ;
                
                % add the new plan to the old plan
                P_other = [P_other, P_idx] ; % should preallocate this
            end
            
            %% find new plan
            % get a bunch of random potential v_peaks
            V_peak = rand_in_bounds(V_bounds,[],[],n_plan_max) ;
            V_peak_mag = vecnorm(V_peak) ;
            delta_V_peak_mag = vecnorm(V_peak - repmat(v_0,1,n_plan_max)) ;
            V_peak_test = (V_peak_mag > v_max | delta_V_peak_mag > delta_v_peak_max) ;
            V_peak(:,V_peak_test) = [] ;
            n_V_peak = size(V_peak,2) ;
            
            % get the final positions for each of these v_peaks
            LPM_p_final = LPM.position(:,end) ;
            p_from_v_and_a_0 = [v_0, a_0] * LPM_p_final(1:2) + p_0 ;
            p_from_v_peak = LPM_p_final(3) * V_peak + repmat(p_from_v_and_a_0,1,n_V_peak) ;
            
            % sort these V_peaks by their suboptimality in terms of
            % distance of each final position to the global goal
            dist_to_goal = vecnorm(p_from_v_peak - repmat(p_goal(:,idx_agent),1,n_V_peak)) ;
            [~,V_sort_idxs] = sort(dist_to_goal,'ascend') ;
            V_peak = V_peak(:,V_sort_idxs) ;
            
            % iterate through the V_peaks until one is feasible
            idx_v_peak = 1 ;
            flag_v_peak_infeas = true ; % pessimism
            while ((idx_v_peak <= n_V_peak) && ...
                   (toc(tic_start_plan) < t_plan(idx_agent)) && ...
                   flag_v_peak_infeas)
                
                % get the position trajectory for the current v_peak
                v_peak = V_peak(:,idx_v_peak) ;
                P_idx = [v_0, a_0, v_peak]*LPM.position + repmat(p_0,1,n_t_plan) ;
                
                % check against the other plans
                if ~isempty(P_other)
                    dist_to_others = vecnorm(P_other - repmat(P_idx,1,n_agents-1));
                    chk_others = any(dist_to_others <= 2*r_agents) ;
                else
                    chk_others = false ;
                end
                
                % check against obstacles
                if ~isempty(O_ctr)
                    D_obs = dist_points_to_points(O_ctr,P_idx) ;
                    chk_obs = any(D_obs(:) < O_rad_mat(:)) ;
                else
                    chk_obs = false ;
                end
                
                % decide feasibility
                if ~chk_others && ~chk_obs
                    flag_v_peak_infeas = false ;
                end
                
                % increment index
                idx_v_peak = idx_v_peak + 1 ;
            end
            
            if flag_v_peak_infeas
               % if no new plan was found, continue the previous plan
               disp('    found no new plan')
               
               T_log = T_old >= t_sim ;
               
               % also, increase the length of the old plan by t_plan
               T_new = [T_old(T_log), T_old(end) + t_plan(idx_agent)] ;
               X_new = [X_old(:,T_log), X_old(:,end)] ;              
            else
               % otherwise, create a new plan
               disp('    found new plan')
               
               X_new = [[v_0, a_0, v_peak]*LPM.position + repmat(p_0,1,n_t_plan) ;
                        [v_0, a_0, v_peak]*LPM.velocity ;
                        [v_0, a_0, v_peak]*LPM.acceleration] ;
            end
            
            %% planning wrap up
            % append the previous trajectory to the new trajectory
            T_old_log = T_old < T_new(1) ;
            
            if isempty(T_new)
                dbstop in testing_LPM_multi_agent_planning at 257
                disp('hi')
            end
            
            T_new = [T_old(T_old_log), T_new] ;
            X_new = [X_old(:,T_old_log), X_new] ;
            
            % update plans object
            plans{1,idx_agent} = T_new ;
            plans{2,idx_agent} = X_new ;
            
            % set the time at which to begin checking the current plan and
            % to begin replanning
            t_next_plan(idx_agent) = t_next_plan(idx_agent) + t_plan(idx_agent) ;
        
            % figure out how much time was spent
            t_plan_spent = toc(tic_start_plan) ;
            
            %% checking
            
            %% plotting plans
            % plot new plan
            plot_data_plans(idx_agent).XData = X_new(1,:) ;
            plot_data_plans(idx_agent).YData = X_new(2,:) ;
            
            % for 3-D
        if n_dim == 3
            plot_data_plans(idx_agent).ZData = X_new(3,:) ;
        end
        end
        
        %% state update
        % finally, update the state of each agent (for now assuming perfect
        % tracking behavior)
        agent_state(:,idx_agent) = match_trajectories(t_sim,T_old,X_old) ;
        
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
    if flag_save_gif
        error('ope')
    else
        pause(t_sim_sample)
    end
end

%% helper functions