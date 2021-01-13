%% description
% This script tests out asynchronous, decentralized planning for multiple
% agents using a linear planning model.
%
% Authors: Shreyas Kousik
% Created:  5 Jan 2021
% Updated: 11 Jan 2021
clear ; clc
%% user parameters
% plotting parameters
flag_save_gif = false ;
gif_delay_time = 1/20 ; % 1/fps
gif_filename = 'multi_agent_planning.gif' ;

% simulation timing parameters
t_sim_total = 5 ; % [s]
t_sim_sample = 0.01 ; % [s]

% world and obstacle parameters (obstacles are all static for now)
n_dim = 2 ;
world_bounds = 4.*[-1,1,-1,1] ; % 2-D world
n_obs = 5 ;
r_obs_min = 0.1 ; % minimum obstacle radius [m]
r_obs_max = 0.5 ; % maximum obstacle radius [m]
r_goal_reached = 0.1 ; % [m] stop planning when within this dist of goal

% agent parameters
r_agents = 0.5 ; % [m]
v_max = 5 ; % [m/s] max allowed velocity (enforced with 2-norm)
delta_v_peak_max = 3 ; % [m/s] max 2-norm change in v_peak allowed between plans

% planning parameters -- note that the length of t_plan, t_check, and
% t_recheck should be the same, and equal to the number of agents that you
% want to tootle around
t_plan = [0.1 0.2 0.1 0.5] ; % [s] amount of time allotted for planning
t_check = [0.01 0.01 0.01 0.02] ; % [s] amount of time allotted for check
t_recheck = [0.01 0.01 0.01 0.01] ; % [s] amount of time allotted for recheck
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

% set up agent timing
t_next_plan = zeros(1,n_agents) ; % each agent will replan at this time
t_next_check = t_plan ; % each agent will check its plan at this time
t_next_recheck = t_plan + t_check ;
t_committed = zeros(1,n_agents) ;
n_t_plan = length(LPM.time) ;

% set up check for goal reached
flag_goal_reached = false(1,n_agents) ;

%% world setup
% generate uniformly-distributed random obstacles
O_ctr = rand_in_bounds(world_bounds,[],[],n_obs) ; % obstacle positions
O_rad = rand_range(r_obs_min,r_obs_max,[],[],1,n_obs) ;

% set up obstacle radii, buffered by agent radii, so that collision
% checking is a little bit faster; by using this, we can treat the agents
% and obstacles as point masses
O_rad_mat = repmat(O_rad(:),1,n_t_plan) + r_agents ;

% create random start and goal locations that are feasible to get to
P_start = make_random_feasible_locations(n_agents,r_agents,O_ctr,O_rad,world_bounds) ;
P_goal = make_random_feasible_locations(n_agents,r_agents,O_ctr,O_rad,world_bounds) ;

%% initialize agents and plans
% initialize the position, velocity, and acceleration of each agent as
% columns of the agent_state array (3*n_dim x n_agents)
agent_state = [P_start ;
               zeros(2*n_dim,n_agents)] ;
           
% initialize cells to store the plan history of each agent; the first cell
% row contains the time vector for the current plan, and the second row
% contains the plan itself as a 3-by-n_t_agent list of positions
plans_committed = cell(2,n_agents) ;
plans_pending = cell(2,n_agents) ;

% planning limits
V_bounds = delta_v_peak_max.*repmat([-1,1],1,n_dim) ;

% populate the agent plans list with initial plans where they stay in place
for idx_agent = 1:n_agents
    plans_committed{1,idx_agent} = LPM.time ;
    plans_committed{2,idx_agent} = repmat(agent_state(:,idx_agent),1,length(LPM.time)) ;
end
           
%% plot setup
figure(1) ; clf ; axis equal ; hold on ; grid on ;
axis(1.25.*(world_bounds))

% start positions
plot_path(agent_state(1:n_dim,:),'go','markersize',12,'linewidth',1.5)

% goal positions
plot_path(P_goal,'gp','markersize',12,'markerfacecolor','g')

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
    h_plans = plot_path(plans_committed{2,idx_agent}(1:n_dim,:),'-','color',[0.5 0.5 1],'linewidth',1.25) ;
    plot_data_plans = [plot_data_plans, h_plans] ;
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
    
    % iterate through the agents to update them for the current time (note
    % that we can randomize the order of this in the future)
    for idx_agent = 1:n_agents
        %% setup
        % get the current agent's plan
        T_old = plans_committed{1,idx_agent} ;
        X_old = plans_committed{2,idx_agent} ;
        
        if t_sim >= t_next_plan(idx_agent) && (~flag_goal_reached(idx_agent))
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
            
            % create the time vector for the new plan (to start at t_plan)
            T_new = LPM.time + t_sim + t_plan(idx_agent) ;
            
            % get the current plans of all other agents, and buffer them by
            % the amount of time extra that is needed to cover the duration
            % of our current plan
            P_other = get_other_agents_committed_plans(n_agents,...
                idx_agent,plans_committed,n_dim,T_new) ;
            
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
            while ((idx_v_peak <= n_V_peak) && ...
                   (toc(tic_start_plan) < t_plan(idx_agent)))
                
                % get the position trajectory for the current v_peak
                v_peak = V_peak(:,idx_v_peak) ;
                P_idx = [v_0, a_0, v_peak]*LPM.position + repmat(p_0,1,n_t_plan) ;
                
                % check against the other plans
                chk_others = check_agent_plan_collisions(P_other,P_idx,n_agents,r_agents) ;
                
                % check against obstacles
                if ~isempty(O_ctr)
                    D_obs = dist_points_to_points(O_ctr,P_idx) ;
                    chk_obs = ~any(D_obs(:) < O_rad_mat(:)) ;
                else
                    chk_obs = false ;
                end
                
                % stop optimizing if found a feasible plan
                if chk_others && chk_obs
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
                    
               
               % check if the global goal has been reached
               if vecnorm(X_new(1:n_dim,end) - P_goal(:,idx_agent)) < r_goal_reached
                   flag_goal_reached(idx_agent) = true ;
               end
            end           
            
            %% planning wrap up
            % append the previous trajectory to the new trajectory
            T_old_log = T_old < T_new(1) ;
            T_new = [T_old(T_old_log), T_new] ;
            X_new = [X_old(:,T_old_log), X_new] ;
            
            % update plans pending
            plans_pending{1,idx_agent} = T_new ;
            plans_pending{2,idx_agent} = X_new ;
            
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
        %% checking
        elseif t_sim >= t_next_check(idx_agent)
            disp(['  agent ',num2str(idx_agent),' checking'])
            
            tic_start_check = tic ;
            
            % get the pending plan
            T_pend = plans_pending{1,idx_agent} ;
            X_pend = plans_pending{2,idx_agent} ;
            
            if ~isempty(X_pend)
                % get the position coordinates of the plan
                P_pend = X_pend(1:n_dim,:) ;
                
                % get the committed trajectories of all other agents
                P_other = get_other_agents_committed_plans(n_agents,...
                    idx_agent,plans_committed,n_dim,T_pend) ;
                
                % check against the other plans
                chk_others = check_agent_plan_collisions(P_other,P_pend,...
                    n_agents,r_agents) ;
                
                % make sure we didn't take too much time
                chk_time = toc(tic_start_check) < t_check(idx_agent) ;
                 
                if chk_others
                    % if we passed the check, do nothing and move on to the
                    % recheck
                    disp('    new plan check ok')
                else
                    % if the agent failed the check, discard its pending plan
                    plans_pending{1,idx_agent} = [] ;
                    plans_pending{2,idx_agent} = [] ;
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
            T_pend = plans_pending{1,idx_agent} ;
            X_pend = plans_pending{2,idx_agent} ;
            
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
                plans_committed{1,idx_agent} = T_pend ; % time
                plans_committed{2,idx_agent} = X_pend ; % traj
                t_committed(idx_agent) = t_sim ;
            end
            
            % mark this recheck as complete by setting its time to inf
            t_next_recheck(idx_agent) = inf ;
            
            t_spent_recheck(idx) = toc(tic_start_recheck) ;
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
end

toc(tic_real_time)
%% helper functions
function P_out = make_random_feasible_locations(n_agents,r_agents,O_ctr,O_rad,world_bounds)
    flag_P_feas = false ;
    O_rad_mat = repmat(O_rad(:),1,n_agents) + 2*r_agents ;
    while ~flag_P_feas
        % create random positions
        P_out = rand_in_bounds(world_bounds,[],[],n_agents) ;

        % check position distances to obstacles
        D_to_obs = dist_points_to_points(O_ctr,P_out) ;
        
        % check position distances to each other
        D_to_self = dist_points_to_points(P_out,P_out) ;
        D_to_self(D_to_self == 0) = nan ; % ignore same points

        if ~any(D_to_obs(:) < O_rad_mat(:)) && ~any(D_to_self(:) < 2*r_agents)
            break
        end
    end
end

function P_other = get_other_agents_committed_plans(n_agents,idx_agent,plans_committed,n_dim,T_new)
    idx_other_agents = 1:n_agents ;
    idx_other_agents(idx_agent) = [] ; % indices of other agents

    P_other = [] ;
    for idx_other = idx_other_agents
        % get the time and plan
        T_idx = plans_committed{1,idx_other} ;
        P_idx = plans_committed{2,idx_other}(1:n_dim,:) ;

        % get all the parts of the plan that are relevant to the
        % current agent's new plan
        P_idx = match_trajectories(T_new,T_idx,P_idx) ;

        % add the new plan to the old plan
        P_other = [P_other, P_idx] ; % should preallocate this
    end
end

function chk_others = check_agent_plan_collisions(P_other,P_agent,n_agents,r_agents)
% return TRUE if the plan is ok, and FALSE if the plan is NOT ok
    if ~isempty(P_other)
        dist_to_others = vecnorm(P_other - repmat(P_agent,1,n_agents-1));
        chk_others = ~any(dist_to_others <= 2*r_agents) ;
    else
        chk_others = false ;
    end
end