%% description
% This script tests out multi-robot planning with reciprocal velocity
% obstacles [1,2]
%
% References
%   [1] Van den Berg, J., Lin, M. and Manocha, D., 2008, May. Reciprocal
%       velocity obstacles for real-time multi-agent navigation. In 2008
%       IEEE International Conference on Robotics and Automation (pp.
%       1928-1935). IEEE.
%
%   [2] Douthwaite, J.A., Zhao, S. and Mihaylova, L.S., 2019. Velocity
%       obstacle approaches for multi-agent collision avoidance. Unmanned
%       Systems, 7(01), pp.55-64.
%
% Authors: Shreyas Kousik
% Created: 1 Feb 2021
% Updated: 2 Feb 2021
clear ; clc
%% user parameters
% plotting parameters
flag_save_gif = false ;
gif_delay_time = 1/20 ; % 1/fps
gif_filename = 'multi_agent_planning.gif' ;

% simulation timing parameters
t_sim_total = 10 ; % [s]
t_sim_sample = 0.1 ; % [s]

% world and obstacle parameters (obstacles are all static for now)
n_dim = 2 ;
world_bounds = 2.*[-1,1,-1,1] ; % 2-D world
n_obs = 0 ;
r_obs_min = 0.1 ; % minimum obstacle radius [m]
r_obs_max = 0.5 ; % maximum obstacle radius [m]
r_goal_reached = 0.3 ; % [m] stop planning when within this dist of goal

% agent parameters
n_agents = 4 ;
r_agents = 0.25 ; % [m]
v_max = 1 ; % [m/s] max allowed velocity (enforced with 2-norm)
delta_v_peak_max = 3 ; % [m/s] max 2-norm change in v_peak allowed between plans

% planning parameters

%% automated from here
% set up simulation timing
T_sim_vec = 0:t_sim_sample:t_sim_total ;
n_t_sim = length(T_sim_vec) ;

%% simulation setup
% set up check for goals reached by the agents and the planners
chk_goal_reached_by_agent = false(1,n_agents) ;
chk_goal_reached_by_plan = false(1,n_agents) ;
           
% initialize indices for all agents
idx_all_agents = 1:n_agents ;
idx_position = 1:n_dim ;
idx_velocity = (n_dim+1):(2*n_dim) ;
idx_acceleration = (2*n_dim+1):(3*n_dim) ;

% generate uniformly-distributed random obstacles
O_ctr = rand_in_bounds(world_bounds,[],[],n_obs) ; % obstacle positions
O_rad = rand_range(r_obs_min,r_obs_max,[],[],1,n_obs) ;

% create random start and goal locations that are feasible to get to
P_start = make_random_feasible_locations(n_agents,r_agents,O_ctr,O_rad,world_bounds) ;
P_goal = make_random_feasible_locations(n_agents,r_agents,O_ctr,O_rad,world_bounds) ;

% initialize the position, velocity, and acceleration of each agent as
% columns of the agent_state array (3*n_dim x n_agents)
agent_state = [P_start ;
               zeros(n_dim,n_agents)] ;
           
% precompute velocity grid to be used for online planning
V_grid = make_grid_2D(v_max*[-1,1,-1,1],100,100) ;
V_grid_speed_log = vecnorm(V_grid) > v_max ;
V_grid(:,V_grid_speed_log) = [] ;
n_V_grid = size(V_grid,2) ;
          
%% plot setup
figure(1) ; clf ; axis equal ; hold on ; grid on ;
axis(1.25.*(world_bounds))

% start positions
plot_path(agent_state(1:n_dim,:),'go','markersize',12,'linewidth',1.5)

% goal areas
for idx = 1:n_agents
    plot_disc(r_goal_reached,P_goal(:,idx),'facecolor','g','edgecolor','g',...
        'facealpha',0.1)
end
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
    
    %disp(['t = ',num2str(t_sim)])
    
    % iterate through the agents to update them for the current time (note
    % that we can randomize the order of this in the future)
    for idx_agent = idx_all_agents
        %% planning (pick a new velocity)
        % get the current agent's state
        p_cur = agent_state(idx_position,idx_agent) ;
        v_cur = agent_state(idx_velocity,idx_agent) ;
        
        % get indices of all other agents
        idx_others = idx_all_agents ;
        idx_others(idx_agent) = [] ;
        
        %% compute velocity obstacles
        % create list of RECIPROCAL velocity obstacles for all the other
        % agents, and for the static obstacles
        VO = [] ;
        
        for idx_other = idx_others
            p_other = agent_state(idx_position,idx_other) ;
            v_other = agent_state(idx_velocity,idx_other) ;
            
            v_RVO = 0.5*(v_cur + v_other) ;
            
            VO_idx = make_velocity_obstacle(p_cur,p_other,v_RVO,r_agents) ;
                
            VO = [VO, VO_idx] ;
        end
        
        v_obs = zeros(n_dim,1) ;
        for idx_obs = 1:n_obs
            p_obs = O_ctr(:,idx_obs) ;
            r_obs = O_rad(idx_obs) ;
            v_RVO = 0.5*(v_cur + v_obs) ;
            
            VO_idx = make_velocity_obstacle(p_cur,p_other,v_RVO,r_agents,r_obs) ;
            VO = [VO, VO_idx] ;
        end
        
        %% compute new velocity for the current agent
        % FOR DEBUGGING: plot all the VOs for the current agent
        % plot_velocity_obstacles(VO,v_max)
        
        % eliminate all velocities in the (R)VO of any other agent or
        % obstacle
        V_log = true(1,n_V_grid) ;
        for idx_VO = 1:length(VO)
            V_in_log = check_points_in_velocity_obstacle(VO(idx_VO),V_grid) ;
            all(V_in_log)
            V_log = V_log & (~V_in_log) ;
        end
        
        if any(V_log)
            V_grid_feas = V_grid(:,V_log) ;
            
            % get the unit direction towards this agent's goal
            u_to_goal = make_unit_length(P_goal(:,idx_agent) - p_cur) ;

            % get the dot product of all the velocities with this unit vector
            cost_of_V_grid = u_to_goal'*V_grid_feas ;
            
            % get the optimal velocity
            [~,idx_max] = max(cost_of_V_grid) ;
            v_new = V_grid_feas(:,idx_max) ;
        else
            v_new = zeros(n_dim,1) ;
        end
        
        %% state update
        % update the state of the current agent using the new velocity
        agent_state(:,idx_agent) = [p_cur + t_sim_sample*v_new ; v_new] ;
        
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