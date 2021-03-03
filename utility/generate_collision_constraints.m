% Generate constraints for inter-agent FRS collision checking

% Given two FRSes, (treating the second FRS as an obstacle), generate 
% halfspace constraints in the trajectory parameter space of the first FRS
% that constrain safe trajectory parameters
%  FRS_agent - 
%              {A_1_1,..., A_1_N}, A_i_j where i is agent index and j is
%              time index
%  FRS_other - 
%              assumes following structure:
%              {A_2_1,..., A_2_N, A_3_1,..., A_3_N, ... , A_M_1,...,A_M_N}
%  k_dim - dimensions of the FRS that the trajectory parameters exist in
%  collision_dim - dimensions of the FRS that the obstacles exist in

function [A_con, b_con] = generate_collision_constraints(FRS_agent, FRS_other, k_dim, collision_dim)

    % extract some parameters
    N = length(FRS_agent); % FRS length 
    n_agents = int8(length(FRS_other) / N); % number of other agents, should come out to an int
    assert(isinteger(n_agents),'number of other agents %s, not an integer',num2str(n_agents))

    % initialize cell arrays to hold constraints
    A_con = cell(1,(N-1)*n_agents);
    b_con = cell(1,(N-1)*n_agents);
    
    % skip first timestep since it is not sliceable
    for t_idx = 2:N

        % extract center and generators of FRS_agent for this timestep
        Z = FRS_agent{t_idx}.Z;
        c = Z(collision_dim, 1); 
        G = Z(:, 2:end);

        % for each other agent
        for agent_idx = 1:n_agents
            % find index of FRS_other corresponding to this timestep and
            % other agent to check against
            FRS_other_idx = t_idx + N * (agent_idx - 1);
            % get the zonotope matrix of the other agent FRS at current timestep
            other_agent_zono = FRS_other{FRS_other_idx}.Z;
            % extract the center and generators which exist in the
            % collision dimension - treat the result as a static obstacle
            obs = other_agent_zono(collision_dim,:);
            
            % find "k-sliceable" generators - this forms a linear map from
            % the 
            [~, k_col] = find(G(k_dim, :) ~= 0); 
            k_slc_G = G(collision_dim, k_col);
            
            % "non-k-sliceable" generators
            k_no_slc_G = G(collision_dim, :); 
            k_no_slc_G(:, k_col) = [];

            buff_obstacle_c = obs(:, 1) - c;
            buff_obstacle_G = [obs(:, 2:end), k_no_slc_G]; % obstacle is "buffered" by non-k-sliceable part of FRS
            buff_obstacle_G(:, ~any(buff_obstacle_G)) = []; % delete zero columns of G
            buff_obstacle = [buff_obstacle_c, buff_obstacle_G];
            [A_obs, b_obs] = polytope_PH(buff_obstacle); % turn zonotope into polytope
            
            % constraint indexing 
            idx = agent_idx + n_agents * (t_idx - 2);
            A_con{idx} = A_obs*k_slc_G; % now polytope is over coefficients of k_slc_G generators
            b_con{idx} = b_obs;
        end
    end

end