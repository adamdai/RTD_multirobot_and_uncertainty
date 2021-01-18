% Given an FRS and a set of obstacles, generate halfspace constraints in
% trajectory parameter space that constrain safe trajectory parameters
%  Xrs - FRS
%  obs - cell array of obstacles (represented as zonotope matrices)
%  k_dim - dimensions of the FRS that the trajectory parameters exist in
%  obs_dim - dimensions of the FRS that the obstacles exist in

function [A_con, b_con] = generate_obs_constraints(Xrs, obs, k_dim, obs_dim)

    A_con = {};
    b_con = {};
    
    % skip first time step since it is not sliceable
    for i = 2:length(Xrs)
        % for each obstacle
        for j = 1:length(obs)
            % get current obstacle
            obstacle = obs{j}.Z;
            
            % extract center and generators of FRS 
            Z = Xrs{i}.Z;
            c = Z(obs_dim, 1); 
            G = Z(:, 2:end);
            
            % find "k-sliceable" generators - this forms a linear map from
            % the parameter space to workspace
            [~, k_col] = find(G(k_dim, :) ~= 0); 
            k_slc_G = G(obs_dim, k_col);
            
            % "non-k-sliceable" generators
            k_no_slc_G = G(obs_dim, :); 
            k_no_slc_G(:, k_col) = [];

            buff_obstacle_c = [obstacle(:, 1) - c];
            buff_obstacle_G = [obstacle(:, 2:end), k_no_slc_G]; % obstacle is "buffered" by non-k-sliceable part of FRS
            buff_obstacle_G(:, ~any(buff_obstacle_G)) = []; % delete zero columns of G
            buff_obstacle = [buff_obstacle_c, buff_obstacle_G];
            [A_obs, b_obs] = polytope_PH(buff_obstacle); % turn zonotope into polytope
            
            idx = j + length(obs)*(i - 2);
            A_con{idx} = A_obs*k_slc_G; % now polytope is over coefficients of k_slc_G generators
            b_con{idx} = b_obs;
        end
    end

end