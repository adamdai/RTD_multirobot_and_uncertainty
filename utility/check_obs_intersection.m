% Check if 2 FRSes, represented as cell arrays of zonotopes, intersect at 
% any given time step

% Given two FRSes of equal length, 
%  FRS_1 - 
%              {A_1_1,..., A_1_N}, A_i_j where i is agent index and j is
%              time index
%  FRS_2 - 
%              assumes following structure:
%              {A_2_1,..., A_2_N}
%  k_dim - dimensions of the FRS that the trajectory parameters exist in
%  int_dim - dimensions of the FRS to check for intersection in

function is_intersecting = check_obs_intersection(FRS, obs)

    % extract some parameters
    N = length(FRS); % FRS length 
    M = length(obs); % number of obstacles
    
    is_intersecting = false;
    
    % iterate thru timesteps
    for t_idx = 1:N
        
        Z = FRS{t_idx}.Z;
        c = Z(1:2,1); G = Z(1:2,2:end);
        z1 = zonotope([c G]);
        %z1 = conZonotope([c G]);
        for o_idx = 1:M
            
            %z2 = conZonotope(obs{o_idx});
            z2 = obs{o_idx};
            
            %if ~isempty(z1 & z2)
            if in(z1 + (z2 - center(z2)), center(z2))
                is_intersecting = true;
                return
            end
        end
    end

end