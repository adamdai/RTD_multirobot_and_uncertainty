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

function is_intersecting = check_FRS_intersection(FRS_1, FRS_2, int_dim)

    % extract some parameters
    N = length(FRS_1); % FRS length 
    
    is_intersecting = false;
    
    % iterate thru timesteps
    for t_idx = 1:N
        
        z1 = FRS_1{t_idx};
        z2 = FRS_2{t_idx};
        
        if in(center(z2), z1 + z2)
            is_intersecting = true;
            return
        end
    end

end