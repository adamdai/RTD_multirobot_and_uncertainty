function VO = make_velocity_obstacle(p_i,p_j,v_j,r_i,r_j,v_max)
% VO = make_velocity_obstacle(p_i,p_j,v_j,r_i,r_j)
%
% Based on this paper: https://shiyuzhao.westlake.edu.cn/3.pdf
%
% See Fig. 2 in particular!
%
% Authors: Shreyas Kousik
% Created: 1 Feb 2021
% Updated: 2 Feb 2021

    if ~exist('r_j','var')
        r_j = r_i ;
    end

    if ~exist('v_max','var')
        v_max = 5 ;
    end
        
    % get radius of augmented robot
    r_c = r_i + r_j ;
    
    % get the vector from the center of robot i to the center of the
    % augmented robot j
    lambda_ij = p_j - p_i ;
    d_ij = vecnorm(lambda_ij) ;
    u_ij = lambda_ij / d_ij ;
    
    % get the VO cone half-angle
    ha = real(asin(r_c / d_ij)) ;
    
    % rotate the unit vector by plus/minus (90º + ha) to create unit
    % normals to the two planes defining the VO cone
    HP_l = rotation_matrix_2D(+pi/2 + ha) * u_ij ;
    HP_r = rotation_matrix_2D(-pi/2 - ha) * u_ij ;
    
    % get the left and right tangent vectors (these are useful for
    % plotting the velocity obstacle)
    t_l = rotation_matrix_2D(+ha) * u_ij ;
    t_r = rotation_matrix_2D(-ha) * u_ij ;
    
    % make faces and vertices for plotting
    faces = [1 2 3 1] ;
    verts = [v_j,...
           v_j + 10*v_max.*t_l,...
           v_j + 10*v_max.*t_r]' ;

    
    % create the output structure
    VO.apex = v_j ;
    VO.HP_l = HP_l ;
    VO.HP_r = HP_r ;
    VO.t_l = t_l ;
    VO.t_r = t_r ;
    VO.faces = faces ;
    VO.vertices = verts ;    
end