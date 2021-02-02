function in_log = check_points_in_velocity_obstacle(VO,P)
% in_log = check_points_in_velocity_obstacle(VO,P)
%
% Given a 2-D velocity obstacle structure VO and a 2-by-n array of points
% P, return a logical vector in_log of size 1-by-n for which P(:,in_log)
% are the points INSIDE the velocity obstacle
%
% Author: Shreyas Kousik
% Created: 2 Feb 2021
% Updated: nah
    n_P = size(P,2) ;
    V_in_l_log = (VO.HP_l'*(P - repmat(VO.apex,1,n_P))) <= 0 ;
    V_in_r_log = (-VO.HP_r'*(P - repmat(VO.apex,1,n_P))) >= 0 ;
    in_log = V_in_l_log & V_in_r_log ;
end