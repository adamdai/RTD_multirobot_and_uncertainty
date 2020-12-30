function [T,Z] = quadrotor_planning_model(v_0,a_0,v_peak,...
                                           t_peak,t_total,t_sample,t_extra)
% [T,Z] = quadrotor_planning_model(v_0,a_0,v_peak,t_peak,...
%                                            t_total,t_sample,t_extra)
%
% This function returns a trajectory represented as a time array T and a
% trajectory array Z. That is, T = 0:t_sample:(t_total + t_extra) and Z \in
% \R^{(5*n_W) x n_T} where n_T = length(T) and n_W is the workspace
% dimension (usually 2-D or 3-D)
%
% The trajectory array format is Z = [X ; V ; A ; J ; S], where X is
% position in \R^{3*n_T} and similarly, V is velocity, A is acceleration, J
% is jerk, and S is snap.
%
% The inputs are: v_0 (initial velocity), a_0 (initial acceleration),
% v_peak (a desired or "peak" velocity) that is reached at time t_peak,
% t_total (total  duration of the trajectory, since each trajectory
% includes a stopping maneuver from t_peak to t_total), t_sample (time
% discretization), and t_extra (extra time that can be added on to the end
% of the trajectory).
%
% This code uses Mark Mueller's analytical solution to quadrotor spline
% planning; we make use of this same spline model for our quadrotor
% trajectory planning, but we compute a reachable set of the splines.
%
% EXAMPLE USAGE:
% % create a random 3-D trajectory of duration 3 s
% [T,Z] = quadrotor_planning_model(rand(3,1),rand(3,1),rand(3,1),1.5,3,0.1)
%
% % plot the trajectory
% figure(1) ; clf ; axis equal ; hold on ; grid on ; view(3)
% plot_path(Z(1:3,:),'b-') % note, this requires the simulator repo
%
% Authors: Patrick Holmes and Shreyas Kousik
% Created: Feb 2019
% Updated: 29 Dec 2020
%
%% ensure inputs are right
    v_0 = v_0(:) ;
    a_0 = a_0(:) ;
    v_peak = v_peak(:) ;
    
    % get workspace dimension
    n_W = length(v_0) ;

%% compute the first part of the spline, up to v_peak
    % time vector
    T_to_peak = 0:t_sample:t_peak ; % assume t_peak/dt is an integer
    
    % desired acceleration at peak speed
    a_peak = zeros(n_W,1) ;
    
    % compute change in velocity/accel for each axis
    delta_v = v_peak - v_0 - a_0.*t_peak ;
    delta_a = a_peak - a_0 ;
    
    % compute spline parameters
    a = nan(n_W,1) ;
    b = nan(n_W,1) ;
    c = nan(n_W,1) ;
    
    for idx = 1:n_W
        [a_idx,b_idx,c_idx] = single_axis_params(delta_v(idx),delta_a(idx),t_peak) ;
        a(idx) = a_idx ;
        b(idx) = b_idx ;
        c(idx) = c_idx ;
    end
    
    % compute spline
    p_0 = zeros(n_W,1) ; % default initial position
    Z_to_peak = make_spline(T_to_peak,p_0,v_0,a_0,a,b,c) ;

%% compute second part of the spline, coming to a stop
    % create time vector for second half
    t_to_stop = t_total - t_peak ;
    T_to_stop = 0:t_sample:t_to_stop ;    

    % desired end speed and acceleration
    v_f = zeros(n_W,1) ;
    a_f = zeros(n_W,1) ;

    % for each axis, compute the change in velocity/accel
    delta_v = v_f - v_peak - a_peak.*t_to_stop ;
    delta_a = a_f - a_peak ;

    % compute spline parameters
    a = nan(n_W,1) ;
    b = nan(n_W,1) ;
    c = nan(n_W,1) ;
    
    for idx = 1:n_W
        [a_idx,b_idx,c_idx] = single_axis_params(delta_v(idx),delta_a(idx),t_peak) ;
        a(idx) = a_idx ;
        b(idx) = b_idx ;
        c(idx) = c_idx ;
    end
    
    % compute spline
    p_peak = Z_to_peak(1:n_W,end) ;
    Z_to_stop = make_spline(T_to_stop,p_peak,v_peak,a_peak,a,b,c) ;

%% connect splines and times
    T = [T_to_peak(1:end-1), T_to_stop + t_peak] ;
    Z = [Z_to_peak(:,1:end-1), Z_to_stop] ;
    
%% add extra time to end of spline if desired
    if nargin > 7 && (t_extra > 0)
            T = [T, T(end) + t_sample, T(end) + t_extra] ;
            Z = [Z, [Z(1:n_W,end);zeros(12,1)], [Z(1:n_W,end);zeros(12,1)]] ;
    end
    
%% compute v and a at t_plan
    if nargout > 2
        z_plan = match_trajectories(t_plan,T_to_peak,Z_to_peak) ;
    end
    
end

function [a,b,c] = single_axis_params(Dv,Da,T)
    M = [0 0 ;
         -12 6*T ;
         6*T -2*T^2] ;
         
    out = (1/T^3)*M*[Dv;Da] ;
    a = out(1) ; b = out(2) ; c = out(3) ;
end

function Z_out = make_spline(T_in,p_0,v_0,a_0,a,b,c)
    % position:
    P = (a/120).*T_in.^5 + (b/24).*T_in.^4 + (c/6).*T_in.^3 + (a_0/2).*T_in.^2 + v_0.*T_in + p_0 ;
    
    % speed:
    V = (a/24).*T_in.^4 + (b/6).*T_in.^3 + (c/2).*T_in.^2 + (a_0).*T_in + v_0 ;
    
    % acceleration:
    A = (a/6).*T_in.^3 + (b/2).*T_in.^2 + (c).*T_in + a_0 ;
    
    % jerk
    J = (a/2).*T_in.^2 + b.*T_in + c ;
    
    % snap
    S = a.*T_in + b ;
    
    % create output traj
    Z_out = [P ; V ; A ; J ; S] ;
end
