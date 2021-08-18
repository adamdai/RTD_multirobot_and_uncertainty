%% description
% This script computes a collection of linear operators (matrices!) that
% represent our polynomial spline planning model for the quadrotor as a
% sequence of axis-aligned boxes. This enables one to plug in arbitrary
% initial and desired velocities and accelerations and generate the
% corresponding spline and boxes (represented as zonotopes in terms of
% centers and halfwidths).
%
% Authors: Shreyas Kousik
% Created: 29 Dec 2020
% Updated: not yet
%
%% user parameters
% timing
t_peak = 1.5 ;
t_sample = 0.1 ;
t_total = 3 ;

% whether or not to save the PRS to ./quadrotor_PRS_analytical.mat 
flag_save_PRS = false ;

% test trajectory parameters for plotting (works for 1-, 2-, or 3-D)
flag_test_PRS = true ; % set to false if you just want to make the PRS
v_0_test = [2; -3; 3] ;
a_0_test = [2; -1; 2] ;
v_peak_test = [-1; -5; 4] ;

%% automated from here
% create symbolic parameters
syms v_0 a_0 v_peak t ;

% set additional parameters
t_to_stop = t_total - t_peak ; % time from peak to stop
a_peak = 0 ; % 0 acceleration at peak speed
v_f = 0 ; % 0 final speed
a_f = 0 ; % 0 final acceleration.

%% compute "to peak" part of trajectory symbolically
% compute change in velocity/accel
delta_v_to_peak = v_peak - v_0 - a_0*t_peak ;
delta_a_to_peak = a_peak - a_0 ;

% compute spline parameters (the planning model is represented by
% polynomial splines)
[a_to_peak, b_to_peak, c_to_peak] = get_single_axis_params(delta_v_to_peak,delta_a_to_peak,t_peak) ;

% compute symbolic planning model position and velocity
p_sym_to_peak = make_single_axis_spline(a_to_peak,b_to_peak,c_to_peak,a_0,v_0,t) ;
v_sym_to_peak = diff(p_sym_to_peak,t) ;
a_sym_to_peak = diff(v_sym_to_peak,t) ;

%% compute "to stop" part of trajectory symbolically
% change in velocity/accel
delta_v_to_stop = v_f - v_peak - a_peak*t_to_stop;
delta_a_to_stop = a_f - a_peak ;

% compute spline parameters
[a_to_stop, b_to_stop, c_to_stop] = get_single_axis_params(delta_v_to_stop,delta_a_to_stop,t_to_stop) ;

% compute symbolic planning model position
p_sym_to_stop = make_single_axis_spline(a_to_stop,b_to_stop,c_to_stop,a_peak,v_peak,t) ;

%% create linear function for position spline
% create vector of sample times
t_vec_to_peak = 0:t_sample:t_peak ;
t_vec_to_stop = 0:t_sample:t_to_stop ;

% make sure the end times are in the time vectors
if t_vec_to_peak(end) < t_peak
    t_vec_to_peak = [t_vec_to_peak, t_peak] ;
end

if t_vec_to_stop(end) < t_to_stop
    t_vec_to_stop = [t_vec_to_stop, t_to_stop] ;
end

% create spline at each point in time
p_sym_to_peak_vec = subs(p_sym_to_peak,t,t_vec_to_peak) ;
p_sym_to_stop_vec = subs(p_sym_to_stop,t,t_vec_to_stop) ;
p_sym = [p_sym_to_peak_vec, p_sym_to_peak_vec(end) + p_sym_to_stop_vec(2:end)] ;

% break spline up into vectors for each parameter
s_vars = [v_0,a_0,v_peak] ;
v_0_vec = double(subs(p_sym,s_vars,[1 0 0])) ;
a_0_vec = double(subs(p_sym,s_vars,[0 1 0])) ;
v_peak_vec = double(subs(p_sym,s_vars,[0 0 1])) ;
p_mat = [v_0_vec ; a_0_vec ; v_peak_vec] ; % this is the linear function

%% create linear functions for bounding spline
% the "trick" here is to use finite differencing on the points given by the
% position spline (represented with p_mat) to get the centers and widths of
% intervals that bound the positions traversed in each time interval of the
% total time [0, t_total] partitioned by t_sample

% timing
T_p = t_sample.*(0:size(p_mat,2)-1) ;
T_c = 0.5.*(T_p(2:end) + T_p(1:end-1)) ;

% centers
c_sym_k = 0.5.*(p_sym(2:end) + p_sym(1:(end-1))) ;
v_0_c = double(subs(c_sym_k,s_vars,[1 0 0])) ;
a_0_c = double(subs(c_sym_k,s_vars,[0 1 0])) ;
v_peak_c = double(subs(c_sym_k,s_vars,[0 0 1])) ;
c_mat = [v_0_c ; a_0_c ; v_peak_c] ; % linear func for centers

% widths
w_sym_k = 0.5.*(p_sym(2:end) - p_sym(1:(end-1))) ;
v_0_w = double(subs(w_sym_k,s_vars,[1 0 0])) ;
a_0_w = double(subs(w_sym_k,s_vars,[0 1 0])) ;
v_peak_w = double(subs(w_sym_k,s_vars,[0 0 1])) ;
w_mat = [v_0_w ; a_0_w ; v_peak_w] ; % linear func for widths

%% save output
% create structure to save
PRS.t_peak = t_peak ;
PRS.t_total = t_total ;
PRS.t_sample = t_sample ;
PRS.T_positions = T_p ;
PRS.T_centers = T_c ;
PRS.positions = p_mat ;
PRS.centers = c_mat ;
PRS.widths = w_mat ;
PRS.n_T = size(c_mat,2) ;

% save!
if flag_save_PRS
    filename = 'quadrotor_PRS_analytical.mat' ;
    save(filename,'PRS')
end

%% plot PRS
if flag_test_PRS
    %% set up parameter values of the appropriate dimension
    K = [v_0_test(:), a_0_test(:), v_peak_test(:)] ;
    
    % get positions, centers, and widths
    P = K * p_mat ;
    C = K * c_mat ;
    W = K * w_mat ;
    
    % get dimension and number of boxes to plot
    n_dim = size(K,1) ;
    n_box = PRS.n_T ;
    
    % generate the spline at a high-ish resolution
    [T_fine,Z_fine] = quadrotor_planning_model(v_0_test,a_0_test,v_peak_test,...
                                           t_peak,t_total,0.01) ;
                                       
    P_fine = Z_fine(1:n_dim,:) ;
    p_peak = match_trajectories(t_peak,T_fine,P_fine) ;

    %% generate different plots depending on the dimension of the input
    % setup figure
    figure(1) ; clf ; hold on ; grid on ;
    
    % plot!
    switch n_dim
        case 1
            % plot the start and end of the trajectory
            h_start = plot(0,P(1),'go','linewidth',1.5,...
                'markersize',10,'markerfacecolor','g') ;
            
            h_end = plot(t_total,P(end),'rp','linewidth',1.5,...
                'markersize',12,'markerfacecolor','r') ;
            
            % plot where the peak velocity occurs (approximately)
            h_peak = plot(t_peak,p_peak,'bp','linewidth',1.5,...
                'markersize',12,'markerfacecolor','b') ;
            
            % plot the (time-varying) bounding boxes
            for idx = 1:n_box
                V = make_box([t_sample,2*W(idx)],[T_c(idx) ; C(idx)]) ;
                h_box = patch('faces',[1 2 3 4],'vertices',V(:,1:4)',...
                    'EdgeColor','b','FaceColor','b','FaceAlpha',0.1) ;
            end
            
            % plot the position trajectory
            h_pos = plot(T_fine,P_fine,'b-','linewidth',1.5) ;
            plot(T_p,P,'b.','markersize',12) ;
            
            % labels
            xlabel('time [s]')
            ylabel('position [m]')
            title('1-D planning model trajectory')
        case 2
            axis equal ;
            
            % plot the start and end of the trajectory
            h_start = plot_path(P(:,1),'go','linewidth',1.5,...
                'markersize',10,'markerfacecolor','g') ;
            h_end = plot_path(P(:,end),'rp','linewidth',1.5,...
                'markersize',12,'markerfacecolor','r') ;
            
            % plot where the peak velocity occurs (approximately)
            h_peak = plot_path(p_peak,'bp','linewidth',1.5,...
                'markersize',12,'markerfacecolor','b') ;
            
            % plot the time-varying bounding boxes
            for idx = 1:n_box
                V = make_box([2*W(1,idx),2*W(2,idx)],C(:,idx)) ;
                h_box = patch('faces',[1 2 3 4],'vertices',V(:,1:4)',...
                    'EdgeColor','b','FaceColor','b','FaceAlpha',0.1) ;
            end
            
            % plot the position trajectory
            h_pos = plot_path(P_fine,'b-','linewidth',1.5) ;
            plot_path(P,'b.','markersize',12) ;
            
            % labels
            xlabel('p_1 [m]')
            ylabel('p_2 [m]')
            title('2-D planning model trajectory')            
        case 3
            axis equal ;
            
            % plot the start and end of the trajectory
            h_start = plot_path(P(:,1),'go','linewidth',1.5,...
                'markersize',10,'markerfacecolor','g') ;
            h_end = plot_path(P(:,end),'rp','linewidth',1.5,...
                'markersize',12,'markerfacecolor','r') ;
            
            % plot where the peak velocity occurs (approximately)
            h_peak = plot_path(p_peak,'bp','linewidth',1.5,...
                'markersize',12,'markerfacecolor','b') ;
            
            % plot the time-varying bounding boxes
            for idx = 1:n_box
                FV = make_cuboid_for_patch(2*W(1,idx),2*W(2,idx),2*W(3,idx),...
                    C(:,idx)) ;
                h_box = patch(FV,'EdgeColor','b','FaceColor','b','FaceAlpha',0.1) ;
            end
            
            % plot the position trajectory
            h_pos = plot_path(P_fine,'b-','linewidth',1.5) ;
            plot_path(P,'b.','markersize',12) ;
            
            % labels
            xlabel('p_1 [m]')
            ylabel('p_2 [m]')
            zlabel('p_3 [m]')
            title('3-D planning model trajectory')
            
            view(3)
            
        otherwise
            error('Please pick 1-D, 2-D, or 3-D trajectory parameters.')
    end
    
    % finalize plot
    legend([h_box,h_pos,h_start,h_end,h_peak],...
                'zono','traj','start','end','peak')
    set(gca,'fontsize',15)
end

%% helper functions
function [a,b,c] = get_single_axis_params(delta_v,delta_a,t_final)
    M = [0 0 ;
         -12 6*t_final ;
         6*t_final -2*t_final^2] ;
         
    out = (1/t_final^3)*M*[delta_v ; delta_a] ;
    a = out(1) ;
    b = out(2) ;
    c = out(3) ;
end

function p = make_single_axis_spline(a,b,c,a_0,v_0,t)
    p = (a/120).*t.^5 + (b/24).*t.^4 + (c/6).*t.^3 + (a_0/2).*t.^2 + v_0*t ;
end