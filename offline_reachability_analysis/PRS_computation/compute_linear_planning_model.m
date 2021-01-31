%% description
% This script computes a collection of linear operators that represent the
% discrete-time linear planning model (LPM). This model uses trajectory
% parameters that cause the planning model to accelerate from an initial
% velocity v_0 and acceleration a_0 to a "peak" velocity v_peak at a time
% t_peak, then accelerate to a final acceleration and velocity of 0 at time
% t_total. The time discretization is pre-specified as t_sample.
%
% This script saves the LPM as a structure to a file named
%
%   quadrotor_linear_planning_model.mat
%
% The LPM structure has fields for the constants t_peak, t_total, and
% t_sample. It also contains the position, velocity, and acceleration as
% matrices representing the LPM. Suppose that k \in K is a given trajectory
% parameter. Then the positions at the times in LPM.time are given by
%
%   P = k' * LPM.position,
%
% and similarly for the velocities and accelerations. To see how these
% work, change the trajectory parameters in v_0_test, a_0_test, and
% v_peak_test (which can be 1-D, 2-D, or 3-D, as long as they are all of
% the same dimension).
%
% Authors: Shreyas Kousik
% Created: 5 Jan 2021
% Updated: 6 Jan 2021
%
%% user parameters
% timing
t_peak = 1.5 ;
t_sample = 0.1 ;
t_total = 3 ;

% whether or not to save the mode to ./quadrotor_linear_planning_model.mat 
flag_save_LPM = false ;

% test trajectory parameters for plotting (works for 1-, 2-, or 3-D)
flag_test_LPM = true ; % set to false if you just want to make the PRS
v_0_test = [2 1];
a_0_test = [-5 0];
v_peak_test = [3 1];

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
v_sym_to_stop = diff(p_sym_to_stop,t) ;
a_sym_to_stop = diff(v_sym_to_stop,t) ;

% add position to peak for stopping trajectory
p_sym_to_stop = p_sym_to_stop + subs(p_sym_to_peak,t,t_peak) ; 

%% create linear function for position, velocity, and acceleration
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

% create position, velocity, and acceleration splines at each point in time
p_sym = eval_spline_at_times(p_sym_to_peak,p_sym_to_stop,t_vec_to_peak,t_vec_to_stop,t) ;
v_sym = eval_spline_at_times(v_sym_to_peak,v_sym_to_stop,t_vec_to_peak,t_vec_to_stop,t) ;
a_sym = eval_spline_at_times(a_sym_to_peak,a_sym_to_stop,t_vec_to_peak,t_vec_to_stop,t) ;

% collect symbolic trajectory parameters
k_sym = [v_0,a_0,v_peak] ;

% get the linear operator representations for the splines
p_mat = make_linear_spline_model(p_sym,k_sym) ;
v_mat = make_linear_spline_model(v_sym,k_sym) ;
a_mat = make_linear_spline_model(a_sym,k_sym) ;

% get the time vector for these splines
T = [t_vec_to_peak, t_vec_to_stop(2:end) + t_peak] ;

%% save output
% create structure to save
LPM.t_peak = t_peak ;
LPM.t_total = t_total ;
LPM.t_sample = t_sample ;
LPM.time = T ;
LPM.position = p_mat ;
LPM.velocity = v_mat ;
LPM.acceleration = a_mat ;

% save!
if flag_save_LPM
    filename = 'quadrotor_linear_planning_model.mat' ;
    
    disp('Saving linear planning model!')
    save(filename,'LPM')
end

%% plot
if flag_test_LPM
    %% set up parameter values of the appropriate dimension
    K = [v_0_test(:), a_0_test(:), v_peak_test(:)] ;
    
    % get positions, velocities, and accelerations
    P = K * p_mat ;
    V = K * v_mat ;
    A = K * a_mat ;
    
    % generate the spline at a high-ish resolution
    [T_fine,Z_fine] = quadrotor_planning_model(v_0_test,a_0_test,v_peak_test,...
                                           t_peak,t_total,0.01) ;
                                       
    n_dim = length(v_0_test) ;
                                       
    P_fine = Z_fine(1:n_dim,:) ;
    p_peak = match_trajectories(t_peak,T_fine,P_fine) ;
    
    % numerically integrate the velocities and accelerations to get the
    % position trajectory
    P_num = [zeros(n_dim,1), cumsum(t_sample.*V(:,1:end-1) + (t_sample^2/2).*A(:,1:end-1),2)] ;
    V_num = [v_0_test(:), cumsum(t_sample.*A(:,1:end-1),2) + v_0_test(:)] ;
    
    %% generate different plots depending on the dimension of the input
    % setup figure
    figure(1) ; clf ; hold on ; grid on ;
    
    % plot!
    if n_dim == 1
            % plot the positions
            subplot(3,1,1) ; hold on ; grid on ;
            
            % plot the start and end of the trajectory
            h_start = plot(0,P(1),'go','linewidth',1.5,...
                'markersize',10,'markerfacecolor','g') ;
            
            h_end = plot(t_total,P(end),'rp','linewidth',1.5,...
                'markersize',12,'markerfacecolor','r') ;
            
            % plot where the peak velocity occurs (approximately)
            h_peak = plot(t_peak,p_peak,'bp','linewidth',1.5,...
                'markersize',12,'markerfacecolor','b') ;
            
            % plot the position trajectory given by the linear operator
            h_pos = plot(T_fine,P_fine,'-','color',[0.7 0.7 1.0],'linewidth',1.5) ;
            plot(T,P,'b.','markersize',12) ;
            
            % plot the position trajectory given by numerical integration
            h_num = plot(T,P_num,'r.','markersize',7) ;
            
            % finalize plot
            ylabel('pos [m]')
            title('1-D planning model trajectory')
            
            legend([h_pos,h_start,h_end,h_peak,h_num],...
                'traj','start','end','peak','num',...
                'location','northwest')
            
            set(gca,'fontsize',15)
            
            % plot velocities
            subplot(3,1,2) ; hold on ; grid on ;
            plot(T,V,'b.','markersize',12) ;
            plot(T,V_num,'r.','markersize',7) ;
            ylabel('vel [m/s]') 
            legend('lin','num','location','northwest')
            set(gca,'fontsize',15)
            
            % plot accelerations
            subplot(3,1,3) ; hold on ; grid on ;
            plot(T,A,'b.','markersize',12) ;
            xlabel('time [s]')
            ylabel('accel [m/s^2]') 
            set(gca,'fontsize',15)
    else
            axis equal ;
            
            % plot the start and end of the trajectory
            h_start = plot_path(P(:,1),'go','linewidth',1.5,...
                'markersize',10,'markerfacecolor','g') ;
            h_end = plot_path(P(:,end),'rp','linewidth',1.5,...
                'markersize',12,'markerfacecolor','r') ;
            
            % plot where the peak velocity occurs (approximately)
            h_peak = plot_path(p_peak,'bp','linewidth',1.5,...
                'markersize',12,'markerfacecolor','b') ;
            
            % plot the position trajectory
            h_pos = plot_path(P_fine,'-','color',[0.7 0.7 1.0],'linewidth',1.5) ;
            plot_path(P,'b.','markersize',12) ;
            
            % plot the position trajectory given by numerical integration
            h_num = plot_path(P_num,'r.','markersize',5) ;
            
            % labels
            xlabel('p_1 [m]')
            ylabel('p_2 [m]')
            
            if n_dim == 2
                title('2-D planning model trajectory')
            else
                zlabel('p_3 [m]')
                title('3-D planning model trajectory')
                view(3)
            end
            
            legend([h_pos,h_start,h_end,h_peak,h_num],...
                'traj','start','end','peak','num',...
                'location','northwest')
            
            set(gca,'fontsize',15)
    end
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

function s_sym = eval_spline_at_times(s_sym_to_peak,s_sym_to_stop,t_vec_to_peak,t_vec_to_stop,t)
    s_sym_to_peak_vec = subs(s_sym_to_peak,t,t_vec_to_peak) ;
    s_sym_to_stop_vec = subs(s_sym_to_stop,t,t_vec_to_stop) ;
    s_sym = [s_sym_to_peak_vec, s_sym_to_stop_vec(2:end)] ;
end

function s_mat = make_linear_spline_model(s_sym,k_sym)
    s_v_0_vec = double(subs(s_sym,k_sym,[1 0 0])) ;
    s_a_0_vec = double(subs(s_sym,k_sym,[0 1 0])) ;
    s_v_peak_vec = double(subs(s_sym,k_sym,[0 0 1])) ;
    s_mat = [s_v_0_vec ; s_a_0_vec ; s_v_peak_vec] ; % this is the linear function
end