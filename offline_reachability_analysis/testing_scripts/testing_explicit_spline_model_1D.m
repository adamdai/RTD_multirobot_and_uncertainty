%% user parameters
% timing
t_peak = 1.5 ;
t_sample = 0.1 ;
t_total = 3 ;

% test trajectory parameters
v_0_test = -10 ;
a_0_test = -5 ;
v_peak_test = 12 ;

%% automated from here
% create symbolic parameters
syms v_0 a_0 v_peak t ;

% set additional parameters
t_to_stop = t_total - t_peak ; % time from peak to stop
a_peak = 0 ; % 0 acceleration at peak speed
v_f = 0 ; % 0 final speed
a_f = 0 ; % 0 final acceleration.

%% "to peak" part of trajectory
% compute change in velocity/accel
delta_v = v_peak - v_0 - a_0*t_peak ;
delta_a = a_peak - a_0 ;

% compute spline parameters (the planning model is represented by
% polynomial splines)
[a, b, c] = single_axis_params(delta_v,delta_a,t_peak) ;

% compute symbolic planning model position and velocity
p_sym_to_peak = (a/120).*t.^5 + (b/24).*t.^4 + (c/6).*t.^3 + (a_0/2).*t^2 + v_0*t ;
v_sym_to_peak = diff(p_sym_to_peak,t) ;
a_sym_to_peak = diff(v_sym_to_peak,t) ;

%% "to stop" part of trajectory
% change in velocity/accel
delta_v = v_f - v_peak - a_peak*t_to_stop;
delta_a = a_f - a_peak ;
% delta_v = v_f - v_sym_to_peak(end) - a_peak*t_to_stop ;
% delta_a = a_f - a_sym_to_peak(end) ;

% compute spline parameters
[a, b, c] = single_axis_params(delta_v,delta_a,t_to_stop) ;

% compute symbolic planning model position
p_sym_to_stop = (a/120).*t.^5 + (b/24).*t.^4 + (c/6).*t.^3 + (a_peak/2).*t^2 + v_peak*t ;

%% create linear functions for position spline
% create vector of sample times
t_vec_to_peak = 0:t_sample:t_peak ;
t_vec_to_stop = 0:t_sample:t_to_stop ;

% create spline at each point in time
p_sym_to_peak = subs(p_sym_to_peak,t,t_vec_to_peak) ;
p_sym_to_stop = subs(p_sym_to_stop,t,t_vec_to_stop) ;
p_sym = [p_sym_to_peak, p_sym_to_peak(end) + p_sym_to_stop(2:end)] ;

% break spline up into vectors for each parameter
s_vars = [v_0,a_0,v_peak] ;
v_0_vec = double(subs(p_sym,s_vars,[1 0 0])) ;
a_0_vec = double(subs(p_sym,s_vars,[0 1 0])) ;
v_peak_vec = double(subs(p_sym,s_vars,[0 0 1])) ;
p_mat = [v_0_vec ; a_0_vec ; v_peak_vec] ; % this is the linear function

%% create linear functions for bounding spline
% timing
% t_vec = 0:t_sample:t_total ;
t_vec = t_sample.*(0:size(p_mat,2)-1) ;
t_c = 0.5.*(t_vec(2:end) + t_vec(1:end-1)) ;

% centers
c_sym_k = 0.5.*(p_sym(2:end) + p_sym(1:(end-1))) ;
v_0_c = double(subs(c_sym_k,s_vars,[1 0 0])) ;
a_0_c = double(subs(c_sym_k,s_vars,[0 1 0])) ;
v_peak_c = double(subs(c_sym_k,s_vars,[0 0 1])) ;
c_mat = [v_0_c ; a_0_c ; v_peak_c] ;

% widths
w_sym_k = 0.5.*(p_sym(2:end) - p_sym(1:(end-1))) ;
v_0_w = double(subs(w_sym_k,s_vars,[1 0 0])) ;
a_0_w = double(subs(w_sym_k,s_vars,[0 1 0])) ;
v_peak_w = double(subs(w_sym_k,s_vars,[0 0 1])) ;
w_mat = [v_0_w ; a_0_w ; v_peak_w] ;

%% test spline
% use the linear formulation to create the spline
k_vals = [v_0_test, a_0_test, v_peak_test] ;
p_vec = k_vals * p_mat ;

% use the analytical formula to create the spline
p_compare = double(subs(p_sym,s_vars,k_vals)) ;

% use the linear function to create centers and widths
c_vec = k_vals * c_mat ;
w_vec = k_vals * w_mat ;

%% plotting
figure(1) ; clf ; hold on ;grid on ;

% plot linear formulation
plot(t_vec,p_vec)
plot(t_c,c_vec,'r.')
plot(t_c,c_vec+w_vec,'r-')
plot(t_c,c_vec-w_vec,'r-')

% plot analytical formulation
% plot(t_vec,p_compare,'b.')

% labeling
xlabel('t [s]')
ylabel('position [m]')
title('1-D nominal trajectory')
legend('nom. traj.','zono. centers','zono. widths','location','northwest')
set(gca,'FontSize',15)

%% helper functions
function [a,b,c] = single_axis_params(delta_v,delta_a,t_final)
    M = [0 0 ;
         -12 6*t_final ;
         6*t_final -2*t_final^2] ;
         
    out = (1/t_final^3)*M*[delta_v ; delta_a] ;
    a = out(1) ;
    b = out(2) ;
    c = out(3) ;
end
   