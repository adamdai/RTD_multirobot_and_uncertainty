function generate_quadrotor_symbolic_planning_model_1D(t_peak,t_total)
% [dyn_to_peak,dyn_failsafe] = generate_symbolic_1D_planning_model(t_peak,t_total)
% This function generates the symbolic planning model used by CORA 2020 to
% compute the planning reachable set (PRS) for the quadrotor.
%
% Authors: Patrick Holmes and Shreyas Kousik
% Created: 9 Oct 2020
%
%% set additional timing parameters automatically
t_to_stop = t_total - t_peak; % time from peak to stop
a_peak = 0; % 0 acceleration at peak speed
v_f = 0; % 0 final speed
a_f = 0; % 0 final acceleration.

%% generate "to peak velocity" model
% create variables for 1D position, initial velocity, initial acceleration,
% peak velocity, and time (we artificially include time as a state so that
% we can make the planning model time-varying polynomials)
syms p v_0 a_0 v_peak t u_dummy ;

% compute change in velocity/accel
delta_v = v_peak - v_0 - a_0*t_peak ;
delta_a = a_peak - a_0 ;

% compute spline parameters (the planning model is represented by
% polynomial splines)
[ax, bx, cx] = single_axis_params(delta_v,delta_a,t_peak) ;

% compute symbolic planning model velocity
p_dot = (ax/24).*t.^4 + (bx/6).*t.^3 + (cx/2).*t.^2 + (a_0).*t + v_0 ;

% create state vector for CORA and planning model dynamics
x = [p; v_0; a_0; v_peak; t] ;
x_dot = [p_dot; 0; 0; 0; 1];

% make symbolic function
matlabFunction(x_dot, 'file','quadrotor_symbolic_planning_model_1D_to_peak',...
    'vars', {x u_dummy}) ;

%% generate failsafe maneuver model
% for each axis, compute the change in velocity/accel
delta_v = v_f - v_peak - a_peak*t_to_stop ;
delta_a = a_f - a_peak ;

% compute spline parameters
[ax, bx, cx] = single_axis_params(delta_v,delta_a,t_to_stop);

% create velocity
p_dot = (ax/24).*t.^4 + (bx/6).*t.^3 + (cx/2).*t.^2 + (a_peak).*t + v_peak;

% create state vector for CORA and planning model dynamics
x = [p; v_0; a_0; v_peak; t];
x_dot = [p_dot; 0; 0; 0; 1];

% save function
matlabFunction(x_dot, 'file','quadrotor_symbolic_planning_model_1D_failsafe',...
    'vars', {x u_dummy}) ;
end

%% helper function
function [a,b,c] = single_axis_params(delta_v,delta_a,t_final)
    M = [0 0 ;
         -12 6*t_final ;
         6*t_final -2*t_final^2] ;
         
    out = (1/t_final^3)*M*[delta_v ; delta_a] ;
    a = out(1) ;
    b = out(2) ;
    c = out(3) ;
end
   