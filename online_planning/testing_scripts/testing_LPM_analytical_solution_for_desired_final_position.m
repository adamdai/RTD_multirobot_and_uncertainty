%% description
% Given initial velocity v_0, acceleration a_0, and desired final position
% p_f, compute v_peak to minimize the distance between the end of a
% parameterized trajectory and the point p_f. That is, we're solving
%
%   min || p(t_f,k) - p_f||_2^2
%    k
%   s.t. ||v_peak||_2 <= v_max and
%        v(0) = v_0, a(0) = a_0.
%
% Authors: Shreyas Kousik and Adam Dai
% Created: 13 Jan 2020
%
%% user parameters
% initial conditions
v_0 = [1;-1;1] ;
a_0 = [1;-3;2] ;

% desired final position
p_f = 4*rand(3,1) - 2;

% max allowed speed
v_max = 1 ; % [m/s]

%% automated from here
load('quadrotor_linear_planning_model.mat') ;

%% strategy 1
% with this strategy, we compute the optimal v_peak, then project it onto
% the ball of radius v_max; this may or may not be the optimal solution

% get final position LTV matrix from the whole LPM position array
LPM_p_f = LPM.position(:,end) ;

% get position change due to initial conditions
p_from_v_and_a_0 = [v_0, a_0] * LPM_p_f(1:2) ;

% recall that we can write
%
%   p_f = k' * LPM_p_f = [v_0 a_0 v_peak] * [p_1 ; p_2 ; p_3], 
%
% where p_i are the entries of LPM_p_f; we then plug in v_0 and a_0, and
% solve for v_peak:
v_peak_1 = (p_f - p_from_v_and_a_0)/LPM_p_f(3) ;

% if v_peak exceeds max speed, project it on to the max speed ball
v_peak_speed = vecnorm(v_peak_1) ;
if v_peak_speed > v_max
    disp('adjusting v_peak for bein too fast')
    v_peak_1 = v_peak_1 * v_max / v_peak_speed ;
end

%% strategy 1
% minimize the cost function explicitly as written above in the
% description, subject to the constraints
n_dim = length(v_0) ;

v_peak_2 = fmincon(@(v_peak) cost(v_peak,v_0,a_0,p_f,LPM_p_f),...
    zeros(n_dim,1),[],[],[],[],[],[],...
    @(v_peak) nonlcon(v_peak,v_max)) ;

%% plug in solutions for plotting
k_strat_1 = [v_0 a_0 v_peak_1] ;
P_strat_1 = k_strat_1*LPM.position ;

k_strat_2 = [v_0 a_0 v_peak_2] ;
P_strat_2 = k_strat_2*LPM.position ;

%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on ;

% plot starting and desired position
plot_path([0;0],'go','markersize',12)
plot_path(p_f,'gp','markersize',12)

% plot trajectory
plot_path(P_strat_1,'b-')
plot_path(P_strat_2,'r--')

xlabel('p_1 [m]')
ylabel('p_2 [m]')
legend('start','goal','strat 1','strat 2')
set(gca,'fontsize',15)
set_plot_linewidths(1.5)

%% helper functions
% cost function, be lazy and let fmincon compute numerical grads
function c = cost(v_peak,v_0,a_0,p_f,LPM_p_f)
    d = vecnorm(p_f - [v_0 a_0 v_peak]*LPM_p_f) ;
    c = d.^2 ;
end

% convex nonlcon
function [c,ceq] = nonlcon(v_peak,v_max)
    c = vecnorm(v_peak)^2 - v_max^2 ;
    ceq = [] ;
end