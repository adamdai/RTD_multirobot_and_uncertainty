%% description
% This script demonstrates how to slice the quadrotor PRS representation
% using linear operations. Furthermore, we show that the sliced PRS (i.e.,
% a choice of coefficients of a polynomial) results in a polynomial
% trajectory that is nearly identical to if we evaluated the polynomial.
% This makes sense since the coefficients appear linearly, but it's a very
% pretty result.
%
% Author: Shreyas Kousik
% Created: 26 Oct 2020
% Updated: 1 Nov 2020
%
%% user parameters
% intial conditions
x_0 = zeros(3,1) ;
v_0 = 10*rand(3,1) - 5 ;
a_0 = 20*rand(3,1) - 10 ;

% trajectory parameter
v_peak = 10*rand(3,1) - 5 ;

%% automated from here
% load the PRS
load('quadrotor_PRS_sparse_v_max_5_a_max_10.mat')

% get values to slice k_v, k_a, and k_peak
k_v_coeff = v_0 ./ PRS.v_max ;
k_a_coeff = a_0 ./ PRS.a_max ;
k_peak_coeff = v_peak ./ PRS.v_max ;

% slice the PRS
k_v_offset = k_v_coeff * PRS.zonotope_widths.k_v ;
k_a_offset = k_a_coeff * PRS.zonotope_widths.k_a ;
k_peak_offset = k_peak_coeff * PRS.zonotope_widths.k_peak ;

% get the sliced trajectory
X_PRS = x_0 + k_v_offset + k_a_offset + k_peak_offset ;

%% compare sliced PRS to exact spline trajectory
[~,Z,~] = quadrotor_planning_model_3D(v_0,a_0,v_peak,0,PRS.t_peak,PRS.t_total,PRS.t_sample) ;
X_exact = Z(1:3,:) + repmat(x_0,1,size(Z,2)) ;

%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on ; view(3) ;

% plot the exact spline
plot_path(X_exact,'b-') ;

% plot the sliced PRS
plot_path(X_PRS(:,1:3:end),'b.','markersize',12) ;

legend('exact spline','sliced PRS')