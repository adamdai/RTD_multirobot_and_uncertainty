%% description
% This script demonstrates how we identify unsafe choices of k_peak (i.e.,
% peak velocity trajectory parameters) for the quadrotor in each planning
% iteration. Our interval (zonotope) representation of the quadrotor's
% reachable set results in a reachable /width/ (i.e., distance) of the PRS
% that varies linearly with one's choice of k_peak. Since obstacles are
% also represented as distances (really, as boxes in workspace), we can
% compare the box corresponding to any choice of k_peak with the box
% corresponding to an obstacle and see if they overlap. If they do overlap,
% then there exist unsafe trajectory parameters (i.e., choices of k_peak)
%
% Author: Shreyas Kousik
% Created: 25 Oct 2020 or so
% Updated: 1 Nov 2020
%
%% user parameters
% obstacle bounds
% obs_lb = [0.3 ; 0.3 ; -0.5] ;
% obs_ub = [0.75 ; 1.0 ; 0.25] ;
obs_lb = 2*rand(3,1) - 1;
obs_ub = obs_lb + [0.5 ; 0.5 ; 0.5] ;

% k peak width
k_peak_width = 0.6 ;

%% get box defining overlap of k_peak with obstacle
w_k = 2*k_peak_width ;

%% get k_peak and obstacle overlap
% create unconstrained lower and upper bounds
k_peak_vec = k_peak_width.*ones(3,1) ;
k_peak_lb = -k_peak_vec ;
k_peak_ub = +k_peak_vec ;

% check conditions for when overlap is possible
cond_1 = k_peak_ub(1) >= obs_lb(1) && k_peak_lb(1) <= obs_ub(1) ;
cond_2 = k_peak_ub(2) >= obs_lb(2) && k_peak_lb(2) <= obs_ub(2) ;
cond_3 = k_peak_ub(3) >= obs_lb(3) && k_peak_lb(3) <= obs_ub(3) ;
cond = cond_1 && cond_2 && cond_3 ;

if cond
    disp('The k_peak box and obstacle overlap')
else
    disp('The k_peak box and obstacle do not overlap')
end

% intersect with obstacle
k_peak_lb_int = max([k_peak_lb,obs_lb],[],2) ;
k_peak_ub_int = min([k_peak_ub,obs_ub],[],2) ;

% make a bunch of k_peaks
k_vec = linspace(-k_peak_width,k_peak_width,40) ;
[K1,K2,K3] = ndgrid(k_vec,k_vec,k_vec) ;
v_peak = [K1(:),K2(:),K3(:)]' ;

% filter unsafe k_peaks with logical indexing
v_peak_unsafe_log = all(v_peak <= obs_ub,1) & all(v_peak >= obs_lb,1) ;

%% plotting setup
% create obstacle box
B_obs = reshape([obs_lb, obs_ub]',1,6) ;
[l_obs,w_obs,h_obs,c_obs] = bounds_to_box(B_obs) ;
[F_obs,V_obs] = make_cuboid_for_patch(l_obs,w_obs,h_obs,c_obs) ;

% create k_peak_box
B_peak = reshape([k_peak_lb_int, k_peak_ub_int]',1,6) ;
[l_k_peak,w_k_peak,h_k_peak,c_k_peak] = bounds_to_box(B_peak) ;
[F_k,V_k] = make_cuboid_for_patch(l_k_peak,w_k_peak,h_k_peak,c_k_peak) ; 

% create full-sized k box
[F_big,V_big] = make_cuboid_for_patch(w_k,w_k,w_k) ;

%% plotting
figure(1) ;  clf ; axis equal ; hold on ; grid on ; view(3) ;

% plot obstacle
patch('faces',F_obs,'vertices',V_obs,'facecolor','r','facealpha',0.1)

% plot constrained set
patch('faces',F_k,'vertices',V_k,'facecolor','b','facealpha',0.1)

% plot big set
patch('faces',F_big,'vertices',V_big,'facecolor','g','facealpha',0.1)

% plot points that are in constraints
if any(v_peak_unsafe_log)
    plot_path(v_peak(:,v_peak_unsafe_log),'r.')
end

legend('obstacle','intersection','k peak')