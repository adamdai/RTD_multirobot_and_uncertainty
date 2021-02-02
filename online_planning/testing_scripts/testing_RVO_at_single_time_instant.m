%% user parameters
v_max = 2 ; % m/s

% agent positions
p_1 = [0;0] ;
p_2 = [1;1] ;

% agent velocities
v_1 = [1;0.5] ;
v_2 = [0;0] ;

% agent radii
r_1 = 0.2 ;
r_2 = 0.3 ;

%% automated from here
% get the 1-2 velocity obstacle
VO_12 = make_velocity_obstacle(p_1,p_2,v_2,r_1,r_2) ;

% get the 1-2 reciprocal velocity obstacle
v_mean =(1/2)*(v_1 + v_2) ;
RVO_12 = make_velocity_obstacle(p_1,p_2,v_mean,r_1,r_2) ;

%% test velocities in/out of VO
% make grid of velocities
v_vec = linspace(-v_max,v_max) ;
[V_1,V_2] = meshgrid(v_vec,v_vec) ;
V_grid = [V_1(:), V_2(:)]' ;

% remove velocities with speeds greater than v_max
V_speed = vecnorm(V_grid) ;
V_grid(:,V_speed > v_max) = [] ;

% test points that are in/out of VO
V_in_log = check_points_in_velocity_obstacle(VO_12,V_grid) ;

%% plotting
% setup
figure(1) ; clf ;

%% plot workspace
subplot(1,2,2) ; cla ; axis equal ; hold on ; grid on ;

% plot agent positions
h_1 = plot_disc(r_1,p_1,'facecolor','b','edgecolor','b','facealpha',0.1) ;
h_2 = plot_disc(r_2,p_2,'facecolor',[0.5 0.5 0],'edgecolor',[0.5 0.5 0],'facealpha',0.1) ;

% plot agent velocities
X = [p_1' ; p_2'] ;
V = [v_1' ; v_2'] ;
quiver(X(:,1),X(:,2),V(:,1),V(:,2),'color','k','linewidth',1.5)

% labeling and cleanup
title('workspace')
xlabel('p_1 [m]')
ylabel('p_2 [m]')
% legend([h_1 h_2],{'robot 1','robot 2'},'location','northwest')

set_plot_linewidths(1.25)
set(gca,'fontsize',15)

%% plot velocity space
subplot(1,2,1) ; cla ; axis equal ; hold on ; grid on ;

% set axis based on v_max
v_axis_limits = 1.1*v_max*[-1 1 -1 1] ;
axis(v_axis_limits)

% plot in/out of VO
% plot_path(V_grid(:,V_in_l),'b.')
% plot_path(V_grid(:,V_in_r),'r.')
% plot_path(V_grid(:,V_in_log),'k.')

% plot v_max area
P_v_max_area = make_v_max_feasible_area(v_max,v_axis_limits) ;
h_v_max = plot(P_v_max_area,'facecolor',[1 0 0],'facealpha',0.1,'edgecolor','r') ;

% plot VO_12
h_VO_12 = patch('faces',VO_12.faces,'vertices',VO_12.vertices,'facecolor',[0.5 0.5 0],...
                'edgecolor',[0.5 0.5 0],'facealpha',0.1) ;
            
% plot RVO_12
h_RVO_12 = patch('faces',RVO_12.faces,'vertices',RVO_12.vertices,'facecolor',[0.5 0.5 0],...
                'edgecolor',[0.5 0.5 0],'facealpha',0.1) ;

% labeling and cleanup
title('velocity space for robot 1')
text(VO_12.apex(1)+0.1,VO_12.apex(2)+0.1,'VO')
text(RVO_12.apex(1)+0.1,RVO_12.apex(2)+0.1,'RVO')
xlabel('v_1 [m]')
ylabel('v_2 [m]')
% legend([h_v_max h_VO_12],{'v_{max}','VO_1^2'},'location','northwest')

set_plot_linewidths(1.25)
set(gca,'fontsize',15)

%% helper functions
function V_poly = make_v_max_feasible_area(v_max,axis_limits)
    % create counterclockwise list of vertices around entire area
    V_outside = bounds_to_box(axis_limits) ;
    
    % create clockwise disc of negative space
    theta = linspace(0,2*pi) ;
    V_inside = [v_max*cos(theta) ; v_max*sin(theta)] ;
    
    % create polyline
    V = [V_outside(:,1:end-1), nan(2,1), V_inside(:,1:end-1)]' ;
    V_poly = polyshape(V) ;
end