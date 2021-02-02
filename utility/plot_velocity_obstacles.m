function H = plot_velocity_obstacles(VO_list,v_max)
    % set axis based on v_max
    v_axis_limits = 1.1*v_max*[-1 1 -1 1] ;
    axis(v_axis_limits)

    % plot v_max area
    P_v_max_area = make_v_max_feasible_area(v_max,v_axis_limits) ;
    H = plot(P_v_max_area,'facecolor',[1 0 0],'facealpha',0.1,'edgecolor','r') ;

    % plot each VO
    for idx = 1:length(VO_list)
        VO = VO_list(idx) ;
        h_VO = patch('faces',VO.faces,'vertices',VO.vertices,'facecolor',[1 0 0],...
            'edgecolor',[1 0 0],'facealpha',0.1) ;
        H = [H, h_VO] ;
    end

    if nargout < 1
        clear H ;
    end
end

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