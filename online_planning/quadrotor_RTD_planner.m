classdef quadrotor_RTD_planner < planner
%% properties
    properties
        PRS
        ERS
        
        use_agent_for_initial_condition = false ;
    end
%% methods
    methods
        %% constructor
        function P = quadrotor_RTD_planner(varargin)
            P@planner(varargin{:}) ;
        end
        
        %% replan
        function [T_des,U_des,Z_des] = replan(P,agent_info,world_info)
            error('this is incomplete!')
        end
        
        %% get initial condition
        function [x_0,v_0,a_0] = get_initial_condition(P,agent_info,world_info)
            % get the initial condition for the next traj
            if P.use_agent_for_initial_condition
                % get the agent's current state
                x_0 = agent_info.position(:,end) ;
                v_0 = agent_info.velocity(:,end) ;
                
                if ~isempty(P.current_plan.T_des)
                    a_0 = match_trajectories(P.t_move,P.current_plan.T_des,P.current_plan.a_est) ;
                else
                    a_0 = P.current_plan.a_est(:,end) ;
                end
            else
                % get the previous traj at t_plan
                if isempty(P.current_plan.T_des)
                    z_0 = [world_info.start ; zeros(6,1)] ;
                else
                    T = P.current_plan.T_des ;
                    Z = P.current_plan.Z_des ;
                    t_0 = min(P.t_move,T(end)) ;
                    z_0 = match_trajectories(t_0,T,Z) ;
                end
                
                x_0 = z_0(1:3) ;
                v_0 = z_0(4:6) ;
                a_0 = z_0(7:9) ;
                agent_info.position(:,end) = x_0 ;
            end
        end
    end
end