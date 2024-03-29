function x_dot = quadrotor_planning_model_1D_to_peak(t_dummy,in2,u_dummy)
%QUADROTOR_PLANNING_MODEL_1D_TO_PEAK
%    X_DOT = QUADROTOR_PLANNING_MODEL_1D_TO_PEAK(T_DUMMY,IN2,U_DUMMY)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    09-Oct-2020 12:26:20

a_0 = in2(3,:);
t = in2(5,:);
v_0 = in2(2,:);
v_peak = in2(4,:);
x_dot = [v_0+a_0.*t-t.^2.*(a_0.*(4.0./3.0)+v_0.*(4.0./3.0)-v_peak.*(4.0./3.0))+t.^3.*(a_0.*(4.0./9.0)+v_0.*(1.6e+1./2.7e+1)-v_peak.*(1.6e+1./2.7e+1));0.0;0.0;0.0;1.0];
