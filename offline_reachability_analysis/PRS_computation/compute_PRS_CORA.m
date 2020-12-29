%% description
% We generate the 1-D values for k_v, k_a, k_peak, and error for
% each zonotope in the format:
%   [k_v(1),    k_v(2),    ..., k_v(n) ;
%    k_a(1),    k_a(2),    ..., k_a(n) ;
%    k_peak(1), k_peak(2), ..., k_peak(n) ;
%    error(1),  error(2),  ..., error(n) ] ;
%
% where n is the number of zonotopes in the PRS
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: 9 Oct 2020
% Updated: 16 Oct 2020
%
%% user parameters
% planning model parameters
t_peak = 1.5 ; % time to peak velocity along each axis
t_total = 3 ; % total duration of trajectory
t_sample = 0.02 ; % timestep for reachability
v_max = 5 ; % max parameterized speed in m/s
a_max = 10 ; % max parameterized acceleration in m/s

%% automated from here
% create symbolic dynamics
disp('Generating symbolic dynamics files in your present working directory.')
generate_quadrotor_symbolic_planning_model_1D(t_peak,t_total) ;

%% generate 1D PRS for "to peak" part of plan
disp('Generating 1D PRS for "to peak" part of plan')

% CORA parameters
params.tStart = 0 ;
params.tFinal = t_peak ;
params.R0 = zonotope([zeros(5,1), diag([0, v_max, a_max, v_max, 0])]) ; % init cond zonotope
params.U = zonotope(0,0) ;

% CORA options
options.timeStep = t_sample ;
options.taylorTerms = 5 ;
options.tensorOrder = 3 ;
options.errorOrder = 1 ;
options.zonotopeOrder = 20 ;
options.intermediateOrder = 20 ;
options.maxError = 1000*ones(5, 1) ;
options.reductionInterval = inf ;
options.reductionTechnique = 'girard' ;
options.alg = 'lin' ;

% set up dynamics
dyn_to_peak = nonlinearSys(@quadrotor_symbolic_planning_model_1D_to_peak,5,1) ;

% run reachability analysis
t_reach_to_peak = tic ;
R_to_peak = reach(dyn_to_peak,params,options) ;
t_reach_to_peak = toc(t_reach_to_peak) ;
disp(['Time to compute "to peak" part of 1D PRS: ',num2str(t_reach_to_peak),' s'])

%% generate 1D PRS for "failsafe" part of trajectory
disp('Generating 1D PRS for failsafe maneuver')

% update parameters
params.tFinal = t_total - t_peak ;
params.R0 = R_to_peak.timePoint.set{end} - [zeros(4,1) ; t_peak] ; % reset time to t = 0

% set up dynamics
dyn_failsafe = nonlinearSys(@quadrotor_symbolic_planning_model_1D_failsafe,5,1) ;

% run reachability analysis
t_reach_failsafe = tic ;
R_failsafe = reach(dyn_failsafe,params,options) ;
t_reach_failsafe = toc(t_reach_failsafe) ;
disp(['Time to compute failsafe part of 1D PRS: ',num2str(t_reach_failsafe),' s'])

%% save the important info from the reachable set
disp('Extracting useful PRS data for sparse representation')

% set up to save a sparse matrix representation of the PRS
n_to_peak = length(R_to_peak.timeInterval.set) ;
n_failsafe = length(R_failsafe.timeInterval.set) ;
n_PRS = n_to_peak + n_failsafe ;

% note that the centers are always zeros because the planning model is
% symmetrical about 0, so we need only save the generators -- furthermore,
% we only need to save the 1-D values for k_v, k_a, k_peak, and error for
% each zonotope in the format:
%   [k_v(1),    k_v(2),    ..., k_v(n) ;
%    k_a(1),    k_a(2),    ..., k_a(n) ;
%    k_peak(1), k_peak(2), ..., k_peak(n) ;
%    error(1),  error(2),  ..., error(n) ] ;
%
% where n is the number of zonotopes in the PRS
PRS_zonotope_widths = zeros(4,n_PRS) ;

% iterate through the 1-D PRS and clean each zonotope
for idx = 1:n_PRS
    % get the current zonotope
    if idx <= n_to_peak
        Z_idx = R_to_peak.timeInterval.set{idx} ;
    else
        Z_idx = R_failsafe.timeInterval.set{idx - n_to_peak} ;
    end
    G_idx = Z_idx.generators ;
    
    % store the relevant data from the generator matrix
    PRS_zonotope_widths(:,idx) = extract_generator_matrix_data(G_idx) ;
end

%% store sparse representation of PRS as a structure
disp('Storing sparse PRS representation')

% store the 1-D PRS representation
PRS.zonotope_widths = [] ;
PRS.zonotope_widths.k_v = PRS_zonotope_widths(1,:) ;
PRS.zonotope_widths.k_a = PRS_zonotope_widths(2,:) ;
PRS.zonotope_widths.k_peak = PRS_zonotope_widths(3,:) ;
PRS.zonotope_widths.error = PRS_zonotope_widths(4,:) ;
PRS.n_zonotopes = size(PRS_zonotope_widths,2) ;

% store dynamic bounds info
PRS.v_max = v_max ;
PRS.a_max = a_max ;

% store timing info
PRS.t_peak = t_peak ;
PRS.t_total = t_total ;
PRS.t_sample = t_sample ;

%% save output
filename = ['quadrotor_PRS_CORA_sparse_v_max_',num2str(v_max),...
            '_a_max_',num2str(a_max),'.mat'] ;
        
save(filename,'PRS')

%% helper functions
function G_out = extract_generator_matrix_data(G_in)
    % remove the time row
    G_in = G_in(1:4,:) ;

    % remove all zero columns
    G_zero_cols_log = all(G_in == 0,1) ;
    G_in(:,G_zero_cols_log) = [] ;
    
    % collect the k-sliceable generators
    G_k = zeros(1,3) ;
    e_cols_log = true(1,size(G_in,2)) ; % log idx for non-k-sliceable generators
    for idx = 2:4
        % get column for the current k value
        col_log = G_in(idx,:) ~= 0 ;
        
        % store the corresponding generator value in the position
        % coordinate row
        G_k(idx-1) = G_in(1,col_log) ;
        
        % mark the current column as used
        e_cols_log(col_log) = false ;
    end
    
    % add the position error from all the non-k-sliceable generators
    G_e = sum(G_in(1,e_cols_log),2) ;
    
    % create 1D generator position values as a row vector
    G_out = [G_k, G_e] ;
end