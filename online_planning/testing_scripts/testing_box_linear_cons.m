%% description
% This script shows how we can analytically convert boxes to linear
% constraints (actually, to affine operations on points), which lets us
% evaluate whether or not trajectory parameters are unsafe at runtime.
% Consider a box defined as lb, ub \in \R^3 (so, lb = [xmin ymin zmin] and
% ub = [xmax ymax zmax]). This script converts lb and ub to a matrix/vector
% representation, A \in \R^{6*3} and b \in \R^6 so that
%
%   A*x + b > 0  <=>  (x >= lb) & (x <= ub)
%
% This script does the above conversion for n_box number of boxes and tests
% n_pts number of points, so you can test how slow A*x + b is. In practice,
% we test about 20 boxes (representing obstacles) and 10k points
% (representing trajectory parameters) in each planning iteration. This
% takes around 10--30 ms.
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: ages ago
% Updated: 1 Nov 2020
%
%% user parameters
% number of boxes
n_box = 20 ;

% number of points to test
n_pts = 10000 ;

% range of points
x_range = 4 ;

%% create boxes
lb = -2*x_range*rand(3,n_box) + x_range ;
ub = lb + 0.5 ;

%% create constraints
tic
% n_box = size(lb,2) ;
b = [-lb ; ub] ;
b_con = b(:) ;

A_con = repmat([eye(3) ; -eye(3)],size(b_con,1)/6,1) ;

%% create points to test
% X = 4.*rand(3,300) - 2 ;
xvec = linspace(-x_range,x_range,ceil(n_pts.^(1/3))) ;
[X1,X2,X3] = ndgrid(xvec,xvec,xvec) ;
X = [X1(:),X2(:),X3(:)]' ;

%% evaluate constraints
% use Ax + b > 0
C_eval = A_con*X + b_con ; % takes 30 ms
C_rshp = reshape(C_eval,6,[]) ; %  < 1 ms
C_min = min(C_rshp,[],1) + 1e-6; % 15 ms
C_rshp_2 = reshape(C_min,size(C_eval,1)/6,[]) ; % < 1 ms
C_max = max(C_rshp_2,[],1) ; % 1 ms
X_log = C_max >= 0 ;
toc

%% evaluate cons the "hard" way
X_log_2 = false(1,size(X,2)) ;
for idx = 1:n_box
    lb_big = repmat(lb(:,idx),1,size(X,2)) ;
    ub_big = repmat(ub(:,idx),1,size(X,2)) ;
    X_log_2 = X_log_2 | all(X >= lb_big & X <= ub_big,1) ;
end

%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on ; view(3)

% plot boxes
for idx = 1:n_box
    b_box = [lb(:,idx) ub(:,idx)]' ;
    [l,w,h,c] = bounds_to_box(b_box(:)') ;
    [F,V] = make_cuboid_for_patch(l,w,h,c) ;
    patch('faces',F,'vertices',V,'facecolor','r','facealpha',0.1)
end

% plot points in box
plot_path(X(:,X_log),'b*')
plot_path(X(:,~X_log),'r.')
plot_path(X(:,X_log_2),'bo')