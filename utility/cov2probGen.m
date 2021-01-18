function [pG] = cov2probGen(cov)
% cov2probGen - Generates the probabilistic generators for given covariance
% matrix
%
% Syntax:  
%    [pG] = cov2probGen(cov)
%
% Inputs:
%    cov - covariance matrix
%
% Outputs:
%    pG - probabilistic generators
%
% Example: 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: none

% Author:       Akshay Shetty
% Written:      19-July-2019
% Last revision: ---

%------------- BEGIN CODE --------------

[u,s,~] = svd(cov);

% s = round(s,4);
s(s<0)=0;

pG = u*sqrt(s);

%------------- END OF CODE --------------