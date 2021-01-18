function [cell_out] = match_cells(T_out,T_in,cell_in)
% 
% Assumes times are sampled with same discretization
% Assumes T_des is shifted forward w.r.t T_old
% Assumes input and output are same length
%
% Authors: Adam Dai
% Created: 1/14/2021

    % initialize cell_out using size of T_out
    cell_out = cell(1,length(T_out));
    % find matching idx of T_in that matches T_out(1)
    start_idx = find(T_in==T_out(1));
    % amount of overlap between T_in and T_out
    len_overlap = min(length(T_out), length(T_in(start_idx:end)));
    % fill in overlap with cell_in entries
    cell_out(1:len_overlap) = cell_in(start_idx:start_idx+len_overlap-1);
    % fill in remainder (if any) with copies of final entry of cell_in
    cell_out(len_overlap+1:end) = cell_in(end);
    
end