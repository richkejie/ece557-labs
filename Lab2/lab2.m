%% 1
%define matrices A, B, C as in (2), 
% with numerical values for the physical parameters
% given in the table presented at the end of Section 1

%% 2
% This function takes as input the matrices A and B of system (2)
% and a row vector p containing desired eigenvalues,
% and it outputs gains K1, K2 for the state feedback controller
function [K1 K2] = state_feedback_design(A,B,p)
    % function code here
end

%% 3
% test your function by finding values of K1, K2
% assigning the eigenvalues of A + BK to be at {−5, −5}

%% 4
% Using the gains K1, K2 youUve just obtained,
% test the eigenvalues of A + BK and check they are at {−5, −5}