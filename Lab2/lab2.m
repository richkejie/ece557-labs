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

%% 5
% This function takes as input the matrix A of system (2)
% and a row vector p containing desired eigenvalues,
% and it outputs observer gains L1, L2
function [L1,L2] = observer_design(A,p)
    % function code here
end

%% 6
% test your function by finding values of L1, L2
% assigning the eigenvalues of A − LC to be at {−10, −10}

%% 7
% using the gains L1, L2 that you%ve just obtained,
% test the eigenvalues of A − LC and check they are at {−10, −10}

%% 8
% This function takes as input the matrices A, B, C of system (2)
% and row vectors p_feedback and p_observer
% containing desired eigenvalues for A + BK and A − LC, respectively,
% and it outputs controller matrices (Actrl, Bctrl, Cctrl, Dctrl)
% must call the functions state_feedback_design and observer_design
function [Actrl,Bctrl,Cctrl,Dctrl] = output_feedback_controller(A,B,C,p_feedback,p_observer)
    % function code here
end

%% 9
% using this new function,
% find the matrices of an output feedback controller
% with p_feedback = [−2 − 2] and p_observer = [−20 − 20]


