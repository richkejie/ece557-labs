%% 1
%define matrices A, B, C as in (2), 
% with numerical values for the physical parameters
% given in the table presented at the end of Section 1

alpha_1 = 1.7265;
alpha_2 = 10.5;
M = 0.8211;

A = [0 1; 0 -alpha_2/M];
B = [0; alpha_1/M];
C = [1 0];



%% 3
% test your function by finding values of K1, K2
% assigning the eigenvalues of A + BK to be at {−5, −5}
[K1, K2] = state_feedback_design(A, B, [-5 -5])
K = [K1 K2];

%% 4
% Using the gains K1, K2 youUve just obtained,
% test the eigenvalues of A + BK and check they are at {−5, −5}
eig(A+B*K)


%% 6
% test your function by finding values of L1, L2
% assigning the eigenvalues of A − LC to be at {−10, −10}
[L1, L2] = observer_design(A, [-10 -10])
L = [L1; L2];

%% 7
% using the gains L1, L2 that you%ve just obtained,
% test the eigenvalues of A − LC and check they are at {−10, −10}
eig(A-L*C)


%% 9
% using this new function,
% find the matrices of an output feedback controller
% with p_feedback = [−2 − 2] and p_observer = [−20 − 20]
[Actrl, Bctrl, Cctrl, Dctrl] = output_feedback_controller(A,B,C,[-2 -2], [-20 -20])

%% 2
% This function takes as input the matrices A and B of system (2)
% and a row vector p containing desired eigenvalues,
% and it outputs gains K1, K2 for the state feedback controller
function [K1, K2] = state_feedback_design(A,B,p)
    % function code here
    a22 = A(2,2);
    b2 = B(2);
    p1 = p(1);
    p2 = p(2);
    K1 = -(p1*p2)/b2;
    K2 = (-a22+p1+p2)/b2;
end

%% 5
% This function takes as input the matrix A of system (2)
% and a row vector p containing desired eigenvalues,
% and it outputs observer gains L1, L2
function [L1,L2] = observer_design(A,p)
    % function code here
    a22 = A(2,2);
    p1 = p(1);
    p2 = p(2);
    L1 = a22 - (p1+p2);
    L2 = L1*a22 + p1*p2;
end

%% 8
% This function takes as input the matrices A, B, C of system (2)
% and row vectors p_feedback and p_observer
% containing desired eigenvalues for A + BK and A − LC, respectively,
% and it outputs controller matrices (Actrl, Bctrl, Cctrl, Dctrl)
% must call the functions state_feedback_design and observer_design
function [Actrl,Bctrl,Cctrl,Dctrl] = output_feedback_controller(A,B,C,p_feedback,p_observer)
    % function code here
    [K1, K2] = state_feedback_design(A, B, p_feedback);
    K = [K1 K2];
    [L1, L2] = observer_design(A, p_observer);
    L = [L1; L2];

    Actrl = A + B*K - L*C;
    Bctrl = [L -B*K];
    Cctrl = K;
    Dctrl = [0 -K];
end
