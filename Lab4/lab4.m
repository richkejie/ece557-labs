%% ECE557 Lab 4 Preparation - Nov 17 2025
%% PRA01 Group 5 - Darrian Shue, Richard Wu, Terrence Zhang

%% Code below copied from lab3
M = 1.1911;
m = 0.2300;
l = 0.3302;
g = 9.8;
alpha_1 = 1.7265;
alpha_2 = 10.5;

% symbolic vars and state equations
syms z z_dot theta theta_dot u real;

z_ddot = (-m*l*sin(theta)*theta_dot^2 + m*g*sin(theta)*cos(theta) + alpha_1*u - alpha_2*z_dot) / (M + m*(sin(theta))^2);
theta_ddot = (-m*l*sin(theta)*cos(theta)*theta_dot^2 + (M+m)*g*sin(theta) + (alpha_1*u - alpha_2*z_dot)*cos(theta)) / (l*(M+m*(sin(theta))^2));

x = [z;z_dot;theta;theta_dot];
% f is the derivative of x, i.e., x_dot, display f:
f = [z_dot;z_ddot;theta_dot;theta_ddot]

% create matlab function cartpen
matlabFunction(eval(f), 'Vars', {x,u}, 'File', 'cartpend', 'Outputs', {'xdot'});

% cartpend test --- should get [0; 1.3249; 0; 29.6790]
cartpend_test = cartpend([0;0;pi/2;0],1)

% jacobians
J1 = jacobian(f,x);
J2 = jacobian(f,u);

% equilibrium point: x_bar = 0, u_bar = 0
x_bar = [0;0;0;0];
u_bar = 0;

A = double(subs(J1, [x;u], [x_bar;u_bar]))
B = double(subs(J2, [x;u], [x_bar;u_bar]))

%% begin 4.2 of Lab 4

% define the output matrix C
C = [1 0 0 0; 0 0 1 0];

% using the commands rank and obsv check that the linearization is
% observable
ctrl_AB = ctrb(A,B)
r_ctrl = rank(ctrl_AB) % full rank=4 means controllable

obs_AC = obsv(A,C)
% Check observability rank
r_obs = rank(obs_AC) % full rank=4 means observable


%% 4.2 continued: get K and L
% design feedback gain K
%p_K = [-5, -5, -5, -5];
p_K = [-5.01, -4.99, -5.02, -4.98];
K = -place(A,B,p_K)
eig(A+B*K)

% design observer gain L
%p_L = [-10, -10, -10, -10];
p_L = [-10.01, -9.99, -10.02, -9.98];
% duality says (A' - C'*L')' = A - L*C
L = place(A',C',p_L)'
eig(A-L*C)

%% 4.2 continued
% create matrices
Actrl = A + B*K - L*C
Bctrl = [L, -B*K]
Cctrl = K
Dctrl = [zeros(1,2), -K]

% initial condition and saturation level
x0 = [0; 0; pi/24; 0]
Ulim = 5.875

%% 4.3

%% NOTE, uncomment this to run the original sim w/ L poles at -10
% Run simulation for A-LC at -10
out=sim('lab4_prep.slx',30)

% plot
figure(1)
subplot(311)
title('State and output feedback controller')
subtitle('Preparation')
ylabel('z')
hold on
plot(out.z)
legend('State Feedback','Output Feedback','Desired Position','Location','NorthEast')
subplot(312)
ylabel('theta')
hold on
plot(out.theta)
legend('State Feedback','Output Feedback','Location','NorthEast')
subplot(313)
ylabel('u')
hold on
plot(out.u)
legend('State Feedback','Output Feedback','Location','NorthEast')

%% NOTE, uncomment this to run the original sim w/ L poles at -40
% %% change L to assign eigenvalues of A-LC at around -40
p_L = [-40.01, -39.99, -40.02, -39.98];
% duality says (A' - C'*L')' = A - L*C
L = place(A',C',p_L)'
% create matrices
Actrl = A + B*K - L*C
Bctrl = [L, -B*K]
Cctrl = K
Dctrl = [zeros(1,2), -K]

% Run simulation
out=sim('lab4_prep.slx',30)

% plot
figure(2)
subplot(311)
title('State and output feedback controller')
subtitle('Preparation')
ylabel('z')
hold on
plot(out.z)
legend('State Feedback','Output Feedback','Desired Position','Location','NorthEast')
subplot(312)
ylabel('theta')
hold on
plot(out.theta)
legend('State Feedback','Output Feedback','Location','NorthEast')
subplot(313)
ylabel('u')
hold on
plot(out.u)
legend('State Feedback','Output Feedback','Location','NorthEast')

%% tune K without changing L, where eig(A-LC) are at -40. Use poles at -8 for K
% tune K using pole assignment
% p_K = [-5.01, -4.99, -5.02, -4.98]; % this was the original value, T_s>1
p_K = [-8.01, -7.99, -8.02, -7.98]; % this value for poles is good, w/ T_s<1
% p_K = [-10.01, -9.99, -10.02, -9.98]; system unstable w/ poles at -10
K = -place(A,B,p_K)
% everything else same as above
p_L = [-40.01, -39.99, -40.02, -39.98];
% duality says (A' - C'*L')' = A - L*C
L = place(A',C',p_L)'
% create matrices
Actrl = A + B*K - L*C
Bctrl = [L, -B*K]
Cctrl = K
Dctrl = [zeros(1,2), -K]

% Run simulation
out=sim('lab4_prep.slx',30)

% plot
figure(3)
subplot(311)
title('State and output feedback controller')
subtitle('Preparation')
ylabel('z')
hold on
plot(out.z)
legend('State Feedback','Output Feedback','Desired Position','Location','NorthEast')
subplot(312)
ylabel('theta')
hold on
plot(out.theta)
legend('State Feedback','Output Feedback','Location','NorthEast')
subplot(313)
ylabel('u')
hold on
plot(out.u)
legend('State Feedback','Output Feedback','Location','NorthEast')


%%
% LQR tuning not working well, just use pole assignment for poles at -8
% % gain K_LQR1
q_1 = 50;
q_2 = 0.01;
Q = [q_1 0 0 0; 0 0 0 0; 0 0 q_2 0; 0 0 0 0];
R = 0.01;
K = -lqr(A,B,Q,R)
% % gain K_LQR2
% q_1 = 0.1;
% q_2 = 1;
% Q = [q_1 0 0 0; 0 0 0 0; 0 0 q_2 0; 0 0 0 0];
% R = 1;
% K = -lqr(A,B,Q,R)

% everything else same as above
p_L = [-40.01, -39.99, -40.02, -39.98];
% duality says (A' - C'*L')' = A - L*C
L = place(A',C',p_L)'
% create matrices
Actrl = A + B*K - L*C
Bctrl = [L, -B*K]
Cctrl = K
Dctrl = [zeros(1,2), -K]

% Run simulation
out=sim('lab4_prep.slx',30)

% plot
figure(4)
subplot(311)
title('State and output feedback controller')
subtitle('Preparation')
ylabel('z')
hold on
plot(out.z)
legend('State Feedback','Output Feedback','Desired Position','Location','NorthEast')
subplot(312)
ylabel('theta')
hold on
plot(out.theta)
legend('State Feedback','Output Feedback','Location','NorthEast')
subplot(313)
ylabel('u')
hold on
plot(out.u)
legend('State Feedback','Output Feedback','Location','NorthEast')


