%% 4.2
%% constants
M = 1.1911;
m = 0.2300;
l = 0.3302;
g = 9.8;
alpha_1 = 1.7265;
alpha_2 = 10.5;

%% symbolic vars and state equations
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

%% jacobians
J1 = jacobian(f,x);
J2 = jacobian(f,u);

% equilibrium point: x_bar = 0, u_bar = 0
x_bar = [0;0;0;0];
u_bar = 0;

A = double(subs(J1, [x;u], [x_bar;u_bar]))
B = double(subs(J2, [x;u], [x_bar;u_bar]))

%% controllability
ctrl_AB = ctrb(A,B)
rankA = rank(A)
rankB = rank(B)

%% 4.3
%% get K_place user acker function from the given eigenvalues in p
p = [-5,-5,-5,-5];
K_place = acker(A,B,p)
% check that the resulting K_place gives back the desired eigenvalues
eig(A+B*K_place)

%% use place function
% perturn eigenvalues slightly
p = [-5.01, -4.99, -5.02, -4.98];
K_place = place(A,B,p)
eig(A+B*K_place)

%% 4.4
%% gain K_LQR1
q_1 = 2;
q_2 = 1;
Q = [q_1 0 0 0; 0 0 0 0; 0 0 q_2 0; 0 0 0 0];
R = 0.2;
K_LQR1 = lqr(A,B,Q,R)

%% gain K_LQR2
q_1 = 0.1;
q_2 = 1;
Q = [q_1 0 0 0; 0 0 0 0; 0 0 q_2 0; 0 0 0 0];
R = 1;
K_LQR2 = lqr(A,B,Q,R)

%% compare eigenvalues
% the eigenvalues are very close
eig(A+B*K_LQR1)
eig(A+B*K_LQR2)
