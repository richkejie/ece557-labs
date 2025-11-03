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
f = [z_dot;z_ddot;theta_dot;theta_ddot]

matlabFunction(eval(f), 'Vars', {x,u}, 'File', 'cartpend', 'Outputs', {'xdot'});

cartpend_test = cartpend([0;0;pi/2;0],1)

%% jacobians
J1 = jacobian(f,x);
J2 = jacobian(f,u);

% equilibrium point: x_bar = 0, u_bar = 0
x_bar = [0;0;0;0];
u_bar = 0;

A = subs(J1, [x;u], [x_bar;u_bar])
B = subs(J2, [x;u], [x_bar;u_bar])

%% controllability
ctrl_AB = ctrb(A,B)
rankA = rank(A)
rankB = rank(B)

%% 4.3
p = [-5,-5,-5,-5];
K_place = acker(A,B,p)
eig(A+B*K+place)


