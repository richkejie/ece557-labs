%% ECE557 Lab 3 Preparation - Nov 3 2025
%% PRA01 Group 5 - Darrian Shue, Richard Wu, Terrence Zhang

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
r = rank(ctrl_AB) % full rank=4 means controllable
rankA = rank(A)
rankB = rank(B)

%% 4.3
%% get K_place user acker function from the given eigenvalues in p
p = [-5,-5,-5,-5];
K_acker = acker(A,B,p) % -14.5282 -17.7043 47.0341 8.3938
% check that the resulting K_place gives back the desired eigenvalues
eig(A - B*K_acker) % poles are approximately at -5

%% use place function
% perturn eigenvalues slightly
p = [-5.01, -4.99, -5.02, -4.98];
K_place = place(A,B,p)
eig(A - B*K_place)

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
eig(A - B*K_LQR1)
eig(A - B*K_LQR2)

%% 4.5
% initial condition
x0=2*[0; 0; pi/24; 0];

% Run simulation for each K calculation
% negative from convention change
K = -K_place 
out_place=sim('lab3_linear.slx',10)

K = -K_LQR1;
out_LQR1 = sim('lab3_linear.slx', 10)

K = -K_LQR2;
out_LQR2 = sim('lab3_linear.slx', 10)

% Output code from lab document
figure(1)
subplot(311)
title('Comparison of the three controllers with linearized plant')
subtitle('Preparation')
ylabel('z')
hold on
plot(out_place.z)
plot(out_LQR1.z)
plot(out_LQR2.z)
subplot(312)
ylabel('theta')
hold on
plot(out_place.theta)
plot(out_LQR1.theta)
plot(out_LQR2.theta)
subplot(313)
ylabel('u')
hold on
plot(out_place.u)
plot(out_LQR1.u)
plot(out_LQR2.u)
legend('pole assignment','LQR1','LQR2','Location','NorthEast')

%% 5: lab activity
Ulim = 5.875;
K = -K_place;
x0 = [0; 0; pi/24; 0];

%% disable saturation
Ulim = inf;
out_place=sim('lab3_nonlinear.slx',10)

% plot figures
figure(2)
subplot(311)
% title('Comparison of the three controllers with linearized plant')
% subtitle('Preparation')
ylabel('z')
hold on
plot(out_place.z.Time,out_place.z.Data(:,1), 'b')
plot(out_place.z.Time,out_place.z.Data(:,2), 'r--')
legend('linear', 'nonlinear')
subplot(312)
ylabel('theta')
hold on
plot(out_place.theta.Time,out_place.theta.Data(:,1), 'b')
plot(out_place.theta.Time,out_place.theta.Data(:,2), 'r--')
legend('linear', 'nonlinear')
subplot(313)
ylabel('u')
hold on
plot(out_place.u.Time,out_place.u.Data(:,1), 'b')
plot(out_place.u.Time,out_place.u.Data(:,2), 'r--')
legend('linear', 'nonlinear')

%% changing initial condition, until unstable
Ulim = inf;
K = -K_place;
x0 = [0; 0; 7*pi/24; 0];
% pos diverges at 8pi/24 (simulation doesn't terminate)
% include plot for 7pi/24 in report
out_place=sim('lab3_nonlinear.slx',10)

% plot figures
figure(3)
% title('Comparison of the three controllers with linearized plant')
% subtitle('Preparation')
ylabel('z')
hold on
plot(out_place.z.Time,out_place.z.Data(:,1), 'b')
plot(out_place.z.Time,out_place.z.Data(:,2), 'r--')
legend('linear', 'nonlinear')

%% changing initial condition, until unstable
% for K_LQR1
Ulim = inf;
K = -K_LQR1; 
x0 = [0; 0; 8*pi/24; 0]; 
% pos diverges at 9pi/24 (simulation doesn't terminate)
% include plot for 8pi/24 in report
out_place=sim('lab3_nonlinear.slx',10)

% plot figures
figure(4)
clf(4)
% title('Comparison of the three controllers with linearized plant')
% subtitle('Preparation')
ylabel('z')
hold on
plot(out_place.z.Time,out_place.z.Data(:,1), 'b')
plot(out_place.z.Time,out_place.z.Data(:,2), 'r--')
legend('linear', 'nonlinear')

%% changing initial condition, until unstable
% for K_LQR2
Ulim = inf;
K = -K_LQR2; 
x0 = [0; 0; 9*pi/24; 0]; 
% pos diverges at 10pi/24 (simulation doesn't terminate)
% include plot for 9pi/24 in report
out_place=sim('lab3_nonlinear.slx',30)

% plot figures
figure(5)
clf(5)
% title('Comparison of the three controllers with linearized plant')
% subtitle('Preparation')
ylabel('z')
hold on
plot(out_place.z.Time,out_place.z.Data(:,1), 'b')
plot(out_place.z.Time,out_place.z.Data(:,2), 'r--')
legend('linear', 'nonlinear')

%% changing initial condition, until unstable
Ulim = 5.875;
K = -K_place;
x0 = [0; 0; 2*pi/24; 0];
% pos diverges at 3pi/24 (simulation doesn't terminate)
% include plot for 2pi/24 in report
out_place=sim('lab3_nonlinear.slx',10)

% plot figures
figure(6)
clf(6)
% title('Comparison of the three controllers with linearized plant')
% subtitle('Preparation')
ylabel('z')
hold on
plot(out_place.z.Time,out_place.z.Data(:,1), 'b')
plot(out_place.z.Time,out_place.z.Data(:,2), 'r--')
legend('linear', 'nonlinear')

%% changing initial condition, until unstable, Ulim
% for K_LQR1
Ulim = 5.875;
K = -K_LQR1; 
x0 = [0; 0; 2*pi/24; 0]; 
% pos diverges at 3pi/24 (simulation doesn't terminate)
% include plot for 2pi/24 in report
out_place=sim('lab3_nonlinear.slx',10)

% plot figures
figure(7)
clf(7)
% title('Comparison of the three controllers with linearized plant')
% subtitle('Preparation')
ylabel('z')
hold on
plot(out_place.z.Time,out_place.z.Data(:,1), 'b')
plot(out_place.z.Time,out_place.z.Data(:,2), 'r--')
legend('linear', 'nonlinear')

%% changing initial condition, until unstable
% for K_LQR2
Ulim = 5.875;
K = -K_LQR2; 
x0 = [0; 0; 2*pi/24; 0]; 
% pos diverges at 3pi/24 (simulation doesn't terminate)
% include plot for 2pi/24 in report
out_place=sim('lab3_nonlinear.slx',10)

% plot figures
figure(8)
clf(8)
% title('Comparison of the three controllers with linearized plant')
% subtitle('Preparation')
ylabel('z')
hold on
plot(out_place.z.Time,out_place.z.Data(:,1), 'b')
plot(out_place.z.Time,out_place.z.Data(:,2), 'r--')
legend('linear', 'nonlinear')