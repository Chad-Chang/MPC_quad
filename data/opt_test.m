
clc;
clear;

% Define problem parameters
n = 4; % Number of states
m = 2; % Number of inputs
Q = eye(n); % State cost matrix
R = 0.1 * eye(m); % Input cost matrix

% System dynamics (example)
A = [1, 0.1, 0, 0;
     0, 1, 0.1, 0;
     0, 0, 1, 0.1;
     0, 0, 0, 1];
B = [0, 0;
     0.1, 0;
     0, 0;
     0, 0.1];

% Initial state and reference state
x0 = [0; 0; 0; 0];
x_ref = [1; 1; 0; 0];

% Time horizon
N = 20; % Number of time steps

% Decision variable: [u_0; u_1; ...; u_{N-1}; x_0; x_1; ...; x_N]
num_vars = N * m + (N + 1) * n;

% Objective function
H = blkdiag(kron(eye(N), R), kron(eye(N+1), Q));
f = zeros(num_vars, 1);
f((N*m+1):(N*m+N*n)) = -2 * kron(ones(N+1, 1), x_ref' * Q)';

objective = @(z) z' * H * z;

% Equality constraints (dynamics)
Aeq = zeros(N * n, num_vars);
beq = zeros(N * n, 1);

for k = 1:N
    % Dynamics: x_{k+1} = A * x_k + B * u_k
    Aeq((k-1)*n+1:k*n, (k-1)*m+1:k*m) = -B;
    Aeq((k-1)*n+1:k*n, (N*m+(k-1)*n+1):(N*m+k*n)) = eye(n);
    Aeq((k-1)*n+1:k*n, (N*m+k*n+1):(N*m+(k+1)*n)) = -A;
end

% Initial condition constraint
Aeq = [Aeq; zeros(n, num_vars)];
Aeq(end-n+1:end, (N*m+1):(N*m+n)) = eye(n);
beq = [beq; x0];

% Input and state bounds
lb = -inf(num_vars, 1);
ub = inf(num_vars, 1);

% Input bounds
for k = 1:N
    lb((k-1)*m+1:k*m) = -10; % Input lower bound
    ub((k-1)*m+1:k*m) = 10;  % Input upper bound
end

% State bounds (optional)
for k = 0:N
    lb(N*m+k*n+1:N*m+(k+1)*n) = -5; % State lower bound
    ub(N*m+k*n+1:N*m+(k+1)*n) = 5;  % State upper bound
end

% Initial guess
z0 = zeros(num_vars, 1);

% Solve the optimization problem
options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'iter');
[z_opt, cost] = fmincon(objective, z0, [], [], Aeq, beq, lb, ub, [], options);

% Extract optimal inputs and states
u_opt = reshape(z_opt(1:N*m), m, N);
x_opt = reshape(z_opt(N*m+1:end), n, N+1);

% Plot results
figure;
t = 0:N;
subplot(2, 1, 1);
plot(t, x_opt', '-o');
title('Optimal State Trajectory');
xlabel('Time Step');
ylabel('States');
legend('x_1', 'x_2', 'x_3', 'x_4');

t = 0:N-1;
subplot(2, 1, 2);
plot(t, u_opt', '-o');
title('Optimal Control Inputs');
xlabel('Time Step');
ylabel('Inputs');
legend('u_1', 'u_2');
