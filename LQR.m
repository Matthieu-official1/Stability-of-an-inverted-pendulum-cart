clc; clear; close all;
%% System Parameters " MATCHING PHYSICAL SYSTEM"
m_p = 0.04; % Pendulum mass (kg) m_c = 0.132; % Cart mass (kg) 
L = 0.2; % Pendulum length to center of mass (m)
b = 0.1; % Damping coefficient (N·s/m)
g = 9.81; % Gravitational acceleration (m/s^2)
I = (1/3)*m_p*L^2; % Moment of inertia of pendulum about pivot point
%% Linearized State-Space Model (around upright position)
% Denominator for simplification in dynamics expressions
den = I*(m_c + m_p) + m_c * m_p * (L^2)/4;
% State matrix A: system dynamics
A = [0 1 0 0;
    0 -((I + m_p*L^2/4)*b)/den (m_p^2 * g * L^2 / 4)/den 0;
    0 0 0 1;
    0 -m_p*L*b/(2*den) g*m_p*L*(m_c + m_p/2)/den 0];
% Input matrix B: how control input affects state
B = [0;
    (I + m_p*L^2/4)/den;
    0;
    (m_p*L/2)/den];
% Output matrix C: output all state variables
C = eye(4);
% Feedthrough matrix D: no direct feedthrough from input to output
D = zeros(4,1);
%% LQR Controller Design
% Define cost function weights
% Uncomment the one you'd like to use (examples shown for tuning)
%Q = diag([10, 10, 20, 10]); % Untuned
Q = diag([4, 2, 156, 82]); % Partially tuned
Q = diag([0.1000, 0.1002, 999.9890, 0.1003]); % Fully tuned
%Q = diag([4, 2, 156, 82]); % Partially tuned
R = 1; % Weight on control effort
% Compute optimal gain matrix K using LQR
K = lqr(A, B, Q, R);
%% Simulation Setup
dt = 0.01; % Time step (s)
T = 20; % Total simulation time (s)
t_span = 0:dt:T; % Time vector
x0 = [0; 0; 5*pi/180; 0]; % Initial condition: 5 degrees offset from upright
%% Simulate System using ODE45
[t, X] = ode45(@(t,x) pendulum_dynamics(t, x, A, B, K), t_span, x0);
% Log control input at each time step
U_log = -X * K'; % Matrix multiplication to get control input history
%% Extract and Convert Results
cart_pos = X(:,1); % Cart position (m)
pend_angle = X(:,3); % Pendulum angle (radians)
pend_angle_deg = pend_angle * 180 / pi; % Convert angle to degrees
%% Plot Results
figure(1);
subplot(3,1,1);
plot(t, cart_pos, 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Cart Position (m)');
title('Cart Position Over Time'); grid on;
subplot(3,1,2);
plot(t, pend_angle_deg, 'r', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Pendulum Angle (deg)');
title('Pendulum Angle Over Time'); grid on;
subplot(3,1,3);
plot(t, U_log, 'k', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Control Input (u)');
title('Control Input Over Time'); grid on;
%% Simple Animation of Cart and Pendulum
h_anim = figure(2);
% Drawing parameters
cart_width = 0.4;
cart_height = 0.2;
track_width = 2.0;
% Loop through simulation steps for animation
for k = 1:10:length(t)
    figure(h_anim); clf(h_anim); hold on; axis equal;
    x_cart = cart_pos(k); % Current cart position
    theta = pend_angle(k); % Current pendulum angle
    % Draw ground track
    plot([-track_width, track_width], [0, 0], 'k', 'LineWidth', 2);

    % Draw cart as a rectangle
    cart_x = x_cart - cart_width/2;
    rectangle('Position', [cart_x, 0, cart_width, cart_height], ...
        'FaceColor', [0.2 0.6 1], 'EdgeColor', 'k', 'LineWidth', 1.5);
    % Calculate pendulum tip position
    pend_tip_x = x_cart + L * sin(theta);
    pend_tip_y = cart_height + L * cos(theta);
    % Draw pendulum as a line
    line([x_cart, pend_tip_x], [cart_height, pend_tip_y], ...
        'Color', 'k', 'LineWidth', 3);
    % Set plot labels and limits
    xlim([-1.5, 1.5]); ylim([-0.2, 1.4]);
    xlabel('Position (m)'); ylabel('Height (m)');
    title(sprintf('Inverted Pendulum at t = %.2f sec | Angle = %.2f°', ...
        t(k), pend_angle_deg(k)), 'FontSize', 12);
    pause(dt); % Control animation speed
end
%% Dynamics Function for ODE Solver
function dxdt = pendulum_dynamics(~, x, A, B, K)
% Compute control input
u = -K * x;
% Compute state derivative using linearized model
dxdt = A*x + B*u;
end