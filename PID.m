clc; clear; close all;
%% System Parameters (updated to match your LQR setup)
M = 0.132; % Cart mass (kg)
m = 0.2; % Pendulum mass (kg)
b = 0.1; % Cart damping (N/m/s)
L = 0.2; % Full rod length (m)
g = 9.81; % Gravity (m/s^2)
I = (1/3)*m*L^2; % Moment of inertia (uniform rod about pivot)
% Denominator term for transfer functions
q = (M + m)*(I + m*L^2) - (m*L)^2;
% Transfer function variable
s = tf('s');
% Transfer function: Force → Pendulum Angle
P_pend = (m*L*s/q) / (s^3 + (b*(I + m*L^2))*s^2/q - ((M + m)*m*g*L)*s/q - b*m*g*L/q);
% Transfer function: Force → Cart Position
P_cart = ((I + m*L^2)*s^2 - m*g*L) / q / (s^4 + (b*(I + m*L^2))*s^3/q - ((M + m)*m*g*L)*s^2/q - b*m*g*L*s/q);
%% PID Controller (only controlling pendulum angle)
Kp = 100;
Ki = 1;
Kd = 1;
C = pid(Kp, Ki, Kd);
% Closed-loop transfer functions
T_pend = feedback(P_pend, C); % Pendulum angle
T_cart = feedback(P_cart, C); % Cart position
% Time vector
t = 0:0.01:10;
% Simulate step responses
[y_pend, ~] = step(T_pend, t);
[y_cart, ~] = step(T_cart, t);
% Convert angle to degrees for animation
y_pend_deg = y_pend * 180 / pi;
%% Plot Results
figure(1);
subplot(2,1,1);
plot(t, y_pend * 180 / pi, 'r', 'LineWidth', 2);
title('Pendulum Angle Response (PID)');
xlabel('Time (s)'); ylabel('Angle (deg)'); grid on;
subplot(2,1,2);
plot(t, y_cart, 'b', 'LineWidth', 2);
title('Cart Position Response (PID)');
xlabel('Time (s)'); ylabel('Position (m)'); grid on;
%% Animation (uses separate figure)
cart_width = 0.3;
cart_height = 0.15;
track_width = 1.5;
h_anim = figure(2);
set(h_anim, 'Name', 'PID-Controlled Inverted Pendulum Animation');
for k = 1:10:length(t)
figure(h_anim); clf(h_anim); hold on; axis equal;
x_cart = y_cart(k); % Cart position
theta = y_pend(k); % Pendulum angle
% Track
plot([-track_width, track_width], [0, 0], 'k', 'LineWidth', 2);
% Cart
cart_x = x_cart - cart_width/2;
rectangle('Position', [cart_x, 0, cart_width, cart_height], 'FaceColor', [0.2 0.6 1], 'EdgeColor', 'k', 'LineWidth', 1.5);
% Pendulum (rod)
pend_tip_x = x_cart + L * sin(theta);
pend_tip_y = cart_height + L * cos(theta);
line([x_cart, pend_tip_x], [cart_height, pend_tip_y], 'Color', 'k', 'LineWidth', 3);
% Labels and limits
xlim([-1.2, 1.2]); ylim([-0.2, 1.4]);
xlabel('Position (m)'); ylabel('Height (m)');
% Inside animation loop
title(sprintf('Inverted Pendulum at t = %.2f sec | Angle = %.2f°', t(k), y_pend_deg(k)), 'FontSize', 12);
pause(0.01);
end