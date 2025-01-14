% Parameters
l_string = 10;
N_string = 10; % Total length of the U-shape

% Generate U-shaped curve
theta = linspace(0, pi, 10); % Angle values from 0 to pi
x = l_string/2 * (1 - cos(theta)); % X-coordinate equation
y = l_string/2 * sin(theta); % Y-coordinate equation
z = zeros(size(theta)); % Z-coordinate is set to 0

% Plot the U-shaped curve
figure;
plot3(x, y, z, '-o', 'LineWidth', 2);
title('U-shaped Curve');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
grid on;
axis equal;