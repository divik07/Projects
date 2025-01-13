clc;clear all;close all;
data_moment = importdata('E:\UCLA Study Material\Winter 2024\Bionics Systems Engineering\HW5\inverse_dynamics.sto');
data_angle = importdata("E:\UCLA Study Material\Winter 2024\Bionics Systems Engineering\HW5\subject1_IK.mot");
moment_hip = data_moment.data(:,5);
moment_knee = data_moment.data(:,8);
moment_ankle = data_moment.data(:,10);

time = data_angle.data(37:109,1);
angle_hip = data_angle.data(37:109, 5);
angle_knee = data_angle.data(37:109, 6);
angle_ankle = data_angle.data(37:109, 7);

%% Plotting joint angles and moments
% figure;
% 
% Right hip flexion/extension angle
% subplot(3, 2, 1);
% plot(time_angle, angle_hip, 'b');
% title('Right Hip Flexion/Extension Angle');
% xlabel('Time (s)');
% ylabel('Angle (degrees)');
% 
% Right knee flexion/extension angle
% subplot(3, 2, 3);
% plot(time_angle, angle_knee, 'b');
% title('Right Knee Flexion/Extension Angle');
% xlabel('Time (s)');
% ylabel('Angle (degrees)');
% 
% Right ankle flexion/extension angle
% subplot(3, 2, 5);
% plot(time_angle, angle_ankle, 'b');
% title('Right Ankle Flexion/Extension Angle');
% xlabel('Time (s)');
% ylabel('Angle (degrees)');
% 
% Right hip flexion/extension moment
% subplot(3, 2, 2);
% plot(time_moment, moment_hip, 'r');
% title('Right Hip Flexion/Extension Moment');
% xlabel('Time (s)');
% ylabel('Moment (N-m)');
% 
% Right knee flexion/extension moment
% subplot(3, 2, 4);
% plot(time_moment, moment_knee, 'r');
% title('Right Knee Flexion/Extension Moment');
% xlabel('Time (s)');
% ylabel('Moment (N-m)');
% 
% Right ankle flexion/extension moment
% subplot(3, 2, 6);
% plot(time_moment, moment_ankle, 'r');
% title('Right Ankle Flexion/Extension Moment');
% xlabel('Time (s)');
% ylabel('Moment (N-m)');
% 
% Adjust subplot spacing
% sgtitle('Joint Angle and Moment Trajectories');

%% Interpolating ankle angle
% Define the number of desired data points
num_points = 200; %frequency is 200/1.2s = 166.66

% Interpolate the ankle angle to 200 data points
time_interpolated = linspace(time(1), time(end), num_points);
angle_ankle_interpolated = interp1(time, angle_ankle, time_interpolated);
moment_ankle_interpolated = interp1(time, moment_ankle, time_interpolated);

windowsize = 1/0.006*0.05;
angle_ankle_interpolated = deg2rad(angle_ankle_interpolated); %converting deg 2 rad
smoothed_angle_ankle = smooth(angle_ankle_interpolated, windowsize);

%Plot the original and smoothed ankle angle
plot(time_interpolated, angle_ankle_interpolated, 'b', 'DisplayName', 'Original');
hold on;
plot(time_interpolated, smoothed_angle_ankle, 'r', 'DisplayName', 'interplocated');
title('Original vs Smoothed Right Ankle Flexion/Extension Angle');
xlabel('Time (s) ');
ylabel('Angle (degrees)');
legend;

%% Code for calculating current and voltage of motor 1,2,3 for a given transmission ratio
N=135; n=0.88;%efficiency and transmission ratio

kt1 = 96.1*10^-3; R1=4.4; L1 = 0.937*10^-3; J1= 101*10^-3*10^-4; %motor1
kt2 = 68.3*10^-3; R2=1.76; L2 = 0.658*10^-3; J2= 99.5*10^-3*10^-4; %motor2
kt3 = 0.14; R3=0.186; L3 = 138*10^-6; J3= 0.12/1000; %motor3

T_l = moment_ankle_interpolated'; %load torque

time = time_interpolated; %time

dtheta = gradient(smoothed_angle_ankle, time);
ddtheta = gradient(dtheta,time);

%If spring is added 
k=10000; dT_l = gradient(T_l, time);
ddT_l = gradient(dT_l, time); %Update V&I function feed this


[i1,ein1]= IandV_sea(kt3,ddtheta, dtheta, J3, N, n,T_l, R3,L3,time,k, ddT_l); %sea current and voltage
[i2,ein2]= IandV(kt3,ddtheta, dtheta, J3, N, n,T_l, R3,L3,time); %sea current and voltage
c=1;
for i=60:10:1000
[i1,ein1] = IandV_sea(kt3,ddtheta, dtheta, J3, N, n,T_l, R3,L3,time,k, ddT_l)
c=c+1;
end

% [i11,ein11] = IandV_sea(kt3,ddtheta, dtheta, J3, 75, n,T_l, R3,L3,time,150, ddT_l);
% [i21,ein21] = IandV_sea(kt3,ddtheta, dtheta, J3, 95, n,T_l, R3,L3,time,220, ddT_l);
% [i31,ein31] = IandV_sea(kt3,ddtheta, dtheta, J3, 135, n,T_l, R3,L3,time,300, ddT_l);


[P1,H1] = Power(kt3,ddtheta, dtheta, J3, 95, n,T_l, R3,L3,time,220, ddT_l);

figure(3);
% Plot current for motor1
subplot(2, 1, 1);
plot(time, P1, 'b', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Power (W)');
title('Power vs. Time ');

% Plot voltage for motor1
subplot(2, 1, 2);
plot(time, H1, 'r', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Heat (W)');
title('Heat vs. time');

% % Plot current for motor1
% subplot(2, 1, 1);
% plot(time, i1, 'b', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Current (A)');
% title('Current vs. Time with SEA spring');
% 
% % Plot voltage for motor1
% subplot(2, 1, 2);
% plot(time, i2, 'r', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Current (V)');
% title('Voltage vs. time without SEA spring');
% 
% % Adjust subplot spacing
% sgtitle('Current Trajectories vs. Time');

% Plotting cost function, I/V vs transmission ratio
% k=1; %counter
% for j=1:1:1000
% [P1,ein1] = Power(kt1,ddtheta, dtheta, J1, j, n,T_l, R1,L1,time); %motor1
% [P2,ein2] = Power(kt2,ddtheta, dtheta, J2, j, n,T_l, R2,L2,time); %motor2
% [P3,ein3] = Power(kt3,ddtheta, dtheta, J3, j, n,T_l, R3,L3,time); %motor3
% P1_array(k)= P1; %appending integral of Power in array for motor1
% P2_array(k)= P2; %appending integral of Power in array for motor2
% P3_array(k)= P3; %appending integral of Power in array for motor3
% 
% ein1_array(k) = max(abs(ein1));
% ein2_array(k) = max(abs(ein2));
% ein3_array(k) = max(abs(ein3));
% k=k+1;
% end
% 
% 
% figure(2)
% Plot cost function(heat loss)
% hold on;
% plot(1:1:1000, P1_array, 'r', 'LineWidth', 2, 'DisplayName', "Motor1");
% plot(1:1:1000, P2_array, 'b', 'LineWidth', 2, 'DisplayName', "Motor2");
% plot(1:1:1000, P3_array, 'g', 'LineWidth', 2, 'DisplayName', "Motor3");
% xlabel('Transmission Ratio(N)');
% ylabel('Integral of i^2R');
% title('Current vs. Transmission Ratio ');
% legend;
% ylim([0, 200]);
% 
% Plotting max voltage
% figure(3);
% hold on;
% plot(1:1:1000, ein1_array, 'r', 'LineWidth', 2, 'DisplayName', "Motor1");
% plot(1:1:1000, ein2_array, 'b', 'LineWidth', 2, 'DisplayName', "Motor2");
% plot(1:1:1000, ein3_array, 'g', 'LineWidth', 2, 'DisplayName', "Motor3");
% xlabel('Transmission Ratio(N)');
% ylabel('Max Voltage');
% title('Max Voltage vs. Transmission Ratio ');
% legend;
% ylim([0, 200]);
% 
% 


