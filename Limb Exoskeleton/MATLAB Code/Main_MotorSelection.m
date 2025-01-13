clc;clear all;close all;
data = importdata('C:\Users\bbdiv\OneDrive\MAE263E Final Project\Code\finaldata.csv');
moment = data.data(:,3);
angle = data.data(:,2);
time = data.data(:,1);

% Plotting data

time_inter = linspace(time(1),time(end),100);
angle_inter = interp1(time,angle,time_inter);
moment_inter = interp1(time,moment,time_inter);

time=time_inter';

angle = smooth(angle_inter,0.07)';
moment= smooth(moment_inter,0.07)';

figure();
%Plot the original and smoothed ankle angle
subplot(2,1,1)
plot(time, angle, 'b','LineWidth',2);
xlabel('Time (s) ');
ylabel('Ankle Angle (degrees)');
title('Ankle Angle vs time');

subplot(2,1,2)
plot(time,moment , 'r', 'LineWidth',2);
title('Actuator Torque vs time');
xlabel('Time (s) ');
ylabel('Torque (Nm)');


%% Code for calculating current and voltage of motor 1,2,3 for a given transmission ratio

kt1 = 96.1*10^-3; R1=4.4; L1 = 0.937*10^-3; J1= 101*10^-3*10^-4; n1=0.82;%motor1
kt2 = 68.3*10^-3; R2=1.76; L2 = 0.658*10^-3; J2= 99.5*10^-3*10^-4;n2=0.9; %motor2
kt3 = 0.14; R3=0.186; L3 = 138*10^-6; J3= 0.12/1000; n3=0.9; %motor3
kt4 = 42.9*10^-3; R4=0.716; L4 = 0.26*10^-3; J4= 98.7*10^-3*10^-4; n4=0.9;%motor 4
kt5 = 58.5*10^-3; R5=1.4; L5 = 0.473*10^-3; J5= 75.9*10^-3*10^-4; n5=0.9;%motor 5
kt6 = 75.2*10^-3; R6=70.2; L6 = 6.68*10^-3; J6= 12.1*10^-3*10^-4; n6=0.83;%motor6 weaker motor

angle = deg2rad(angle);

T_l = moment; %load torque

dtheta = gradient(angle, time);
ddtheta = gradient(dtheta,time);

%% Calculate cost function
c=1; P1_arr=[]; P2_arr=[]; P3_arr=[];P4_arr=[]; P5_arr=[]; P6_arr=[];

for j=0:10:1000

[P1,i,ein1] = Power(kt1,ddtheta, dtheta, J1, j, n1,T_l, R1,L1,time);
[P2,i,ein2] = Power(kt2,ddtheta, dtheta, J2, j, n2,T_l, R2,L2,time);
[P3,i,ein3] = Power(kt3,ddtheta, dtheta, J3, j, n3,T_l, R3,L3,time);
[P4,i,ein4] = Power(kt4,ddtheta, dtheta, J4, j, n4,T_l, R4,L4,time);
[P5,i,ein5] = Power(kt5,ddtheta, dtheta, J5, j, n5,T_l, R5,L5,time);
[P6,i,ein6] = Power(kt6,ddtheta, dtheta, J6, j, n6,T_l, R6,L6,time);

%storing the integral of I2R
P1_arr = [P1_arr P1]; 
P2_arr = [P2_arr P2];
P3_arr = [P3_arr P3];
P4_arr = [P4_arr P4];
P5_arr = [P5_arr P5];
P6_arr = [P6_arr P6];

end

%% Plotting
figure();
hold on;
% Power I^2R
plot(0:10:1000, P1_arr, 'b', 'LineWidth', 2,'DisplayName','Motor1');
plot(0:10:1000, P2_arr, 'r', 'LineWidth', 2, 'DisplayName','Motor2');
plot(0:10:1000, P3_arr, 'g', 'LineWidth', 2,'DisplayName','Motor3');
plot(0:10:1000, P4_arr, 'k', 'LineWidth', 2,'DisplayName','Motor4');
plot(0:10:1000, P5_arr, 'c', 'LineWidth', 2, 'DisplayName','Motor5');
plot(0:10:1000, P6_arr, 'm', 'LineWidth', 2,'DisplayName','Motor6');

xlabel('Transmission ratio (N)');
ylabel('Intergral of I^2R');
title('Intergral of I^2R vs. Transmission ratio');
ylim([0 350]);
legend;
hold off;

%% Adding SEA
dT_l = gradient(T_l, time); P_SEA_arr1=[]; P_SEA_arr2=[]; P_SEA_arr3=[];
ddT_l = gradient(dT_l, time); %Update V&I function feed this

for j=10:10:1000    
    [P_SEA1, i, ein] = Power_SEA(kt3,ddtheta, dtheta, J3, 80, n3,T_l, R3,L3,time,j,ddT_l);
    P_SEA_arr1 = [P_SEA_arr1 P_SEA1];

    [P_SEA2, i, ein] = Power_SEA(kt3,ddtheta, dtheta, J3, 100, n3,T_l, R3,L3,time,j,ddT_l);
    P_SEA_arr2 = [P_SEA_arr2 P_SEA2];

    [P_SEA3, i, ein] = Power_SEA(kt3,ddtheta, dtheta, J3, 130, n3,T_l, R3,L3,time,j,ddT_l);
    P_SEA_arr3 = [P_SEA_arr3 P_SEA3];


end

%% Plotting
figure();

% Power I^2R
plot(10:10:1000, P_SEA_arr1, 'b', 'LineWidth', 2,'DisplayName','N=80');
hold on;
plot(10:10:1000, P_SEA_arr2, 'r', 'LineWidth', 2,'DisplayName','N=100');
plot(10:10:1000, P_SEA_arr3, 'g', 'LineWidth', 2,'DisplayName','N=130');
xlabel('Spring Stiffness (k)');
ylabel('Intergral of I^2R');
title('Intergral of I^2R vs Spring stiffness(k)');
ylim([0 20]);
legend;

%% Plotting current and voltage
figure;
[P_SEA1, i_SEA, ein_SEA] = Power_SEA(kt3,ddtheta, dtheta, J3, 100, n3,T_l, R3,L3,time,180,ddT_l);

subplot(2,1,1)
plot(time, i_SEA, 'b', 'LineWidth', 2,'DisplayName','current');
xlabel('Time(s)');
ylabel('Current(A)');
title('Current vs Time for N=100 SEA K=180');

subplot(2,1,2)
plot(time, ein, 'r', 'LineWidth', 2,'DisplayName','Volage');
xlabel('Time(s)');
ylabel('Voltage(V)');
title('Voltage vs Time for N=100 with SEA K=180');

BatteryCap = trapz(time,i_SEA.*ein_SEA);











