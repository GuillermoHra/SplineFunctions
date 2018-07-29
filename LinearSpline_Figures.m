% Figures for presentation (Linear Spline)
close all
clear
clc

data_table = readtable('Experiment32_conv.csv');
data = table2array(data_table);
data_table2 = readtable('Experiment34_conv.csv');
data2 = table2array(data_table2);

[ySize, xSize] = size(data);
[ySize2, xSize2] = size(data2);

% Joint angle and joint torque plot
joint_angle = data(:,33) / 100; %deg
joint_angle2 = data2(:,33) / 100; %deg
torque = data(:, 37); %Nm
torque2 = data2(:, 37); %Nm
torque = torque / 500; % to help visualization
torque2 = torque2 / 500; % to help visualization
%theta_des = data(:, 35) / 100; %deg, only enabled in fsm
current_state = data(:, 40);
current_state2 = data2(:, 40);
motor_acc = data(:, 16);
motor_acc2 = data2(:, 16);
motor_acc = motor_acc / 500; % to help visualization
motor_acc2 = motor_acc2 / 500; % to help visualization
%joint_vel = data(:, 12);

samples = [];
for i=1:ySize
    samples(i) = i;
end
samples2 = [];
for i=1:ySize2
    samples2(i) = i;
end

samples_temp = [];
for i=1:551
    samples_temp(i) = i;
end

%plot(samples_temp, joint_angle(5670:6220,1), 'LineWidth', 2);
%hold on
plot(samples_temp, motor_acc(5670:6220,1), 'LineWidth', 2);
hold on
plot(samples_temp, current_state(5670:6220,1), 'LineWidth', 4);

%hold on
%plot(samples_temp, joint_angle2(3710:4260,1), 'LineWidth', 2);
%hold on
plot(samples_temp, motor_acc2(3710:4260,1), 'LineWidth', 2);
hold on
plot(samples_temp, current_state2(3710:4260,1), 'LineWidth', 4);

ylim([-25 25]) % -10 30, -25 25
xlabel("Samples");
ylabel("Deg/s/s and State");
legend('Motor Acc (deg/s/s)', 'State', 'Motor Acc LS (deg/s/s)', 'State LS');
%title('FSM vs FSM w/ Linear Spline');