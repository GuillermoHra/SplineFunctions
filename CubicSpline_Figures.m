% Figures for presentation (Cubic Spline)
close all
clear
clc

data_table = readtable('Exp19_conv.csv'); 
data = table2array(data_table);
data_table2 = readtable('Exp18_conv.csv'); 
data2 = table2array(data_table2);

[ySize, xSize] = size(data);
[ySize2, xSize2] = size(data2);

% Joint angle and joint torque plot
joint_angle = data(:,35) / 100; %deg
des_joint_angle = data(:,36) / 100; %deg
motor_acc = ((data(:, 37)/100) * (180/3.1415)) / 100; %deg/s/s/100
motor_jerk = diff(motor_acc);

joint_angle2 = data2(:,35) / 100; %deg
des_joint_angle2 = data2(:,36) / 100; %deg
motor_acc2 = ((data2(:, 37)/100) * (180/3.1415)) / 100; %deg/s/s/100
motor_jerk2 = diff(motor_acc2);

% samples = [];
% for i=1:ySize
%     samples(i) = i;
% end
% samples2 = [];
% for i=1:ySize2
%     samples2(i) = i;
% end

samples = [];
for i=1:1001
    samples(i) = i;
end
point1 = 17930;
point2 = 27930;

plot(samples, joint_angle((point1-500):(point1+500)), 'LineWidth', 3);
hold on
plot(samples, des_joint_angle((point1-500):(point1+500)), 'LineWidth', 3);
hold on
plot(samples, motor_acc((point1-500):(point1+500)), 'LineWidth', 3);
hold on
plot(samples(1:1000), motor_jerk((point1-500):(point1+499)), 'LineWidth', 4);

% hold on
% plot(samples, joint_angle2((point2-500):(point2+500)), 'LineWidth', 3);
% %hold on
% %plot(samples, des_joint_angle2((point2-500):(point2+500)), 'LineWidth', 3);
% %hold on
% %plot(samples, motor_acc2((point2-500):(point2+500)), 'LineWidth', 3);
% hold on
% plot(samples(1:1000), motor_jerk2((point2-500):(point2+499)), 'LineWidth', 4);

ylim([-20 20]); %Acc-(-35)
xlim([400 750]);
xlabel("Samples");
%ylabel("Deg/s/s and State");
legend('Joint Angle CS2 (deg)', 'Des Joint Angle CS2 (deg)', 'Motor Acc CS2 (deg/s/s/100)', 'Motor Jerk CS2');
%title('FSM vs FSM w/ Linear Spline');