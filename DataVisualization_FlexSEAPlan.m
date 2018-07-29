% Ankle Prosthesis data visualization from FlexSEA plan csv file
close all
clear
clc

data_table = readtable('Exp19_conv.csv');
data = table2array(data_table);

% FlexSEA plan data collection frequency = 100Hz or 500Hz
[ySize, xSize] = size(data);
timeStamp = (data(end,1) - data(1,1)) / ySize;
freq = 1 / (timeStamp/1000); 

% Joint angle and joint torque plot
joint_angle = data(:,35) / 100; %deg
des_joint_angle = data(:,36) / 100; %deg
motor_acc = ((data(:, 37)/100) * (180/3.1415)) / 100; %deg/s/s/100
%motor_acc_test = data(:,16); 
joint_torque = data(:,33) / 100; %Nm
des_joint_torque = data(:,34) / 100; %Nm
motor_torque = data(:, 32) / 100;
motor_jerk = diff(motor_acc);
current_state = data(:, 37);

samples = [];
for i=1:ySize
    samples(i) = i;
end

% -----Plots-----
plot(samples, joint_angle, 'LineWidth', 3);
hold on
plot(samples, des_joint_angle, 'LineWidth', 3);
%hold on
%plot(samples, joint_torque, 'LineWidth', 3);
%hold on
%plot(samples, motor_torque, 'LineWIdth', 3);
hold on
plot(samples, motor_acc, 'LineWidth', 3);
hold on
plot(samples(1:ySize-1), motor_jerk, 'LineWidth', 4);
%hold on
%plot(samples, current_state, 'LineWidth', 4);

%ylim([-30 30])
xlabel("Samples");
legend('Joint angle (deg)', 'Des Joint Angle (deg)', 'Motor Acc (deg/s/s)', 'Motor Jerk');
%title('Exp1 (k1=150, b=.3)');

% % -----Moving average filter-----
% N = 100;
% coeff = ones(1,N)/N; % for N=50
% avgJointVel = filter(coeff, 1, joint_vel);
% fDelay = (N-1)/2;
% plot(samples, joint_vel, samples-fDelay/N, avgJointVel);
% axis tight
% legend('Joint Velocity','N=50 Average','location','best')
% ylabel('Velocity (Deg/s)')
% xlabel('Samples')

% % -----Low-pass Filtering joint velocity data-----
% Fs = freq;
% N = ySize,
% rng default;
% t = (0:N-1)/Fs;
% % Los-pass filter
% Fnorm = 40/(Fs/2);           % Normalized frequency
% df = designfilt('lowpassfir','FilterOrder',10,'CutoffFrequency',Fnorm);
% % delay
% grpdelay(df,2048,Fs) % plot group delay
% D = mean(grpdelay(df)) % filter delay in samples
% y = filter(df,[joint_vel; zeros(D,1)]); % Append D zeros to the input data
% y = y(D+1:end); % Shift data to compensate for delay
% figure
% plot(t,joint_angle,t,y,'r','linewidth',1.5);
% title('Filtered Joint Angle');
% xlabel('Time (s)')
% legend('Original ignal','Filtered Signal');
% grid on
% axis tight

% % -----linearSpline-----
% x = [];
% y = [];
% % Initial and final points are theta_des in each state
% x(1,1) = 8900;
% x(2,1) = 9270; 
% y(1,1) = joint_angle(8900, 1);
% y(2,1) = joint_angle(9270, 1);
% h = 1;
% [X, Y] = linearSpline(x,y,h);
% % Plot joint angle with linear spline
% index = 1;
% for i=8900:9270
%     joint_angle(i,1) = Y(1,index);
%     index = index + 1;
% end
% 
% x2 = [];
% y2 = [];
% % Initial and final points are theta_des in each state
% x2(1,1) = 10520;
% x2(2,1) = 10940; 
% y2(1,1) = joint_angle(10520, 1);
% y2(2,1) = joint_angle(10940, 1);
% h = 1;
% [X2, Y2] = linearSpline(x2,y2,h);
% % Plot joint angle with linear spline
% index = 1;
% for i=10520:10940
%     joint_angle(i,1) = Y2(1,index);
%     index = index + 1;
% end
% hold on
% plot(samples, joint_angle, 'LineWidth', 2.5);
% xlabel("Samples");
% ylabel("Degrees and Nm*100");
% legend('Joint angle (degrees)', 'Joint torque (Nm*100)', 'Joint angle w linear spline (degrees)');
% title('Experiment 1');




