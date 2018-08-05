% Cubic spline ankle simulation
close all
clear 
clc

% Initialize once
time = 1;
joint_ang = 14; % theta
theta_set_fsm = -10;
joint_v = 0; % velocity
tor_d = 0; % torque
% Impedance controller
k1 = .2; 
k2 = 0;
damp_b = .9;
% Motor
motor_t = 0;
inertia_motor_const = .00007307114; % kg*m^2
motor_a = motor_t / inertia_motor_const; 
N  = 60; % Gear ratio
motor_torque_const = .0954929; %Nm/amp
inertia_joint_const = 1; %.0018499; % kg*m^2
bD = 0;

% % Linear Spline
% res_factor = 100;
% x = [0 res_factor];
% y = [joint_ang theta_set_fsm]; 

% Cubic Spline
res_factor = 100; 
x = [0 (res_factor/2)*.4 res_factor/2];
yi_1 = joint_ang;
yf_1 = yi_1 + ((theta_set_fsm - yi_1)/2);
y_int_1 = yi_1 - ((yi_1 - yf_1) * .15);
y = [yi_1 y_int_1 yf_1];
k = cubicSpline(x, y);
% Compute ai and bi, Note a(1) and b(1) are 0, a1 is at a(2)...
n = length(k);
for i=2:n
    a(i) = k(i-1)*(x(i)-x(i-1)) - (y(i)-y(i-1));
    b(i) = -k(i)*(x(i)-x(i-1)) + (y(i)-y(i-1));
end
x2 = [res_factor/2 (res_factor-(res_factor/2))*.6+(res_factor/2) res_factor];
yi_2 = yi_1 + ((theta_set_fsm - yi_1)/2);
yf_2 = theta_set_fsm;
y_int_2 = yf_2 + ((yi_2 - yf_2) * .15);
y2 = [yi_2 y_int_2 yf_2];
k2 = cubicSpline(x2, y2);
% Compute ai and bi, Note a(1) and b(1) are 0, a1 is at a(2)...
n2 = length(k2);
for i=2:n2
    a2(i) = k2(i-1)*(x2(i)-x2(i-1)) - (y2(i)-y2(i-1));
    b2(i) = -k2(i)*(x2(i)-x2(i-1)) + (y2(i)-y2(i-1));
end

% Motor and joint initialization
joint_torque(time) = tor_d;
theta_set = joint_ang;
joint_angle(time) = joint_ang;
joint_vel(time) = joint_v;
Y(time) = theta_set;
X(time) = time;
time = time + 1;

while time < 100
%     % Instantaneous change
%     theta_set = theta_set_fsm;
    
%     % Linear Spline
%         yn = ((y(2)-y(1))*((time-1) - x(1)) / (x(2) - x(1))) + y(1);
%         theta_set = yn;
%    
%     if (y(1) - theta_set_fsm) > 0
%         if theta_set < theta_set_fsm % negative spline
%            theta_set = theta_set_fsm; 
%         end
%     else
%         if theta_set > theta_set_fsm % positive spline
%            theta_set = theta_set_fsm; 
%         end
%     end
    
    % Cubic Spline
    
        if time <= (res_factor/2)
            for i=2:n % TODO: Without for-loop for C code
                t = ((time-1) - x(i-1)) / (x(i)-x(i-1)); % x = time? YES
                q(i) = (1-t)*y(i-1) + t*y(i) ...
                    + (t*(1-t)*(a(i)*(1-t)+(b(i)*t)));
            end
            if time <= ((res_factor/2)*.4)
                theta_set = q(2);
            else
                theta_set = q(3);
            end
        else
            for i=2:n2 % TODO: Without for-loop for C code
                t = ((time-1) - x2(i-1)) / (x2(i)-x2(i-1)); % x = time? YES
                q2(i) = (1-t)*y2(i-1) + t*y2(i) ...
                    + (t*(1-t)*(a2(i)*(1-t)+(b2(i)*t)));
            end
            if time <= ((res_factor-(res_factor/2))*.6+(res_factor/2))
                theta_set = q2(2);
            else
                theta_set = q2(3);
            end
        end
        
    if (y(1) - theta_set_fsm) > 0
        if(theta_set < (theta_set_fsm)) % negative splines
            theta_set = theta_set_fsm;
        end
    else
        if(theta_set > (theta_set_fsm)) % positive splines
            theta_set = theta_set_fsm;
        end
    end
    
    % Motor and joint simulation
    tor_d = k1 * (theta_set - joint_ang) - (damp_b * joint_v);
    joint_torque(time) = tor_d;
    joint_a = (tor_d - (bD * joint_v)) / inertia_joint_const; % Convert to kg?
    joint_acc(time) = joint_a;
    
    motor_t = inertia_motor_const * N * joint_a + (tor_d/N);
    motor_torque(time) = motor_t;
    motor_a = (motor_t - (tor_d/N)) / inertia_motor_const;
    motor_acc(time) = (motor_a * (180/3.1415)) / 100;
    motor_jerk(time) = (diff(motor_acc(time-1:time)));
    
    joint_v = joint_vel(1) + trapz(joint_acc);
    joint_vel(time) = joint_v;
    joint_ang = joint_angle(1) + trapz(joint_vel); % doble integrate joint acc
    joint_angle(time) = joint_ang;
    Y(time) = theta_set;
    X(time) = time;
    
    time = time + 1;
end

%plot(X, joint_acc, 'LineWidth', 2);
%hold on
%plot(X, joint_vel, 'LineWidth', 2);
%hold on
plot(X, joint_angle, 'LineWidth', 3);
hold on
plot(X, Y, 'LineWidth', 3);
% hold on
% plot(X, motor_torque, 'LineWidth', 2);
% hold on
%plot(X, motor_acc/10, 'LineWidth', 2);
%hold on
%plot(X, motor_jerk, 'LineWidth', 2);
%legend('Joint Acc (deg/s/s)', 'Joint Angle (deg)', 'Theta des (deg)', 'Motor Torque', 'Motor Acc/10', 'Motor Jerk/10');
legend('Joint Angle (deg)', 'Trajectory (deg)');
xlabel('Time (ms)');
ylabel('Joint Angle (deg)');
title('Cubic Spline Trajectory');
