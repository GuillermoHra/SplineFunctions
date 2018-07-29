% Spline design
% Functions cubicSpline and solveTridiagonalMatrix are required
% Parameter to change: 
% joint_ang - initial joint angle
% theta_set_fsm - final joint angle
% firstf and secondf - cubic spline slope
% res_factor - delta X (time)
% Uncomment linear or cubic spline to use them
% ---------------------------------------------
% Biomechatronics group
% Guillermo Herrera-Arcos 
close all
clear
clc

% Initialize once
time = 1;
joint_ang = 14; % initial point
theta_set_fsm = -10; % final point

% % Linear Spline
% res_factor = 100;
% x = [0 res_factor];
% y = [joint_ang theta_set_fsm]; 

% Cubic Spline
firstf = .4; 
secondf = .6; 
res_factor = 100; % delta X (time)
x = [0 (res_factor/2)*firstf res_factor/2];
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
x2 = [res_factor/2 (res_factor-(res_factor/2))*secondf+(res_factor/2) res_factor];
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

% Initialization
theta_set = joint_ang;
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
            if time <= ((res_factor/2)*firstf)
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
            if time <= ((res_factor-(res_factor/2))*secondf+(res_factor/2))
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
   
    Y(time) = theta_set;
    X(time) = time;
    
    time = time + 1;
end

plot(X, Y, 'LineWidth', 3);

legend('Trajectory (deg)');
xlabel('Time (ms)');
ylabel('Joint Angle (deg)');
title('Trajectory');

