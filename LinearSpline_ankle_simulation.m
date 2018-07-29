    clear
    clc
    
    time = 1;
    theta_set_fsm = -10; % NEW
    
    % Impedance controller
    k1 = 1;
    k2 = 0;
    b = 0;
    
    theta_d = 0; % velocity
    joint_angle = 14; % theta
    theta_set = -10;
  
    % C-in a function
    X = [];
    Y = [];
    torque = [];
    angle = [];

    while time < 30
        x = [1, 4]; % Resolution factor? YES
        y = [joint_angle, theta_set_fsm];
        theta = joint_angle; % Joint angle
        yn = ((y(2)-y(1))*(time - x(1)) / (x(2) - x(1))) + y(1);
        Y(time) = yn;
        theta_set = yn;
        tor_d = k1 * (theta_set - theta); 
        torque(time) = tor_d;
        % motor simple simulation
        if joint_angle > theta_set_fsm
            joint_angle = joint_angle + tor_d; % For simulation purposes
            angle(time) = joint_angle;
        else
            joint_angle = joint_angle;
            angle(time) = joint_angle;
        end
        time = time + 1;
    end
    
%     % Plot
%     plot(x(1), y(1), 'b*');
%     hold on
%     plot(x(2), y(2), 'b*');
%     hold on
%     plot(X, Y, 'r-');
%     xlim([0 40]);
%     ylim([-20 20]);
