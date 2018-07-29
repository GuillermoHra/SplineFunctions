% Linear spline function
% Inputs: x and y vectors - initial and final points
% Inputs: h-resolution factor, n-number of points
% Outputs: X and Y vectors - vectors with linear spline points
%function [X, Y] = linearSpline(x, y)
    x = [0, 30]; % C-with structs
    y = [0, 14];
    % C-in a function
    index = 1;
    % If more than 2 points are required, another for is needed
    for time=x(1):x(2)
        yn = ((y(2)-y(1))*(time - x(1)) / (x(2) - x(1))) + y(1);
        Y(index) = yn;
        X(index) = time;
        index = index + 1;
    end
  
    % Plot
    plot(x(1), y(1), 'b*');
    hold on
    plot(x(2), y(2), 'b*');
    hold on
    plot(X, Y, 'r-');
    xlim([0 40]);
    ylim([-20 20]);
%end
