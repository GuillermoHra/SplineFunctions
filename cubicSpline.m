% Cubic Spline function
% Inputs: x vector, y vector
% Outputs: k values

%x = [0 21 30];
%y = [14 8 -10];

function k = cubicSpline(x, y)

% Fill matrix, TODO: how obtain values?
B(1) = 2 / (x(2) - x(1)); %B(0)
B(2) = 2*((1/(x(2)-x(1))) + (1/(x(3)-x(2)))); %B(1)
B(3) = 2 / (x(3)-x(2)); %B(2)
A(1) = 1 / (x(2)-x(1)); %A(0)
A(2) = 1 / (x(3)-x(2)); %A(1)
C(1) = 1 / (x(2)-x(1)); %C(0)
C(2) = 1 / (x(3)-x(2)); %C(1)
r(1) = 3*((y(2)-y(1))/(power(x(2)-x(1),2))); %r(0)
r(2) = 3*(((y(2)-y(1))/(power(x(2)-x(1),2))) + ((y(3)-y(2))/(power(x(3)-x(2),2)))); %r(1)
r(3) = 3*((y(3)-y(2))/(power(x(3)-x(2),2))); %r(2)

e = [0 A(1) A(2)];
f = [B(1) B(2) B(3)];
g = [C(1) C(2)];

% Solve tridiagonal matrix
k = solveTridiagonalMatrix(e,f,g,r);

end