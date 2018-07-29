% Function to solve tridiagonal matrices
% Inputs: e-subdiagonal vector, f-diagonal vector, g-upperdiagonal vector
% Inputs: r-Y values
% Output: k values

%e = [0 -1 -1 -1];
%f = [2.04 2.04 2.04 2.04];
%g = [-1 -1 -1];
%r = [40.8 .8 .8 200.8];

function k = solveTridiagonalMatrix(e, f, g, r)
    n = length(f);
    % Forward elimination
    for i = 2:n
        factor = e(i) / f(i-1);
        f(i) = f(i) - (factor * g(i-1));
        r(i) = r(i) - (factor * r(i-1));
    end
    % Back substitution
    k(n) = r(n) / f(n);
    for i = (n-1):-1:1
        k(i) = (r(i) - (g(i) * k(i+1))) / f(i);
    end
end